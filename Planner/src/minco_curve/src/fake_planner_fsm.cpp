#include "fake_planner_fsm.h"

namespace fake_planner
{

// bool  have_plan_traj_ = false;
// bool have_plan_traj_1 = false;
void FakeReplanFSM::init(ros::NodeHandle &nh)
{
    node_ = nh;

    nh.param("fsm/flight_type", target_type_, -1);
    node_.param("fsm/replan_thresh", replan_thresh_, 0.5);
    node_.param("fsm/planning_horizen", planning_horizen_, 8.0);
    node_.param("fsm/emergency_stop_time", emergency_stop_time_, 3.0);
    node_.param("fsm/target_vel_x", target_vel_(0), 0.0);
    node_.param("fsm/target_vel_y", target_vel_(1), 0.0);
    node_.param("fsm/target_vel_z", target_vel_(2), 0.0);
    node_.param("fsm/target_acc_x", target_acc_(0), 0.0);
    node_.param("fsm/target_acc_y", target_acc_(1), 0.0);
    node_.param("fsm/target_acc_z", target_acc_(2), 0.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
      for (int i = 0; i < waypoint_num_; i++)
      {
          nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
          nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
          nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
      }

    planner_manager_.reset(new FakePlanManager);
    planner_manager_->initPlanModules(node_);

    exec_state_ = INIT;
    have_odom_ = false;
    have_traj_ = false;
    touch_goal_ = false;
    current_wp_idx_ = 0;

    exec_timer_ = node_.createTimer(ros::Duration(0.02), &FakeReplanFSM::execFSMCallback, this);
    // safety_timer_ = node_.createTimer(ros::Duration(0.05), &FakeReplanFSM::checkCollisionCallback, this);

    odom_sub_ = node_.subscribe("/odom_world", 10, &FakeReplanFSM::odometryCallback, this);
    // trigger_sub_ = node_.subscribe("trigger", 1, &FakeReplanFSM::triggerCallback, this);
    poly_traj_pub_ = node_.advertise<traj_utils::PolyTraj>("/fake_planner_node/poly_traj", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET){
      waypoint_list_.clear();
      trigger_sub_ = node_.subscribe("/move_base_simple/goal_mine", 1, &FakeReplanFSM::triggerCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      waypoint_list_.clear();
      for (int i = 0; i < waypoint_num_; i++) {
          Eigen::Vector3d wp(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
          waypoint_list_.push_back(wp);
      }
      // target_pt_ = waypoint_list_.back();
      current_wp_idx_ = 0;
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      trigger_ = true;
      // planToGivenWps(waypoint_list_);
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
      
    ROS_INFO("FSM initialized, waiting for odom and target.");
    nh.param("fsm/predict_dt", predict_dt_, 0.1);      
    nh.param("fsm/use_kalman_filter", use_kalman_filter_, true);
      
    if (use_kalman_filter_) {
        kf_ = ConstantVelocityKalmanFilter(0.2,0.8,0.05,0.05); 
    }

    // parameters for rising
    consecutive_replan_cnt_ = 0;
    rising_traj_generated_ = false;

    // Odometry jump detection
    has_last_odom_ = false;
    odom_jumped_ = false;
    odom_jump_time_ = ros::Time(0);
    node_.param("fsm/odom_jump_thresh", odom_jump_thresh_, 0.5);     // >0.5m jump is abnormal
    node_.param("fsm/odom_jump_cooldown", odom_jump_cooldown_, 0.5); // wait 0.5s after jump

    // DWA
    node_.param("fsm/use_dwa",use_dwa_,false);
    dynamic_obs_sub_ = node_.subscribe("/have_dynamic_obstacle", 1, &FakeReplanFSM::dynamicObstacleCallback, this);
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    have_dynamic_obs_ = false;
    current_yaw_ = 0.0;
    current_angular_z_ = 0.0;

    // //class
    // plan_sub_ = nh.subscribe("/plan_trajectory", 1, &FakeReplanFSM::planCallback, this);
}

// void FakeReplanFSM::planCallback(const std_msgs::Bool& msg){
//   have_plan_traj_ = msg.data;
// }

void FakeReplanFSM::dynamicObstacleCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(!use_dwa_)return;
    have_dynamic_obs_ = msg->data;

    if (have_dynamic_obs_) {
        if (exec_state_ != DWA && exec_state_ != EMERGENCY_STOP && exec_state_ != RISING) {
            have_traj_ = false;
            changeFSMExecState(DWA, "Dynamic obstacle detected, switch to DWA");
        }
    } else {
        if (exec_state_ == DWA) {
            changeFSMExecState(WAIT_TRAJ, "No dynamic obstacle, resume trajectory tracking");
            if (!waypoint_list_.empty() && current_wp_idx_ < (int)waypoint_list_.size()) {
                trigger_ = true;
                have_traj_ = false;
            }
        }
    }
}

void FakeReplanFSM::publishDWACommand(double v, double w)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_vel_pub_.publish(cmd);
}

void FakeReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    if (exec_state_ == EMERGENCY_STOP)
    {
      return;
    }

    switch (exec_state_)
    {
      case INIT:{
      if(! have_odom_)goto force_return;
      if(! trigger_)goto force_return;
        changeFSMExecState(WAIT_TRAJ, "INIT -> WAIT_TRAJ");
        break;
      }

      case WAIT_TRAJ:{
        if(! trigger_ || ! have_odom_) goto force_return;
          // Skip replanning during odom jump cooldown period
          if (odom_jumped_ && (ros::Time::now() - odom_jump_time_).toSec() < odom_jump_cooldown_) {
            ROS_WARN_THROTTLE(0.2, "Odom jump cooldown, waiting %.1fs before replanning",
                              odom_jump_cooldown_ - (ros::Time::now() - odom_jump_time_).toSec());
            goto force_return;
          }
          if (odom_jumped_) {
            odom_jumped_ = false;  // cooldown passed, clear flag
          }
          if(!waypoint_list_.empty() && current_wp_idx_ < (int)waypoint_list_.size()){
            bool ok = planToTarget(waypoint_list_[current_wp_idx_]);
            if (ok) {
                changeFSMExecState(EXEC_TRAJ, "WAIT_TRAJ -> EXEC_TRAJ");
            }
          }else{
            goto force_return;
          }
        break;
      }
    
      case EXEC_TRAJ:{
          if(!have_traj_) {
              ROS_WARN("EXEC_TRAJ without valid trajectory, switching to WAIT_TRAJ");
              changeFSMExecState(WAIT_TRAJ, "Missing traj in EXEC_TRAJ");
              break;
          }
          bool touch_the_goal =  (odom_pos_ - waypoint_list_[current_wp_idx_]).norm() < replan_thresh_; 
          //ROS_INFO("Reached preset waypoint ");
          if(touch_the_goal){
              ROS_INFO_THROTTLE(1.0,"Reached waypoint %d",current_wp_idx_);
              current_wp_idx_++;
              have_traj_ = false;
            if(current_wp_idx_ < (int)waypoint_list_.size()){
              // 有航点未完成
              ROS_INFO_THROTTLE(1.0,"have not finished waypoint");
              changeFSMExecState(WAIT_TRAJ,"EXEC_TRAJ -> WAIT_TRAJ (next waypoint)");
            }else{
              ROS_INFO("ALL waypoints reached!");
              touch_goal_ = true;
              trigger_ = false;
              have_traj_ = false;
              changeFSMExecState(WAIT_TRAJ, "EXEC_TRAJ -> WAIT_TRAJ (finished)");
            }
          }else{
            ROS_INFO_THROTTLE(1.0,"jump to exec the current traj");
              // changeFSMExecState(WAIT_TRAJ,"plan to waypoint");
                static ros::Time last_replan_time = ros::Time::now();
              if (planner_manager_->needRePlan() && (ros::Time::now() - last_replan_time).toSec()>0.02) {   
                ROS_WARN_THROTTLE(1.0,"Need Replan");
                if((ros::Time::now() - last_replan_time).toSec()<0.04){
                    consecutive_replan_cnt_++;
                }else if((ros::Time::now() - last_replan_time).toSec() > 1.2){
                    consecutive_replan_cnt_ = 0;
                }
                last_replan_time = ros::Time::now();
                ROS_WARN("Current consecutive_replan_cnt = %d",consecutive_replan_cnt_);
                if (consecutive_replan_cnt_ >= 30) {
                  rise_target_ = odom_pos_;
                  rise_target_.z() += 2.0;
                  changeFSMExecState(RISING, "Too many replans, rising");
                }else{
                  have_traj_ = false;
                  changeFSMExecState(WAIT_TRAJ, "Need replan due to collision");
                }
            }
          }
        break;
      }
      case RISING: {
        if (!rising_traj_generated_){
        
        bool success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, odom_acc_,
                                                        rise_target_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        if (success) {
            traj_utils::PolyTraj poly_msg;
            planner_manager_->polyTraj2ROSMsg(poly_msg);
            publishTraj(poly_msg);
            have_traj_ = true;
            rising_traj_generated_ = true;
        } else {
            ROS_ERROR("Failed to generate rising trajectory, emergency stop?");
            changeFSMExecState(EMERGENCY_STOP, "Rising failed");
        }
      }
        else{
        if ((odom_pos_ - rise_target_).norm() < replan_thresh_) {
            consecutive_replan_cnt_ = 0;   
            have_traj_ = false;
            rising_traj_generated_ = false;
            changeFSMExecState(WAIT_TRAJ, "Rising completed, resume original target");
        }
      }
        break;
    }
case DWA: {
    if (!have_dynamic_obs_ || waypoint_list_.empty()) {
        changeFSMExecState(WAIT_TRAJ, "DWA -> WAIT_TRAJ (no obstacle or no target)");
        ROS_WARN("NO OBS or NO TARGET");
        break;
    }
    if(odom_vel_.norm() < 0.05){
        changeFSMExecState(WAIT_TRAJ, "DWA -> WAIT_TRAJ (no obstacle or no target)");
        ROS_WARN("TOO SLOW");
        break;
    }

    std::vector<double> pose = {odom_pos_.x(), odom_pos_.y(),  current_yaw_, odom_pos_.z()};
    double v_c = odom_vel_.norm(); 
    double dynamic_safe_radius = 0.6;
    ROS_WARN("STATE 1");
    auto velocity = planner_manager_->getDWAcmd(pose, v_c, current_angular_z_,dynamic_safe_radius);
    ROS_WARN("We got the vel cmd !");
    publishDWACommand(velocity.v,velocity.w);

    if ((odom_pos_ - waypoint_list_[current_wp_idx_]).norm() < replan_thresh_) {
        if (current_wp_idx_ < (int)waypoint_list_.size() - 1) {
            current_wp_idx_++;
            have_traj_ = false;
            changeFSMExecState(WAIT_TRAJ, "DWA reached waypoint, go next");
        } else {
            ROS_INFO("Final target reached in DWA mode");
            touch_goal_ = true;
            trigger_ = false;
            have_traj_ = false;
            changeFSMExecState(WAIT_TRAJ, "DWA finished");
        }
    }
        break;
    }
      default:
        break;
      }
    force_return:;
    exec_timer_.start();
}

bool FakeReplanFSM::planToTarget(const Eigen::Vector3d &target_pt) {
    Eigen::Vector3d start_pt, start_vel;
    
    // if (use_kalman_filter_ && have_odom_) {
    //     start_pt = kf_.getPos();
    //     start_vel = kf_.getVel();
    //     ROS_DEBUG("Kalman predicted position: [%.2f, %.2f, %.2f]",
    //               start_pt.x(), start_pt.y(), start_pt.z());
    // } else {
    //     start_pt = odom_pos_;
    //     start_vel = odom_vel_;
    // }

    start_pt = odom_pos_ + odom_vel_ * predict_dt_;
    ROS_WARN("STARTPOINT x = %f,y = %f,z = %f",start_pt[0],start_pt[1],start_pt[2]);
    start_vel = odom_vel_;
    start_vel = start_vel * 0.5;
    
    Eigen::Vector3d start_acc = odom_acc_; 
    Eigen::Vector3d end_vel, end_acc;
    
    bool success = false;
    if (current_wp_idx_ == waypoint_list_.size() - 1) {
        success = planner_manager_->planGlobalTraj(start_pt, start_vel, start_acc,
                                                   target_pt, target_vel_, target_acc_);
    } else {
        end_vel.setZero();
        end_acc.setZero();
        success = planner_manager_->planGlobalTraj(start_pt, start_vel, start_acc,
                                                   target_pt, end_vel, end_acc);
    }
    
    if (success) {
        traj_utils::PolyTraj poly_msg;
        planner_manager_->polyTraj2ROSMsg(poly_msg);
        publishTraj(poly_msg);
        have_traj_ = true;
    }
    
    return success;
}

void FakeReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{
  if (exec_state_ == EMERGENCY_STOP) return;

}

void FakeReplanFSM::emergencyStop()
{
  if (exec_state_ != EMERGENCY_STOP)
  {
    changeFSMExecState(EMERGENCY_STOP, "Emergency stop");

  }
}

void FakeReplanFSM::triggerCallback(const nav_msgs::PathPtr &msg)
{
  if (exec_state_ == EMERGENCY_STOP)
  {
    changeFSMExecState(WAIT_TRAJ, "Resume from emergency stop");
  }
  if(have_traj_){
    ROS_WARN("On our way to current target, set goal later");
    return;
  }
    target_pt_ << msg -> poses[0].pose.position.x,
                  msg -> poses[0].pose.position.y,
                  msg -> poses[0].pose.position.z;            
  
  waypoint_list_.clear();
  waypoint_list_.push_back(target_pt_);
  current_wp_idx_ = 0;
  trigger_ = true;
}

void FakeReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  Eigen::Vector3d new_pos;
  new_pos(0) = msg->pose.pose.position.x;
  new_pos(1) = msg->pose.pose.position.y;
  new_pos(2) = msg->pose.pose.position.z;

  // Detect odometry jump (VIO/SLAM relocalization)
  if (has_last_odom_ && have_odom_) {
    double jump_dist = (new_pos - last_odom_pos_).norm();
    // Estimate max expected displacement from velocity (dt ~ 0.02s at 50Hz odom)
    double max_expected = odom_vel_.norm() * 0.1 + 0.1; // velocity * time + margin
    if (jump_dist > odom_jump_thresh_ && jump_dist > max_expected) {
      ROS_ERROR("ODOM JUMP DETECTED! jump=%.3f m, last_pos=[%.2f,%.2f,%.2f], new_pos=[%.2f,%.2f,%.2f]",
                jump_dist, last_odom_pos_.x(), last_odom_pos_.y(), last_odom_pos_.z(),
                new_pos.x(), new_pos.y(), new_pos.z());
      odom_jumped_ = true;
      odom_jump_time_ = ros::Time::now();
      // Reset Kalman filter on jump to avoid stale state
      if (use_kalman_filter_) {
        kf_ = ConstantVelocityKalmanFilter(0.2, 0.8, 0.05, 0.05);
      }
    }
  }

  last_odom_pos_ = new_pos;
  has_last_odom_ = true;

  odom_pos_ = new_pos;
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  current_yaw_ = tf::getYaw(q);
  current_angular_z_ = msg->twist.twist.angular.z;

  odom_acc_.setZero();
  have_odom_ = true;

  if (use_kalman_filter_) {
      double t = ros::Time::now().toSec();
      kf_.update(odom_pos_, odom_vel_, t);
  }
}

void FakeReplanFSM::publishTraj(const traj_utils::PolyTraj &traj_msg)
{
  poly_traj_pub_.publish(traj_msg);
}

void FakeReplanFSM::printFSMExecState()
{
  const char *state_names[] = {"INIT", "WAIT_TRAJ", "EXEC_TRAJ", "RISING","DWA","EMERGENCY_STOP"};
  ROS_INFO("FSM State: %s", state_names[exec_state_]);
}

void FakeReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, const std::string &pos_call)
{
  if (exec_state_ == new_state) return;
//   ROS_INFO("[%s] -> %s", pos_call.c_str(), exec_state_.c_str());
  exec_state_ = new_state;
  printFSMExecState();
}


} // namespace fake_planner