#include "fake_planner_fsm.h"

namespace fake_planner
{

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
      trigger_sub_ = node_.subscribe("/move_base_simple/goal", 1, &FakeReplanFSM::triggerCallback, this);
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
              if (planner_manager_->needRePlan() && (ros::Time::now() - last_replan_time).toSec()>0.0) {   
                ROS_WARN_THROTTLE(1.0,"Need Replan");
                if((ros::Time::now() - last_replan_time).toSec()<0.04){
                    consecutive_replan_cnt_++;
                }else if((ros::Time::now() - last_replan_time).toSec() > 0.1){
                    consecutive_replan_cnt_ = 0;
                }
                last_replan_time = ros::Time::now();
                ROS_WARN("Current consecutive_replan_cnt = %d",consecutive_replan_cnt_);
                if (consecutive_replan_cnt_ >= 30) {
                  rise_target_ = odom_pos_;
                  rise_target_.z() += 5.0;
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

// bool FakeReplanFSM::planToTarget(const Eigen::Vector3d &target_pt)
// {
//   Eigen::Vector3d start_pt = odom_pos_;
//   Eigen::Vector3d start_vel = odom_vel_;

//   Eigen::Vector3d start_acc = odom_acc_;  

//   Eigen::Vector3d end_vel = Eigen::Vector3d{2.0,0.0,0.0};
//   Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

//   bool success = false;
//   if(current_wp_idx_ == waypoint_list_.size() - 1){
//       success = planner_manager_->planGlobalTraj(start_pt,start_vel,start_acc,target_pt,target_vel_,target_acc_);
//     }
//   else{
//       success = planner_manager_->planGlobalTraj(start_pt,start_vel,start_acc,target_pt,end_vel,end_acc);
//   }

//   if (success)
//   {
//     traj_utils::PolyTraj poly_msg;
//     planner_manager_->polyTraj2ROSMsg(poly_msg);
//     publishTraj(poly_msg);
//     have_traj_ = true;
//   }

//   return success;
// }

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
    ROS_WARN("On out way to current target, set goal later");
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
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;
  odom_acc_.setZero();
  have_odom_ = true;
  // ROS_WARN_THROTTLE(0.5,"Current speed is : %f",odom_vel_.norm());
  if (use_kalman_filter_) {
      // 使用当前观测更新滤波器
      ROS_WARN_THROTTLE(2.0,"USE Kalman filter");
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
  const char *state_names[] = {"INIT", "WAIT_TRAJ", "EXEC_TRAJ", "RISING","EMERGENCY_STOP"};
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