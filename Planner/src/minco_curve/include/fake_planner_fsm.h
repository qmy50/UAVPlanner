# ifndef _FAKE_PLANNER_FSM_
# define _FAKE_PLANNER_FSM_


#include <utils/poly_traj_utils.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manager.h>
#include <traj_utils/PolyTraj.h>
#include <plan_env/grid_map.h>

using std::vector;

namespace fake_planner
{

 class ConstantVelocityKalmanFilter {
public:
    // 状态向量: [px, py, pz, vx, vy, vz]^T
    // 测量向量: [px, py, pz, vx, vy, vz]^T
    ConstantVelocityKalmanFilter(double process_noise_pos = 0.1,
                                 double process_noise_vel = 0.3,
                                 double measure_noise_pos = 0.05,
                                 double measure_noise_vel = 0.05) {
        // 状态转移矩阵 F (初始占位，实际 dt 会动态更新)
        F_.setIdentity();
        // 测量矩阵 H (直接测量全部状态)
        H_.setIdentity();
        // 过程噪声协方差 Q
        Q_.setIdentity();
        Q_.block<3,3>(0,0) *= process_noise_pos;
        Q_.block<3,3>(3,3) *= process_noise_vel;
        // 测量噪声协方差 R
        R_.setIdentity();
        R_.block<3,3>(0,0) *= measure_noise_pos;
        R_.block<3,3>(3,3) *= measure_noise_vel;
        // 初始状态协方差 P
        P_.setIdentity();
        P_.block<3,3>(0,0) *= 10.0;   // 位置初始不确定性较大
        P_.block<3,3>(3,3) *= 10.0;   // 速度初始不确定性较大
        
        is_initialized_ = false;
        last_time_sec_ = 0.0;
    }

    // 初始化滤波器（使用第一帧测量）
    void init(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double timestamp_sec) {
        x_.head<3>() = pos;
        x_.tail<3>() = vel;
        P_.setIdentity();
        P_.block<3,3>(0,0) *= 10.0;
        P_.block<3,3>(3,3) *= 10.0;
        is_initialized_ = true;
        last_time_sec_ = timestamp_sec;
    }

    void predict(double dt) {
        if (!is_initialized_ || dt <= 0.0) return;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q_;
    }

    void update(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double timestamp_sec) {
        if (!is_initialized_) {
            init(pos, vel, timestamp_sec);
            return;
        }
        
        double dt = timestamp_sec - last_time_sec_;
        if (dt < 0.0) {
            ROS_WARN("KalmanFilter: timestamp went backwards, skipping prediction");
            dt = 0.0;
        }
        
        predict(dt);
        Eigen::VectorXd z(6);
        z.head<3>() = pos;
        z.tail<3>() = vel;
        
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        Eigen::VectorXd y = z - H_ * x_;
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6,6) - K * H_) * P_;
        
        last_time_sec_ = timestamp_sec;
    }

    Eigen::Vector3d getPos() const { return x_.head<3>(); }
    Eigen::Vector3d getVel() const { return x_.tail<3>(); }
    bool isInitialized() const { return is_initialized_; }

private:
    Eigen::MatrixXd F_{6,6}, H_{6,6}, Q_{6,6}, R_{6,6}, P_{6,6};
    Eigen::VectorXd x_{6};
    bool is_initialized_;
    double last_time_sec_;   // 上一次更新的时间戳（秒）
};

class FakeReplanFSM
{
public:
  FakeReplanFSM() {}
  ~FakeReplanFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    enum FSM_EXEC_STATE
    {
      INIT,          
      WAIT_TRAJ,   
      EXEC_TRAJ,    
      RISING,
      EMERGENCY_STOP  
    };

      enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2
    };

  FakePlanManager::Ptr planner_manager_;
    ConstantVelocityKalmanFilter kf_;
    double predict_dt_;
    bool use_kalman_filter_;
 
  double replan_thresh_;          
  double planning_horizen_;      
  double emergency_stop_time_;   
  bool have_odom_, have_traj_,touch_goal_,trigger_;
  int waypoint_num_;
  int target_type_;
  double waypoints_[50][3];
  double max_vel_,max_acc_;
  FSM_EXEC_STATE exec_state_;

  int consecutive_replan_cnt_;
  bool is_rising_;
  bool rising_traj_generated_; 
  Eigen::Vector3d rise_target_;

  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;  
  Eigen::Vector3d target_pt_, target_vel_,target_acc_;                      
  std::vector<Eigen::Vector3d> waypoint_list_;          
  int current_wp_idx_;                              
  
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_;
  ros::Subscriber odom_sub_, waypoint_sub_, trigger_sub_, mandatory_stop_sub_;
  ros::Publisher poly_traj_pub_;

  void execFSMCallback(const ros::TimerEvent &e);
  void changeFSMExecState(FSM_EXEC_STATE new_state, const std::string &pos_call);
  void printFSMExecState();

  bool planToTarget(const Eigen::Vector3d &target);
  // bool planToGivenWps(const std::vector<Eigen::Vector3d>& wps);

  void checkCollisionCallback(const ros::TimerEvent &e);
  void emergencyStop();

  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  //void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void triggerCallback(const nav_msgs::PathPtr &msg);
//   void mandatoryStopCallback(const std_msgs::Empty &msg);

  void publishTraj(const traj_utils::PolyTraj &traj_msg);
};

}// namespace fake_planner

# endif