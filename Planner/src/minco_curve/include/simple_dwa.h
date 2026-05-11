#ifndef SIMPLE_DWA_H
#define SIMPLE_DWA_H


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>

#include <plan_env/grid_map_new.h>

class DWAController {
public:
    DWAController(){};
    ~DWAController() = default;

    struct Velocity {
        double v;
        double w;
    };

    struct Traj{
        std::vector<Eigen::Vector3d> traj;
        double final_yaw;
    };

    Velocity calculateBestVelocity(const std::vector<double>& pose, double v_c, double w_c,
                                    const Eigen::Vector3d& target,
                                   double dynamic_safe_radius);
    void setupDWA(ros::NodeHandle &nh, const GridMap::Ptr &map);
    typedef std::unique_ptr<DWAController> Ptr;
    double lookahead_dist_;

                        
private:
    Traj predictTrajectory(const std::vector<double>& pose, double v, double w);

    double evaluateTrajectory(const Traj& _traj,const Eigen::Vector3d& target,double v, double safe_radius,
                                std::vector<Eigen::Vector3d> predicted_obs);

    double v_max, v_min, w_max, w_min, v_acc, w_acc, v_res, w_res, dt, predict_time;
    double safe_radius, startup_safe_radius, alpha, beta, gamma;
    bool startup_flag;
    bool has_map_;
    int deadlock_count, deadlock_threshold;
    // double init_z_;
    // std::vector<Eigen::Vector3d> predicted_obs_;

    GridMap::Ptr grid_map_;
    std::vector<Eigen::Vector3d>traj_;

    ros::ServiceClient dynamic_obs_client_;
    onboard_detector::GetDynamicObstacles srv_;
    std::vector<onboardDetector::box3D> dynamic_obstacles_;
    // std::mutex dynamic_obs_mutex_;
    ros::Timer dynamic_obs_timer_;

    ros::NodeHandle node_;
    ros::Subscriber predict_obs_sub_;

};

#endif