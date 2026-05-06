#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

class AStarPathFollower
{
public:
    AStarPathFollower() : nh_("~")
    {
        // 参数配置
        nh_.param<double>("rate", rate_, 100.0);                 // 控制频率 (Hz)
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.3); // 到达距离阈值 (m)
        nh_.param<std::string>("world_frame", world_frame_, "world");

        // 订阅话题（可重映射）
        std::string path_topic, odom_topic;
        nh_.param<std::string>("path_topic", path_topic, "/a_star_planned_path");
        nh_.param<std::string>("odom_topic", odom_topic, "/sim/odom");

        path_sub_ = nh_.subscribe(path_topic, 1, &AStarPathFollower::pathCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic, 1, &AStarPathFollower::odomCallback, this);

        // 发布 PositionCommand
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    }

    void run()
    {
        ros::Rate loop_rate(rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            if (!path_points_.empty() && current_pose_valid_)
            {
                // 检查是否到达当前目标点
                if (current_idx_ < path_points_.size())
                {
                    Eigen::Vector3d diff = path_points_[current_idx_] - current_pose_;
                    double dist = diff.norm();
                    if (dist < arrival_threshold_)
                    {
                        // 切换到下一个点
                        current_idx_++;
                        ROS_INFO("Reached waypoint %zu, moving to next", current_idx_ - 1);
                    }
                }

                // 如果还有目标点，发布指令
                if (current_idx_ < path_points_.size())
                {
                    publishPositionCommand(path_points_[current_idx_]);
                }
                else
                {
                    // 路径已完成，可选择保持悬停或发布最后一个点
                    ROS_INFO_THROTTLE(5, "All waypoints reached, hovering");
                    if (!path_points_.empty())
                        publishPositionCommand(path_points_.back());
                }
            }
            loop_rate.sleep();
        }
    }

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path_points_.clear();
        path_points_.reserve(msg->poses.size());
        for (const auto& pose : msg->poses)
        {   
            path_points_.emplace_back(pose.pose.position.x,
                                      pose.pose.position.y,
                                      pose.pose.position.z);
        }
        current_idx_ = 0;
        ROS_INFO("Received new path with %zu points", path_points_.size());
        if (!path_points_.empty())
            ROS_INFO("First point: (%.2f, %.2f, %.2f)",
                     path_points_[0].x(), path_points_[0].y(), path_points_[0].z());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_pose_(0) = msg->pose.pose.position.x;
        current_pose_(1) = msg->pose.pose.position.y;
        current_pose_(2) = msg->pose.pose.position.z;
        current_pose_valid_ = true;
    }

    void publishPositionCommand(const Eigen::Vector3d& target)
    {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = world_frame_;
        cmd.position.x = target.x();
        cmd.position.y = target.y();
        cmd.position.z = target.z();
        // 速度和加速度设为0，控制器将仅使用位置误差（默认PD增益）
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = 0.0;
        cmd.yaw_dot = 0.0;


        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    std::vector<Eigen::Vector3d> path_points_;
    size_t current_idx_ = 0;
    Eigen::Vector3d current_pose_ = Eigen::Vector3d::Zero();
    bool current_pose_valid_ = false;

    double rate_;
    double arrival_threshold_;
    std::string world_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_path_follower");
    AStarPathFollower follower;
    follower.run();
    return 0;
}