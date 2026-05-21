#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <memory>

// 3D 恒定速度卡尔曼滤波器 (状态: x,y,z, vx,vy,vz)
class ConstantVelocityKalmanFilter3D
{
public:
    ConstantVelocityKalmanFilter3D(double process_noise_pos, double process_noise_vel,
                                   double measure_noise_pos)
        : process_noise_pos_(process_noise_pos), process_noise_vel_(process_noise_vel),
          measure_noise_pos_(measure_noise_pos)
    {
        F_.setIdentity();
        Q_.setZero();
        H_.setZero();
        R_.setZero();
        Q_continuous_.setZero();

        H_(0,0) = 1; H_(1,1) = 1; H_(2,2) = 1;
        R_(0,0) = measure_noise_pos;
        R_(1,1) = measure_noise_pos;
        R_(2,2) = measure_noise_pos;

        for (int i = 0; i < 3; ++i) {
            Q_continuous_(i, i) = process_noise_pos;
            Q_continuous_(3+i, 3+i) = process_noise_vel;
        }

        x_.setZero();
        P_.setIdentity() * 0.1;
        last_time_ = 0.0;
        initialized_ = false;
    }

    void predict(double dt)
    {
        if (dt <= 0.0) return;

        Eigen::Matrix<double,6,6> F;
        F.setIdentity();
        for (int i = 0; i < 3; ++i) {
            F(i, 3+i) = dt;
        }
        F_ = F;

        Eigen::Matrix<double,6,6> Q;
        Q.setZero();
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        for (int i = 0; i < 3; ++i) {
            Q(i, i)     = Q_continuous_(i,i)*dt + Q_continuous_(3+i,3+i)*dt3/3.0;
            Q(i, 3+i)   = Q_continuous_(3+i,3+i)*dt2/2.0;
            Q(3+i, i)   = Q_continuous_(3+i,3+i)*dt2/2.0;
            Q(3+i, 3+i) = Q_continuous_(3+i,3+i)*dt;
        }
        Q_ = Q;

        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const Eigen::Vector3d& z)
    {
        if (!initialized_)
        {
            x_.head<3>() = z;
            x_.tail<3>().setZero();
            initialized_ = true;
            return;
        }

        Eigen::Matrix<double,3,3> S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<double,6,3> K = P_ * H_.transpose() * S.inverse();

        Eigen::Vector3d y = z - H_ * x_;
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix<double,6,6>::Identity() - K * H_) * P_;
    }

    Eigen::Vector3d getPosition() const { return x_.head<3>(); }
    Eigen::Vector3d getVelocity() const { return x_.tail<3>(); }
    void setTime(double t) { last_time_ = t; }
    double getLastTime() const { return last_time_; }

    double getProcessNoisePos() const { return process_noise_pos_; }
    double getProcessNoiseVel() const { return process_noise_vel_; }
    double getMeasureNoisePos() const { return measure_noise_pos_; }
    bool isInitialized() const { return initialized_; }

private:
    Eigen::Matrix<double,6,6> F_;
    Eigen::Matrix<double,6,6> Q_;
    Eigen::Matrix<double,3,6> H_;
    Eigen::Matrix<double,3,3> R_;
    Eigen::Matrix<double,6,6> Q_continuous_;
    Eigen::Matrix<double,6,1> x_;
    Eigen::Matrix<double,6,6> P_;

    double last_time_;
    bool initialized_;
    double process_noise_pos_, process_noise_vel_, measure_noise_pos_;
};

// 1D 恒定速度卡尔曼滤波器
class ConstantVelocityKalmanFilter1D
{
public:
    ConstantVelocityKalmanFilter1D(double process_noise_pos, double process_noise_vel,
                                   double measure_noise_pos)
        : process_noise_pos_(process_noise_pos), process_noise_vel_(process_noise_vel),
          measure_noise_pos_(measure_noise_pos)
    {
        Q_continuous_ << process_noise_pos, 0.0,
                         0.0, process_noise_vel;
        R_ = measure_noise_pos;
        x_.setZero();
        P_ = Eigen::Matrix2d::Identity() * 0.1;
        initialized_ = false;
        last_time_ = 0.0;
    }

    void predict(double dt)
    {
        if (dt <= 0.0) return;
        Eigen::Matrix2d F;
        F << 1.0, dt, 0.0, 1.0;
        double dt2 = dt*dt;
        double dt3 = dt2*dt;
        Eigen::Matrix2d Q;
        Q << Q_continuous_(0,0)*dt + Q_continuous_(1,1)*dt3/3.0, Q_continuous_(1,1)*dt2/2.0,
             Q_continuous_(1,1)*dt2/2.0, Q_continuous_(1,1)*dt;
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    void update(double z)
    {
        if (!initialized_)
        {
            x_(0) = z;
            x_(1) = 0.0;
            initialized_ = true;
            return;
        }
        Eigen::Vector2d H; H << 1.0, 0.0;
        double S = H.transpose() * P_ * H + R_;
        Eigen::Vector2d K = P_ * H / S;
        double y = z - H.dot(x_);
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix2d::Identity() - K * H.transpose()) * P_;
    }

    double getAngle() const { return x_(0); }
    double getAngularVelocity() const { return x_(1); }
    void setTime(double t) { last_time_ = t; }
    double getLastTime() const { return last_time_; }
    bool isInitialized() const { return initialized_; }

private:
    Eigen::Matrix2d Q_continuous_;
    double R_;
    Eigen::Vector2d x_;
    Eigen::Matrix2d P_;
    bool initialized_;
    double last_time_;
    double process_noise_pos_, process_noise_vel_, measure_noise_pos_;
};

class PoseToOdomNode
{
public:
    PoseToOdomNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("pose_topic", pose_topic_, "/pose");
        pnh.param<std::string>("odom_topic", odom_topic_, "/fake_odom");
        pnh.param<std::string>("frame_id", frame_id_, "odom");
        pnh.param<std::string>("child_frame_id", child_frame_id_, "base_link");
        pnh.param<double>("publish_rate", publish_rate_, 0.0);
        pnh.param<bool>("use_kalman", use_kalman_, true);
        pnh.param<double>("odom_jump_thresh", odom_jump_thresh_, 0.5);
        pnh.param<double>("odom_jump_cooldown", odom_jump_cooldown_, 0.5);

        pnh.param<double>("process_noise_pos", process_noise_pos_, 0.05);
        pnh.param<double>("process_noise_vel", process_noise_vel_, 1.0);
        pnh.param<double>("measure_noise_pos", measure_noise_pos_, 0.05);
        pnh.param<double>("yaw_process_noise_pos", yaw_process_noise_pos_, 0.05);
        pnh.param<double>("yaw_process_noise_vel", yaw_process_noise_vel_, 0.5);
        pnh.param<double>("yaw_measure_noise", yaw_measure_noise_, 0.03);

        pose_sub_ = nh.subscribe(pose_topic_, 10, &PoseToOdomNode::poseCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);

        kf3d_ = std::make_shared<ConstantVelocityKalmanFilter3D>(
            process_noise_pos_, process_noise_vel_, measure_noise_pos_);
        kf_yaw_ = std::make_shared<ConstantVelocityKalmanFilter1D>(
            yaw_process_noise_pos_, yaw_process_noise_vel_, yaw_measure_noise_);

        if (publish_rate_ > 0) {
            timer_ = pnh.createTimer(ros::Duration(1.0/publish_rate_), &PoseToOdomNode::timerCallback, this);
        }

        ROS_INFO("PoseToOdomNode ready");
    }

private:
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        ros::Time now = msg->header.stamp;
        if (now.isZero()) now = ros::Time::now();

        Eigen::Vector3d new_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                         msg->pose.orientation.z, msg->pose.orientation.w);
        double new_yaw = tf::getYaw(q);

        if (has_last_pose_ && use_kalman_ && kf3d_->isInitialized()) {
            double dt = (now - last_time_).toSec();
            if (dt > 0 && dt < 1.0) {
                double jump_dist = (new_pos - last_pos_).norm();
                double max_expected = kf3d_->getVelocity().norm() * dt + 0.1;
                if (jump_dist > odom_jump_thresh_ && jump_dist > max_expected) {
                    ROS_WARN("ODOM JUMP reset filter");
                    kf3d_ = std::make_shared<ConstantVelocityKalmanFilter3D>(
                        process_noise_pos_, process_noise_vel_, measure_noise_pos_);
                    kf_yaw_ = std::make_shared<ConstantVelocityKalmanFilter1D>(
                        yaw_process_noise_pos_, yaw_process_noise_vel_, yaw_measure_noise_);
                    kf3d_->update(new_pos);
                    kf_yaw_->update(new_yaw);
                    has_last_pose_ = false;
                    last_time_ = now;
                    last_pos_ = new_pos;
                    last_yaw_ = new_yaw;
                    return;
                }
            }
        }

        double current_time = now.toSec();
        if (has_last_pose_ && use_kalman_) {
            double dt = (now - last_time_).toSec();
            if (dt > 0 && dt < 0.5) {
                kf3d_->predict(dt);
                kf_yaw_->predict(dt);
            }
        }

        if (use_kalman_) {
            kf3d_->update(new_pos);
            kf_yaw_->update(new_yaw);
        }

        Eigen::Vector3d odom_pos, odom_vel;
        double odom_yaw, odom_ang_z;
        if (use_kalman_) {
            odom_pos = kf3d_->getPosition();
            odom_vel = kf3d_->getVelocity();
            odom_yaw = kf_yaw_->getAngle();
            odom_ang_z = kf_yaw_->getAngularVelocity();
        } else {
            odom_pos = new_pos;
            odom_vel = computeVelocity(new_pos, now);
            odom_yaw = new_yaw;
            odom_ang_z = computeAngularVelocity(new_yaw, now);
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_id_;

        odom.pose.pose.position.x = odom_pos.x();
        odom.pose.pose.position.y = odom_pos.y();
        odom.pose.pose.position.z = odom_pos.z();
        odom.pose.pose.orientation = msg->pose.orientation;

        odom.twist.twist.linear.x = odom_vel.x();
        odom.twist.twist.linear.y = odom_vel.y();
        odom.twist.twist.linear.z = odom_vel.z();
        odom.twist.twist.angular.z = odom_ang_z;

        odom.pose.covariance[0] = 0.01;
        odom.pose.covariance[7] = 0.01;
        odom.pose.covariance[14] = 0.01;
        odom.twist.covariance[0] = 0.1;
        odom.twist.covariance[7] = 0.1;
        odom.twist.covariance[14] = 0.1;

        if (publish_rate_ <= 0)
            odom_pub_.publish(odom);
        else
            last_odom_ = odom;

        last_pos_ = new_pos;
        last_yaw_ = new_yaw;
        last_time_ = now;
        has_last_pose_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (has_last_pose_) odom_pub_.publish(last_odom_);
    }

    Eigen::Vector3d computeVelocity(const Eigen::Vector3d& new_pos, const ros::Time& now) {
        if (!has_last_pose_) return Eigen::Vector3d::Zero();
        double dt = (now - last_time_).toSec();
        if (dt < 1e-6)
            return Eigen::Vector3d::Zero();
        return (new_pos - last_pos_) / dt;

    }

    double computeAngularVelocity(double new_yaw, const ros::Time& now) {
        if (!has_last_pose_) return 0.0;
        double dt = (now - last_time_).toSec();
        if (dt < 1e-6) return 0.0;
        double diff = new_yaw - last_yaw_;
        if (diff > M_PI) diff -= 2*M_PI;
        if (diff < -M_PI) diff += 2*M_PI;
        return diff / dt;
    }

    ros::Subscriber pose_sub_;
    ros::Publisher odom_pub_;
    ros::Timer timer_;
    std::string pose_topic_, odom_topic_, frame_id_, child_frame_id_;
    double publish_rate_;
    bool use_kalman_;
    double odom_jump_thresh_, odom_jump_cooldown_;

    double process_noise_pos_, process_noise_vel_, measure_noise_pos_;
    double yaw_process_noise_pos_, yaw_process_noise_vel_, yaw_measure_noise_;

    std::shared_ptr<ConstantVelocityKalmanFilter3D> kf3d_;
    std::shared_ptr<ConstantVelocityKalmanFilter1D> kf_yaw_;

    bool has_last_pose_ = false;
    Eigen::Vector3d last_pos_;
    double last_yaw_ = 0.0;
    ros::Time last_time_;
    nav_msgs::Odometry last_odom_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_to_odom_node");
    ros::NodeHandle nh, pnh("~");
    PoseToOdomNode node(nh, pnh);
    ros::spin();
    return 0;
}
