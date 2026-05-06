#include "stc_gen_3D.h"

namespace stc_gen_3D{

    int STCGen::poly_count_ = 0;

    STCGen::STCGen(ros::NodeHandle& nh, std::string path_topic)
        : nh_(nh), path_topic_(path_topic) {
        _path_sub = nh_.subscribe(path_topic_, 1, &STCGen::pathCallBack, this);
        ROS_INFO("Init the 3D STC Generator, subscribe path topic %s", path_topic.c_str());
    }

    void STCGen::pathCallBack(const nav_msgs::Path::ConstPtr& wp) {
        path_.clear();
        double cum_dist = 0.0;
        Eigen::Vector3d pt(wp->poses[0].pose.position.x,
                           wp->poses[0].pose.position.y,
                           wp->poses[0].pose.position.z);
        path_.push_back(pt);
        start_point_ = pt;
        ROS_INFO("Start point set: (%.2f, %.2f, %.2f)", start_point_.x(), start_point_.y(), start_point_.z());
        for (int k = 1; k < (int)wp->poses.size(); k++) {
            Eigen::Vector3d pt(wp->poses[k].pose.position.x,
                               wp->poses[k].pose.position.y,
                               wp->poses[k].pose.position.z);
            path_.push_back(pt);
            cum_dist += (path_[k] - path_[k-1]).norm();
            if (wp->poses[k].pose.position.z < 0.0)
                break;
        }
        cost = cum_dist;
        ROS_INFO("got a* path");
    }
} // namespace stc_gen_3D