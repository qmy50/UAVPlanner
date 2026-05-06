#ifndef STC_GEN_H
#define STC_GEN_H

#include <memory>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <nav_msgs/Path.h>

namespace stc_gen_3D{
    class STCGen{
        private:
            static int poly_count_;
            ros::NodeHandle nh_;
            std::string path_topic_;
            bool have_obs = false;
            bool set_start = false;
            std::vector<Eigen::Vector3d> path_;
            Eigen::Vector3d start_point_;
            double cost = 0;

            ros::Subscriber _path_sub;
            void pathCallBack(const nav_msgs::Path::ConstPtr& wp);

        public:
            STCGen(ros::NodeHandle& nh, std::string path_topic);
           static inline void ConvexHull(const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &line_segment,
                                   const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &point_cloud,
                                   Eigen::MatrixX4d &hpoly,
                                   const double max_aaxis,
                                   const double max_baxis,
                                   const double max_caxis) {
                auto line = std::make_shared<LineSegment<3>>(line_segment[0], line_segment[1]);
                line->set_local_bbox(Eigen::Vector3d(max_aaxis, max_baxis, max_caxis));
                line->set_obs(point_cloud);
                line->dilate(0);
                auto lc3d = LinearConstraint3D((line_segment[0] + line_segment[1]) / 2, line->get_polyhedron().hyperplanes());
                hpoly.resize(lc3d.A_.rows(), 4);
                hpoly << lc3d.A_, lc3d.b_;
            }

           static inline void ConvexHull(const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &line_segment,
                                        const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &point_cloud,
                                        Eigen::MatrixX4d &hpoly,
                                        Polyhedron<3> &poly_vis,
                                        const double max_aaxis,
                                        const double max_baxis,
                                        const double max_caxis) {
                auto line = std::make_shared<LineSegment<3>>(line_segment[0], line_segment[1]);
                line->set_local_bbox(Eigen::Vector3d(max_aaxis, max_baxis, max_caxis));
                line->set_obs(point_cloud);
                line->dilate(0);
                poly_vis = line->get_polyhedron();
                auto lc3d = LinearConstraint3D((line_segment[0] + line_segment[1]) / 2, line->get_polyhedron().hyperplanes());
                hpoly.resize(lc3d.A_.rows(), 4);
                hpoly << lc3d.A_, lc3d.b_;
            }

            inline void setStart(const Eigen::Vector3d& start_point) {
                path_.clear();
                start_point_ = start_point;
                path_.push_back(start_point_);
                set_start = true;
                ROS_INFO("Start point set: (%.2f, %.2f, %.2f)", start_point.x(), start_point.y(), start_point.z());
            }

            inline double getPlanPath(std::vector<Eigen::Vector3d> &path) {
                if (path_.empty()) {
                    return 0.0;
                }
                path = path_;
                return cost;
            }
    };
} // namespace stc_gen

#endif