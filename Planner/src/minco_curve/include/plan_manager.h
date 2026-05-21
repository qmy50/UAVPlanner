#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include "traj_optmizer_3D.h"
#include "plan_container.hpp"
#include <ros/ros.h>
#include <memory>
#include <plan_env/grid_map_new.h>
#include "simple_dwa.h"
#include <traj_utils/Waypoints.h>


namespace fake_planner{
    class FakePlanManager{
    
        public:
            FakePlanManager(){};
            ~FakePlanManager(){};

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


            bool EmergencyStop(Eigen::Vector3d stop_pos);

            bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                const Eigen::Vector3d &end_pos,const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
            void initPlanModules(ros::NodeHandle &nh);

            void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg);

            void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

            // bool planGlobalTrajWaypoints(
            //     const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
            //     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

            bool getCurrentTraj();

            //void updateGlobalPathPoints();

            Eigen::Vector3d getLookaheadTarget(const Eigen::Vector3d& current_pos,
                                                            double lookahead_dist);

            DWAController::Velocity getDWAcmd(const std::vector<double>& pose, 
                                                double v_c, double w_c,
                                                double dynamic_safe_radius);
            
            bool checkCollision(int drone_id);

            inline bool needRePlan(){
                if(path_optimizer_rebound_->checkTrajCollision() || !path_optimizer_rebound_->have_current_traj_){
                    return true;
                }
                return false;
            }

            PlanParameters pp_;
            TrajContainer traj_;
            GlobalTrajData global_data_;
            GridMap::Ptr grid_map_;
            ros::Publisher my_traj_pub_,other_traj_pub_;


        private:
            PathPlannerSim3D::Ptr path_optimizer_rebound_;
            DWAController::Ptr dwa_controller_;


            std::vector<Eigen::Vector3d> global_path_points_;

        public:
            typedef std::unique_ptr<FakePlanManager> Ptr;
    };

}


#endif