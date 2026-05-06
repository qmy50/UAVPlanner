#include "plan_manager.h"


namespace fake_planner{
    void FakePlanManager::initPlanModules(ros::NodeHandle &nh)
    {

        nh.param("manager/max_vel", pp_.max_vel_, 15.0);
        nh.param("manager/max_acc", pp_.max_acc_, 5.0);
        // nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        // nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
        // nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
        nh.param("manager/drone_id", pp_.drone_id, -1);

        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        path_optimizer_rebound_.reset(new PathPlannerSim3D(nh));
        path_optimizer_rebound_->setParam(nh);
        path_optimizer_rebound_->setEnvironment(grid_map_);
    }

    bool FakePlanManager::EmergencyStop(Eigen::Vector3d stop_pos)
    {
        return false;
    }

    bool FakePlanManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {   
        // // 需要重规划
        // if(path_optimizer_rebound_->checkTrajCollision() || !path_optimizer_rebound_->have_current_traj_){
        path_optimizer_rebound_->Planning(start_pos,start_vel,start_acc,end_pos,end_vel,end_acc);
        if(getCurrentTraj()){
            return true;
        }else{
            return false;
        }
        // // 不需要重规划
        // return true;
    }


    bool FakePlanManager::getCurrentTraj(){
        if(path_optimizer_rebound_->have_current_traj_){
            // global_data_.traj = path_optimizer_rebound_->getCurrentTraj();
            auto time_now = ros::Time::now();
            global_data_.setGlobalTraj(path_optimizer_rebound_->getCurrentTraj(),time_now.toSec());
            return true;
        }
        return false;
    }

    void FakePlanManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg)
    {
        Eigen::VectorXd durs =  global_data_.traj.getDurations();
        // ROS_INFO("Total time is %f",durs.sum());
        int piece_num = global_data_.traj.getPieceNum();

        poly_msg.drone_id = 0;
        poly_msg.traj_id = 0;
        poly_msg.start_time = ros::Time::now();
        poly_msg.order = 5; // todo, only support order = 5 now.
        poly_msg.duration.resize(piece_num);
        poly_msg.coef_x.resize(6 * piece_num);
        poly_msg.coef_y.resize(6 * piece_num);
        poly_msg.coef_z.resize(6 * piece_num);
        for (int i = 0; i < piece_num; ++i)
        {
            poly_msg.duration[i] = durs(i);

            poly_traj::CoefficientMat cMat = global_data_.traj.getPiece(i).getCoeffMat();
            int i6 = i * 6;
            for (int j = 0; j < 6; j++)
            {
            poly_msg.coef_x[i6 + j] = cMat(0, j);
            poly_msg.coef_y[i6 + j] = cMat(1, j);
            poly_msg.coef_z[i6 + j] = cMat(2, j);
            }
        }
    }
}
