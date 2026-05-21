#include "plan_manager.h"


bool use_dwa_;
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

        path_optimizer_rebound_.reset(new PathPlannerSim3D(nh,pp_.drone_id));
        path_optimizer_rebound_->setParam(nh);
        path_optimizer_rebound_->setEnvironment(grid_map_);

        nh.param("fsm/use_dwa",use_dwa_,false);

        if(use_dwa_){
            dwa_controller_.reset(new DWAController);
            dwa_controller_->setupDWA(nh,grid_map_);
        }
        my_traj_pub_ = nh.advertise<nav_msgs::Path>("my_traj", 10);;
        other_traj_pub_ = nh.advertise<nav_msgs::Path>("other_traj",10);
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
            traj_.setGlobalTraj(path_optimizer_rebound_->getCurrentTraj(),time_now.toSec(),pp_.drone_id);
            //global_data_.setGlobalTraj(path_optimizer_rebound_->getCurrentTraj(),time_now.toSec());
            return true;
        }
        return false;
    }

    void FakePlanManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg)
    {
        //Eigen::VectorXd durs =  global_data_.traj.getDurations();
        Eigen::VectorXd durs =  traj_.global_traj.traj.getDurations();
        // ROS_INFO("Total time is %f",durs.sum());
        //int piece_num = global_data_.traj.getPieceNum();
        int piece_num = traj_.global_traj.traj.getPieceNum();

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

            //poly_traj::CoefficientMat cMat = global_data_.traj.getPiece(i).getCoeffMat();
            poly_traj::CoefficientMat cMat = traj_.global_traj.traj.getPiece(i).getCoeffMat();
            int i6 = i * 6;
            for (int j = 0; j < 6; j++)
            {
                poly_msg.coef_x[i6 + j] = cMat(0, j);
                poly_msg.coef_y[i6 + j] = cMat(1, j);
                poly_msg.coef_z[i6 + j] = cMat(2, j);
            }
        }
    }


    void FakePlanManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
    {

        //auto data = global_data_.traj;
        auto data = traj_.global_traj.traj;
        
        Eigen::VectorXd durs = data.getDurations();
        //double total_duration = global_data_.duration;
        double total_duration = traj_.global_traj.duration;
        int piece_num = data.getPieceNum();
        poly_msg.drone_id = pp_.drone_id;
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

            poly_traj::CoefficientMat cMat = data.getPiece(i).getCoeffMat();
            int i6 = i * 6;
            for (int j = 0; j < 6; j++)
            {
                poly_msg.coef_x[i6 + j] = cMat(0, j);
                poly_msg.coef_y[i6 + j] = cMat(1, j);
                poly_msg.coef_z[i6 + j] = cMat(2, j);
            }
        }
        
        // Waypoints_msg.drone_id = pp_.drone_id;
        // Waypoints_msg.start_time = ros::Time::now();
        // Waypoints_msg.duration = 2.0f;
        // Waypoints_msg.arrival_time.resize(20);
        // Waypoints_msg.waypoint_x.resize(20);
        // Waypoints_msg.waypoint_y.resize(20);
        // Waypoints_msg.waypoint_z.resize(20);
        // int t = 0.1;
        // for(int i = 0;i < 20; i++){
        //     Waypoints_msg.arrival_time[i] = Waypoints_msg.start_time + t;
        //     Waypoints_msg.waypoint_x[i] = traj_.global_traj.traj.getPos()
        // }

        MINCO_msg.drone_id = pp_.drone_id;
        MINCO_msg.traj_id = pp_.drone_id;
        MINCO_msg.start_time = ros::Time::now();
        MINCO_msg.order = 5; // todo, only support order = 5 now.
        MINCO_msg.duration.resize(piece_num);
        Eigen::Vector3d vec;
        vec = data.getPos(0);
        MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
        vec = data.getVel(0);
        MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
        vec = data.getAcc(0);
        MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
        vec = data.getPos(total_duration);
        MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
        vec = data.getVel(total_duration);
        MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
        vec = data.getAcc(total_duration);
        MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
        MINCO_msg.inner_x.resize(piece_num - 1);
        MINCO_msg.inner_y.resize(piece_num - 1);
        MINCO_msg.inner_z.resize(piece_num - 1);
        Eigen::MatrixXd pos = data.getPositions();
        for (int i = 0; i < piece_num - 1; i++)
        {
            MINCO_msg.inner_x[i] = pos(0, i + 1);
            MINCO_msg.inner_y[i] = pos(1, i + 1);
            MINCO_msg.inner_z[i] = pos(2, i + 1);
        }
        for (int i = 0; i < piece_num; i++)
            MINCO_msg.duration[i] = durs[i];
    }


    Eigen::Vector3d FakePlanManager::getLookaheadTarget(const Eigen::Vector3d& current_pos,
                                                        double lookahead_dist) {
        if(path_optimizer_rebound_->getCurrentMinisnapPoints()){
            global_path_points_ = path_optimizer_rebound_->current_minisnap_waypoints_;
        }else{
            return Eigen::Vector3d(current_pos.x(), current_pos.y(), current_pos.z());
        }
        if (global_path_points_.size() < 2) {
            // if (global_data_.traj.getPieceNum() > 0) {
            //     double total_dur = global_data_.traj.getTotalDuration();
            //     return global_data_.traj.getPos(total_dur);
            // } 
            if (traj_.global_traj.traj.getPieceNum() > 0) {
                double total_dur = traj_.global_traj.traj.getTotalDuration();
                return traj_.global_traj.traj.getPos(total_dur);
            } 
            else {
                return Eigen::Vector3d(current_pos.x(), current_pos.y(), current_pos.z());
            }
        }

        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_path_points_.size(); ++i) {
            double dx = global_path_points_[i].x() - current_pos.x();
            double dy = global_path_points_[i].y() - current_pos.y();
            double dist = std::hypot(dx, dy);
            if(dist < 0.1){
                nearest_idx = i;
                break;
            }
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        double actual_lookahead = lookahead_dist;
        if (min_dist > 1.0) actual_lookahead += lookahead_dist * 0.5;
        double accum = 0.0;
        for (size_t i = nearest_idx; i < global_path_points_.size() - 1; ++i) {
            double dx = global_path_points_[i+1].x() - global_path_points_[i].x();
            double dy = global_path_points_[i+1].y() - global_path_points_[i].y();
            accum += std::hypot(dx, dy);
            if (accum >= actual_lookahead) {
                return global_path_points_[i+1];
            }
        }
        return global_path_points_.back();
    }

    DWAController::Velocity FakePlanManager::getDWAcmd(const std::vector<double>& pose, 
                                                        double v_c, double w_c,
                                                        double dynamic_safe_radius) {

        Eigen::Vector3d current_pos(pose[0], pose[1],pose[3]);
        double lookahead = dwa_controller_->lookahead_dist_ + v_c* 0.3;
        Eigen::Vector3d target = getLookaheadTarget(current_pos, lookahead);
        return dwa_controller_->calculateBestVelocity(pose, v_c, w_c, target, dynamic_safe_radius);
    }



    bool FakePlanManager::checkCollision(int drone_id)
    {
        if (traj_.global_traj.global_start_time < 1e9) // 尚未开始规划
            return false;
        if (traj_.swarm_traj[drone_id].drone_id != drone_id) // 轨迹无效
            return false;
        

        if(drone_id == 1){
            std::vector<Eigen::Vector3d> my_traj,other_traj;
            for (double t = 0.0; t <= traj_.global_traj.traj.getTotalDuration(); t += 0.05)
                my_traj.push_back(traj_.global_traj.traj.getPos(t));
            for (double t = 0.0; t <= traj_.swarm_traj[drone_id].traj.getTotalDuration(); t += 0.05)
                other_traj.push_back(traj_.swarm_traj[drone_id].traj.getPos(t));

            path_optimizer_rebound_->VisuaTraj(my_traj,my_traj_pub_);
            path_optimizer_rebound_->VisuaTraj(other_traj,other_traj_pub_);
        }

        double my_start = traj_.global_traj.global_start_time;
        double my_duration = traj_.global_traj.duration;          // 关键修正
        double other_start = traj_.swarm_traj[drone_id].start_time;
        double other_duration = traj_.swarm_traj[drone_id].duration;

        double t_start = max(my_start, other_start);
        double t_end = min(my_start + my_duration, other_start + other_duration);

        if (t_start >= t_end){
            ROS_ERROR("No time overlap !!");
            return false;
        }

        const double step = 0.05;
        double min_dist = std::numeric_limits<double>::infinity(); 
        for (double t = t_start; t <= t_end; t += step)
        {
            Eigen::Vector3d my_pos = traj_.global_traj.traj.getPos(t - my_start);
            Eigen::Vector3d other_pos = traj_.swarm_traj[drone_id].traj.getPos(t - other_start);
            // ROS_WARN("My Pose y is %f",my_pos[1]);
            // ROS_WARN("Other Pose y is %f",other_pos[1]);
            double dist = (my_pos - other_pos).norm();
            if (dist < min_dist)
                min_dist = dist; 
            if (dist < 1.0f) 
            {
                return true;
            }
        }
        ROS_ERROR("%d and %d ,Not close enough,closest dist is :%f",pp_.drone_id,drone_id,min_dist);
        return false;
    }

    
}
