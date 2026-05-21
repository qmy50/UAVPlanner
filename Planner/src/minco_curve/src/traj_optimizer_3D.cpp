#include "traj_optmizer_3D.h"


PathPlannerSim3D::PathPlannerSim3D(ros::NodeHandle &nh,const int& id)
        : public_nh(nh), has_map_(false), planned_(false), drone_id_(id) {
        std::string cloud_topic;
        public_nh.param<std::string>("cloud_topic", cloud_topic, "/random_complex/global_map");
        ROS_WARN("The cloud topic is %s",cloud_topic.c_str());
        ROS_WARN("The cloud topic is %s",cloud_topic.c_str());
        ROS_WARN("The cloud topic is %s",cloud_topic.c_str());
        public_nh.param("use_real_model",use_real_model_,false);
        map_sub_ = public_nh.subscribe(cloud_topic, 1, &PathPlannerSim3D::MapCallback, this);

        path_pub_ = public_nh.advertise<nav_msgs::Path>("search_path", 1);
        traj_pub_ = public_nh.advertise<nav_msgs::Path>("opt_traj", 1);
        grid_map_vis_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
        stc_pub_ = public_nh.advertise<decomp_ros_msgs::PolyhedronArray>("stc", 1);
        waypoint_pub_ = public_nh.advertise<visualization_msgs::Marker>("minco_waypoints", 10);
        minisnap_pub_ = public_nh.advertise<visualization_msgs::Marker>("minisnap_waypoints", 10);
        wp_traj_vis_pub_ = public_nh.advertise<visualization_msgs::Marker>("minijerk_waypoints", 10);
        // poly_traj_pub_ = public_nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);


        public_nh.param<std::string>("astar_topic", astar_topic_, "/a_star_planned_path");
        stc_gen_3d_ = std::make_shared<stc_gen_3D::STCGen>(public_nh, astar_topic_);
        ROS_INFO("set a* topic");

        // check_timer_ = public_nh.createTimer(ros::Duration(0.2), &PathPlannerSim3D::checkAndPlan, this);

        ROS_INFO("3D Path Planner initialized. Waiting for point cloud and A* path...");
        have_a_star_path_ = false;
        have_minco_waypoints_ = false;
        have_current_traj_ = false;
        have_minisnap_waypoints_ = false;
        
    }

    void PathPlannerSim3D::setParam(ros::NodeHandle &nh){
        nh.param("manager/max_vel", max_vel_, 2.0);
        nh.param("manager/max_acc", max_acc_, 1.5);
    }

    void PathPlannerSim3D::setEnvironment(const GridMap::Ptr &map){
        grid_map_ = map;

        a_star_.reset(new AStar(drone_id_));
        a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
        has_map_ = true;

    }

    bool PathPlannerSim3D::checkTrajCollision(){
        if(! have_minco_waypoints_ ){
            return true;
        }
        double offset = 0.15; 
        Eigen::Vector3d dir(1.0, 1.0, 0.0); 
        for(const auto& waypoint_position : current_minco_waypoints_){
            if(a_star_->checkOccupancy(waypoint_position,drone_id_)){
                have_a_star_path_ = false;
                have_current_traj_ = false;
                have_minco_waypoints_ = false;
                return true;
            }
        }
        return false;
    }

    void PathPlannerSim3D::Planning(const Eigen::Vector3d& start_position,const Eigen::Vector3d& start_vel,const Eigen::Vector3d& start_acc,
                                    const Eigen::Vector3d& end_position, const Eigen::Vector3d& end_vel,const Eigen::Vector3d& end_acc) {

        ros::Time start = ros::Time::now();
        current_minco_waypoints_.clear();
        current_a_star_waypoints_.clear();
        current_minisnap_waypoints_.clear();
        current_traj_.clear();
        have_a_star_path_ = false;
        have_minco_waypoints_=false;
        have_current_traj_=false;
        have_minisnap_waypoints_ = false;
        

        // std::vector<Eigen::Vector3d> path;
        double step_size;
        if(use_real_model_){
            step_size = grid_map_->getResolution() + 0.2;
            //ROS_WARN("USED");
        }else{
            step_size = grid_map_->getResolution() + 0.5;
            //ROS_WARN("DID NOT USED");
        }
        // ROS_INFO("resolution is %f",step_size);
        // ROS_INFO("the start pos is x=%f,y=%f,z=%f",start_position(0),start_position(1),start_position(2));
        // ROS_INFO("the end pos is x=%f,y=%f,z=%f",end_position(0),end_position(1),end_position(2));
        int flag = a_star_->AstarSearch(step_size, start_position, end_position);

        if(flag == ASTAR_RET::SUCCESS){
           current_a_star_waypoints_ = a_star_->getPath();
           have_a_star_path_ = true;
        }else{
            ROS_WARN("A star Failed, quit");
            return;
        }

        // double cost = stc_gen_3d_->getPlanPath(path);
        if (current_a_star_waypoints_.size() < 2) {
            ROS_WARN("Invalid A* path (size < 2), skipping optimization.");
            return;
        }

        // 可视化原始 A* 路径
        VisuaTraj(current_a_star_waypoints_,path_pub_);

        VecE<Polyhedron<3>> ploys_vis;
        STCGen3D(current_a_star_waypoints_, hpolys, ploys_vis);
        Visua(ploys_vis);

        /**
         * @note minisnap traj generation 
         */

        std::vector<Eigen::Vector3d> centers(current_a_star_waypoints_.size());
        centers[0] = current_a_star_waypoints_[0];
        for (size_t i = 0; i < ploys_vis.size(); ++i) {
            // ROS_INFO("Get center,time is %ld",i);
            Eigen::Vector3d centroid = cal_center(ploys_vis[i]);
            // ROS_INFO("The certer is x:%f,y:%f,z:%f",centroid.x(),centroid.y(),centroid.z());
            centers[i+1] = centroid;  // 第 i 段终点对应 path[i+1]
        }
        int N = current_a_star_waypoints_.size();
        double lambda_center = 15.0;
        centers.back() = current_a_star_waypoints_.back();
        //std::vector<Eigen::Vector3d> minisnap_waypoints;
        // ROS_INFO("num of hpoly is %ld",hpolys.size());
        // ROS_INFO("num of path is %d",N);
        // ROS_INFO("num of poly is %ld",ploys_vis.size());
        current_minisnap_waypoints_ = minisnap::minisnap_solver(
                    centers,
                    hpolys,   // 每个多面体的半平面矩阵 (a,b,c,d), a*x+b*y+c*z+d <= 0
                    current_a_star_waypoints_,
                    N,
                    lambda_center);
        // ROS_INFO("we are going to visulize the minisnap traj");
        // ROS_INFO("num of minisnap_waypoints is %ld",minisnap_waypoints.size());
        // VisuaWaypoints(minisnap_waypoints, minisnap_pub_);  

        have_minisnap_waypoints_ = true;
        std::vector<std::array<Eigen::Vector3d,4>> smooth_control_points;
        smooth_control_points = minisnap::buildSafeBezierPath(current_minisnap_waypoints_,hpolys);

        // std::vector<Eigen::Vector3d> minco_waypoints;
        Eigen::Vector3d last_waypoints = Eigen::Vector3d::Zero();
        int counter = 0;
        for(size_t i = 0;i < smooth_control_points.size();i ++){
            for(const auto& points:smooth_control_points[i]){
                if( (points - last_waypoints).norm() < 1e-6)continue;
                current_minco_waypoints_.push_back(points);
                last_waypoints = points;
                // ROS_INFO("current control points is:x=%f, y=%f, z=%f",points.x(),points.y(),points.z());
                counter ++;
            }
        }
        have_minco_waypoints_ = true;

        VisuaWaypoints(current_minco_waypoints_, minisnap_pub_);  

        // poly_traj::Trajectory minisnap_traj_;
        // std::vector<Eigen::Vector3d> traj_pts;
        // minisnap_traj_ = minisnap::convertMinijerkToTraj3(bezier_control_points,polytime_init_);
        // // ROS_INFO("The total time of traj is %f",minisnap_traj_.getTotalDuration());
        // for (double t = 0.0; t <= minisnap_traj_.getTotalDuration(); t += 0.2)
        //     traj_pts.push_back(minisnap_traj_.getPos(t));
        // ROS_INFO("We got the minisnap traj");
        // VisuaTraj(traj_pts,traj_pub_);
        
        /**
         * @note minco traj generation
        */

        pieceNum_ = current_minisnap_waypoints_.size() - 1;
        Eigen::Vector3d start_dir = (current_a_star_waypoints_[1] - current_a_star_waypoints_[0]).normalized();
        Eigen::Vector3d end_dir   = (current_a_star_waypoints_.back() - current_a_star_waypoints_[current_a_star_waypoints_.size()-2]).normalized();

        double start_speed = start_vel.norm();
        // ROS_WARN_THROTTLE(0.5,"Start speed is : %f"start_speed);
        double end_speed = end_vel.norm();
        if(start_speed > max_vel_)start_speed = max_vel_;
        // if(end_speed > max_vel_)end_speed = max_vel_;
        Eigen::Vector3d start_vel_calc = start_dir * start_speed;
        Eigen::Vector3d end_vel_calc   = end_dir   * end_speed;

        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState.row(0) = current_a_star_waypoints_.front().transpose();
        headState.row(1) = start_vel_calc.transpose();
        // headState.row(2) = Eigen::Vector3d::Zero().transpose();
        headState.row(2) = start_acc.transpose();
        tailState.row(0) = current_a_star_waypoints_.back().transpose();
        tailState.row(1) = end_vel_calc.transpose();
        // tailState.row(2) = Eigen::Vector3d::Zero().transpose();
        tailState.row(2) = end_acc.transpose();

        opt_.setConditions(headState, tailState, pieceNum_);
        Eigen::Matrix<double, 3, -1> inPos(3, pieceNum_ - 1);
        for (int i = 1; i < pieceNum_; ++i)
            inPos.col(i - 1) = current_minisnap_waypoints_[i];
        ts_ = Eigen::VectorXd::Constant(pieceNum_, 1);
        //ROS_WARN("ts_ segment is %ld",ts_.rows());
        //ROS_WARN("init time segment is %ld",polytime_init_.rows());

        Eigen::VectorXd polytime_init_ = minisnap::timeAllocation(current_minisnap_waypoints_,max_vel_,max_acc_,start_speed,end_speed); 
        // double total_dist=0.0f; 
        // for(int i=1;i<minisnap_waypoints.size();i++){
        //     total_dist += (minisnap_waypoints[i]-minisnap_waypoints[i-1]).norm();
        // }
        // ROS_WARN("The init total distance is %f",total_dist);
        //ROS_WARN("The init total time is %f",polytime_init_.sum());
        opt_.setParameters(inPos.transpose(), polytime_init_);

        // Trajectory<3, 5> traj;
        poly_traj::Trajectory traj;
        opt_.getTrajectory(traj);
        current_traj_ = traj;
        have_current_traj_ = true;

        std::vector<Eigen::Vector3d> traj_pts;
        // for (double t = 0.0; t <= traj.getPieceNum(); t += 0.05)
        //     traj_pts.push_back(traj.getPos(t));
        for (double t = 0.0; t <= traj.getTotalDuration(); t += 0.05)
            traj_pts.push_back(traj.getPos(t));
        VisuaTraj(traj_pts,traj_pub_);
        ros::Time end = ros::Time::now();
        double elapsed = (end - start).toSec();
        //ROS_WARN("Plan succeed ! Planning time elapsed: %.6f s", elapsed);
    }
    
    // decompose velocity into x/y components
    // theta: velocity direction（maybe yaw）
void PathPlannerSim3D::DecompVel(const double theta, const double vel, double &vx, double &vy,double &vz){
    const double epsi = 1e-8;
    double vt = std::abs(vel) < epsi?epsi:std::abs(vel);
    vx = vt*std::cos(theta);
    vy = vt*std::sin(theta);
    vz = 0.0;
}


template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs, int delta = 1){
        decomp_ros_msgs::PolyhedronArray msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        int i = 1;
        for (const auto &v : vs){
            if(i == delta){
                msg.polyhedrons.push_back(DecompROS::polyhedron_to_ros(v)); 
                i = 0;   
            }
            i++;
        }
        return msg;
    }

    // Visualize safety corridor
void PathPlannerSim3D::Visua(const VecE<Polyhedron<3>> &ploys_vis){
        decomp_ros_msgs::PolyhedronArray poly_msg = polyhedron_array_to_ros(ploys_vis, 1);
        stc_pub_.publish(poly_msg);
    }

double pointToSegmentSqrDist(const Eigen::Vector3d& pt, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    Eigen::Vector3d ab = p2 - p1;
    Eigen::Vector3d ac = pt - p1;
    double t = ac.dot(ab) / ab.squaredNorm();
    if (t <= 0.0) return ac.squaredNorm();
    if (t >= 1.0) return (pt - p2).squaredNorm();
    return (ac - t * ab).squaredNorm();
}

void PathPlannerSim3D::STCGen3D(const std::vector<Eigen::Vector3d> &path, VecE<Eigen::MatrixX4d> &hpoly,VecE<Polyhedron<3>> &ploys_vis) {
        hpoly.resize(path.size() - 1);
        ploys_vis.resize(path.size() - 1);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            const auto& p1 = path[i];
            const auto& p2 = path[i+1];
            double len = (p2 - p1).norm();
            double max_aaxis = len * 0.5 + 0.2;  // 沿线段方向半长
            double max_baxis = 1.0;              // 水平垂直方向半长
            double max_caxis = 0.5;              // 垂直方向半长
            Eigen::Vector3d center = (p1 + p2) / 2.0;
            Eigen::Vector3d min_bb = center - Eigen::Vector3d(max_aaxis, max_baxis, max_caxis);
            Eigen::Vector3d max_bb = center + Eigen::Vector3d(max_aaxis, max_baxis, max_caxis);
            // 筛选ROI内的障碍物点
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>  roi_points;
            for (const auto& pt : local_obstacles_) {
                double max_dist_sq = max_baxis * max_baxis;  
                if (pointToSegmentSqrDist(pt, p1, p2) <= max_dist_sq) {
                    roi_points.push_back(pt);
                }
            }
            ploys_vis[i] = Polyhedron3D();
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> seg = {p1, p2};
            stc_gen_3D::STCGen::ConvexHull(seg, roi_points, hpoly[i], ploys_vis[i],
                                        max_aaxis, max_baxis, max_caxis);
        }
}

    // VisuaTraj
void PathPlannerSim3D::VisuaTraj(const std::vector<Eigen::Vector3d> &path,ros::Publisher path_pub){
        nav_msgs::Path vis_path;
        vis_path.header.frame_id = "map";
        vis_path.header.stamp = ros::Time::now();
        vis_path.poses.resize(path.size());
        for(int i = 0;i<path.size();i++){
            vis_path.poses[i].header.frame_id = "map";
            vis_path.poses[i].header.stamp = ros::Time::now();
            vis_path.poses[i].pose.position.x = path[i].x();
            vis_path.poses[i].pose.position.y = path[i].y();
            vis_path.poses[i].pose.position.z = path[i].z();
        }
        path_pub.publish(vis_path);
}

void PathPlannerSim3D::VisuaWaypoints(const std::vector<Eigen::Vector3d> &traj, ros::Publisher marker_pub){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "minisnap_waypoints";
    marker.id = 0;              

    // 1. 先删除相同 id 的旧 Marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // 2. 重新配置为添加模式
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    
    marker.points.clear();
    for (const auto &pt : traj) {
        geometry_msgs::Point p;
        p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
        marker.points.push_back(p);
    }
    marker_pub.publish(marker);
}

void PathPlannerSim3D::MapCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_map){
    if(!has_map_ || have_current_traj_)return;
    ROS_INFO_THROTTLE(1.0,"Have Local Map");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    local_obstacles_.clear();
    // pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    // sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(*pointcloud_map,cloud);
    pcl::PointXYZ pt;
    for (const auto& pt : cloud.points) {
        Eigen::Vector3d temp(pt.x, pt.y, pt.z);
        local_obstacles_.push_back(temp);
        // cloud_vis.points.push_back(pt);
    }
    // cloud_vis.width    = cloud_vis.points.size();
    // cloud_vis.height   = 1;
    // cloud_vis.is_dense = true;
    // pcl::toROSMsg(cloud_vis, map_vis);
    // map_vis.header.frame_id = "map";
    // grid_map_vis_pub_.publish(map_vis);
    // has_map_ = true;
    // ROS_INFO("SimpleMoveBase::MapCallback");
}

void PathPlannerSim3D::visWayPointTraj( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time){
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";
    float _vis_traj_width = 0.15;
    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;

    _traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    Eigen::Vector3d cur = Eigen::Vector3d::Zero();

    for(int i = 0; i < time.size(); i++ )
    {
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
            pos = minisnap::getPosPoly(polyCoeff, i, t);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
            _traj_vis.points.push_back(pt);

        }
    }
    wp_traj_vis_pub_.publish(_traj_vis);
}

void PathPlannerSim3D::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg)
  {
    Eigen::VectorXd durs = minco_traj_ -> getDurations();
    ROS_WARN("Total time is %f",durs.sum());
    int piece_num = minco_traj_ -> getPieceNum();

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

      poly_traj::CoefficientMat cMat = minco_traj_ -> getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
  }