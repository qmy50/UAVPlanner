#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <utils/poly_traj_utils.hpp>

using std::vector;


namespace fake_planner{
  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct GlobalTrajData
  {
    
    poly_traj::Trajectory traj;
    double global_start_time; // world time
    double duration;
    int drone_id;
  };

  struct swarmTrajData
  {
    
    poly_traj::Trajectory traj;
    double start_time; // world time
    double duration;
    int drone_id;
    double des_clearance;
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double polyTraj_piece_length;  // distance between adjacient B-spline control points
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_distinctive_trajs;
    bool touch_goal;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  typedef std::vector<swarmTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    SwarmTrajData swarm_traj;

    TrajContainer() {}
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time,const int& drone_id)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.drone_id = drone_id;

    }
  };

}// namespace fake_planner

#endif