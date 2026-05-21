#include <ros/ros.h>
#include "traj_optmizer_3D.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "fake_planner_node");
    ros::NodeHandle nh("~");
    PathPlannerSim3D pathPlanner(nh,1);
    //PathPlannerSim3D sim(nh);
    
    ros::spin();
    return 0;
}