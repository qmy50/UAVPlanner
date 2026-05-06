#include <ros/ros.h>
#include "fake_planner_fsm.h"

using namespace fake_planner;

int main(int argc, char **argv){
    ros::init(argc, argv, "fake_planner_node");
    ros::NodeHandle nh("~");
    FakeReplanFSM my_planner;
    my_planner.init(nh);
    //PathPlannerSim3D sim(nh);
    
    ros::spin();
    return 0;
}