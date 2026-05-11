/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node
*/
#include <ros/ros.h>
#include <onboard_detector/dynamicDetector.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_detector_node");
	ros::NodeHandle nh;

	onboardDetector::dynamicDetector d (nh);
	ros::ServiceServer service = nh.advertiseService("/get_dynamic_obstacles",
								&onboardDetector::dynamicDetector::getDynamicObstacles,&d);
	ROS_WARN("START SERVICE");
	ros::spin();

	return 0;
}