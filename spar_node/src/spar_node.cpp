#include <ros/ros.h>
#include <spar_node/Spar.h>

using namespace SparNode;

int main(int argc, char** argv) {
	ros::init(argc, argv, "spar_node");

	Spar spar;
	ros::spin();

	return 0;
}