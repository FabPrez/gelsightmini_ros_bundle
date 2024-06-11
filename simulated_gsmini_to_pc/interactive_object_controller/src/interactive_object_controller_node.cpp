#include "../include/interactive_object_controller.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "interactive_object_controller_node");
	ros::NodeHandle nh;

	InteractiveObjectController objectController(nh);

	ros::Rate rate(10);

	while (ros::ok())
	{
		objectController.spinner();
		rate.sleep();
		ros::spinOnce();
	}
}
