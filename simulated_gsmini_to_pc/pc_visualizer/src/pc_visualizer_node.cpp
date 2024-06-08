#include "../include/pc_visualizer.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_visualizer");
	ros::NodeHandle nh;

	pc_visualizer pc_visualizer(nh);

	ros::Rate rate(10);
	while (ros::ok())
	{
		pc_visualizer.spinner();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
