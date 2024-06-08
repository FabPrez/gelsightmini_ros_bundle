#include "taxim_pointcloud_preparator.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "taxim_pointcloud_preparator");
    ros::NodeHandle nh;
	
	TaximPointCloudPreparator preparator(nh);

    ros::spin();
    return 0;
}
