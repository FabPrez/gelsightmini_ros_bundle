#ifndef TAXIM_POINTCLOUD_PREPARATOR_HPP
#define TAXIM_POINTCLOUD_PREPARATOR_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

#include "taxim_pointcloud_preparator/SavePointcloud.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class TaximPointCloudPreparator
{
public:
	TaximPointCloudPreparator(ros::NodeHandle &nh);
	void spinner(void);

private:
	bool enable_debug;
	ros::NodeHandle nh;
	ros::Subscriber sub_obj_pointclod;

	ros::ServiceServer service_save_pointcloud;

	PointCloud::Ptr cloud_object{new PointCloud()};
	ros::Publisher object_pub;
	std::string cloud_object_frame_id;

	ros::Publisher firstFinger_pub;
	ros::Publisher secondFinger_pub;
	sensor_msgs::PointCloud2 firstFinger_cloud_msg;
	sensor_msgs::PointCloud2 secondFinger_cloud_msg;

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);
	bool trasformAndSavePointCloud(taxim_pointcloud_preparator::SavePointcloud::Request &req,
								   taxim_pointcloud_preparator::SavePointcloud::Response &res);

	void saveAsPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	bool generateTwoFingerContact(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void saveAsPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fingerNumber);
};

#endif // TAXIM_POINTCLOUD_PREPARATOR_HPP
