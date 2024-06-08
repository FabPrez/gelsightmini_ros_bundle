#ifndef pc_visualizer_H
#define pc_visualizer_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/obj_io.h>
#include <ros/package.h>
#include <pcl/common/transforms.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen_conversions/eigen_msg.h>


class pc_visualizer
{
public:
	pc_visualizer(ros::NodeHandle &nh);
	void spinner(void);
	void obj_transform(void);
	void sensor_trasform(void);

private:
	
	ros::NodeHandle nh;
	ros::Publisher object_pub;
	ros::Publisher object_pub_modified;
	ros::Publisher sensor_pub;
	sensor_msgs::PointCloud2 cloud_msg_object;
	sensor_msgs::PointCloud2 cloud_msg_object_modified;
	sensor_msgs::PointCloud2 cloud_msg_sensor;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_modified;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor;

	tf2_ros::TransformBroadcaster br_object;
	geometry_msgs::TransformStamped transformStamped_object;

	tf2_ros::TransformBroadcaster br_sensor;
	geometry_msgs::TransformStamped transformStamped_sensor;

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

	void loadObjectPly(void);
	void loadSensorPly(void);

	// Use these two functions to move the pointcloud of the sensor and to save it properly
	// ---- NB that you have to modify manually the header part of the ply stored
	void moveSensorReferenceFrame(void);
	void saveAsPly(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	
	void centralizeAndTransformPoints(void);
	void prepareForSimulation(void);
};

#endif /* pc_visualizer_H */
