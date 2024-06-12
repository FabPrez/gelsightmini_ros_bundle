#ifndef INTERACTIVE_OBJECT_CONTROLLER_HPP
#define INTERACTIVE_OBJECT_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <interactive_object_controller/object_controllerConfig.h>

#include <Eigen/Dense>

class InteractiveObjectController
{
public:
	InteractiveObjectController(ros::NodeHandle &nh);
	void spinner(void);

private:
	bool enable_debug;
	bool single_contact;
	ros::NodeHandle nh;

	// Object stuff
	ros::Publisher object_pub;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object;
	sensor_msgs::PointCloud2 cloud_msg_object;
	geometry_msgs::TransformStamped transformStamped_object;
	void loadObjectPly(void);
	void transformPcForTaxim(void);

	// Sensor stuff
	ros::Publisher sensor_pub;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor;
	sensor_msgs::PointCloud2 cloud_msg_sensor;
	geometry_msgs::TransformStamped transformStamped_sensor;
	void loadSensorPly(void);
	
	void initTransform(void); // for both sensor and object

	// Pointcloud and msg to be elaborated befor sent to Taxim Simulator
	ros::Publisher pub_firstFingerContact;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_firstFingerTaximPointcloud;
	sensor_msgs::PointCloud2 cloud_msg_firstFingerTaximPointcloud;
	ros::Publisher pub_secondFingerContact;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_secondFingerTaximPointcloud;
	sensor_msgs::PointCloud2 cloud_msg_secondFingerTaximPointcloud;

	// Dynamic reconfigure stuff
	dynamic_reconfigure::Server<interactive_object_controller::object_controllerConfig> server_;
	dynamic_reconfigure::Server<interactive_object_controller::object_controllerConfig>::CallbackType f_;
	void configCallback(interactive_object_controller::object_controllerConfig &config, uint32_t level);


	// Use these two functions to move the pointcloud of the sensor and to save it properly
	// ---- NB that you have to modify manually the header part of the ply stored
	void moveSensorReferenceFrame(void);
	void saveAsPly(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void centralizeAndTransformPoints(void);
};

#endif /* INTERACTIVE_OBJECT_CONTROLLER_HPP */
