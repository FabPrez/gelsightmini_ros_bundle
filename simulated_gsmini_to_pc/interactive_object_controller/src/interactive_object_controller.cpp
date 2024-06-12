#include "../include/interactive_object_controller.hpp"

InteractiveObjectController::InteractiveObjectController(ros::NodeHandle &nh)
{
  this->nh = nh;
  nh.getParam("/interactive_object_controller_node/enable_debug", enable_debug);
  ROS_INFO("Enable debug: %s", enable_debug ? "true" : "false");
  nh.getParam("/interactive_object_controller_node/single_contact", single_contact);
  ROS_INFO("Single contact: %s", single_contact ? "true" : "false");

  // Publishers
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc", 1);
  sensor_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor", 1);
  pub_firstFingerContact = nh.advertise<sensor_msgs::PointCloud2>("/first_finger_contact_pointcloud", 1);

  // Pointclouds initialization
  cloud_object = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_sensor = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_firstFingerTaximPointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  cloud_secondFingerTaximPointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pub_secondFIngerContact = nh.advertise<sensor_msgs::PointCloud2>("/second_finger_contact_pointcloud", 1);

  loadObjectPly();
  pcl::toROSMsg(*cloud_object, cloud_msg_object);
  loadSensorPly();
  pcl::toROSMsg(*cloud_sensor, cloud_msg_sensor);

  cloud_msg_object.header.frame_id = "frame_object";
  cloud_msg_sensor.header.frame_id = "frame_sensor";

  initTransform();

  // Dynamic reconfigure stuff
  f_ = boost::bind(&InteractiveObjectController::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void InteractiveObjectController::loadObjectPly()
{
  // Load object .PLY and publish it
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  // load the object from the folder "data_folder", take a look on the launch file to see it!
  std::string obj_ply_path;
  nh.getParam("/interactive_object_controller_node/obj_ply_path", obj_ply_path);
  if (enable_debug)
    ROS_INFO("Obj .ply path: %s", obj_ply_path.c_str());

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(obj_ply_path, *cloud_object) == -1)
  {
    ROS_INFO("Impossibile caricare il file .ply dell'Oggetto");
    return;
  }

  if (enable_debug)
    ROS_INFO("PLY object uploaded!");
}

void InteractiveObjectController::loadSensorPly()
{
  // Load Sensor .PLY
  std::string sensor_ply_path;
  std::string data_folder_path = ros::package::getPath("data_folder"); // Nome del pacchetto
  std::string relative_path = "obj_ply/gsmini_pointcloud.ply";
  std::string sensor_file_path = data_folder_path + "/" + relative_path;
  if (enable_debug)
    ROS_INFO("Sensor .ply path: %s", sensor_file_path.c_str());

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(sensor_file_path, *cloud_sensor) == -1)
  {
    ROS_INFO("Impossibile caricare il file .ply del Sensore");
    return;
  }

  if (enable_debug)
    ROS_INFO("PLY sensor file uploaded!");
}

void InteractiveObjectController::initTransform()
{
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // Send static object reference frame
  transformStamped_object.header.frame_id = "world";
  transformStamped_object.child_frame_id = "frame_object";

  transformStamped_object.transform.translation.x = 0;
  transformStamped_object.transform.translation.y = 0;
  transformStamped_object.transform.translation.z = 0;

  transformStamped_object.transform.rotation.x = 0.0;
  transformStamped_object.transform.rotation.y = 0.0;
  transformStamped_object.transform.rotation.z = 0.0;
  transformStamped_object.transform.rotation.w = 1.0;

  transformStamped_object.header.stamp = ros::Time::now();
  static_broadcaster.sendTransform(transformStamped_object);

  // Send static sensor reference frame
  transformStamped_sensor.header.frame_id = "world";
  transformStamped_sensor.child_frame_id = "frame_sensor";

  transformStamped_sensor.transform.translation.x = 0.0;
  transformStamped_sensor.transform.translation.y = 0.0;
  transformStamped_sensor.transform.translation.z = 0.0;

  transformStamped_sensor.transform.rotation.x = 0.0;
  transformStamped_sensor.transform.rotation.y = 0.0;
  transformStamped_sensor.transform.rotation.z = 0.0;
  transformStamped_sensor.transform.rotation.w = 1.0;

  transformStamped_sensor.header.stamp = ros::Time::now();
  static_broadcaster.sendTransform(transformStamped_sensor);
}

void InteractiveObjectController::configCallback(interactive_object_controller::object_controllerConfig &config, uint32_t level)
{
  // Dynamic reconfigure callback - update the object position every time the parameters are changed
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  transformStamped_object.transform.translation.x = config.translation_x;
  transformStamped_object.transform.translation.y = config.translation_y;
  transformStamped_object.transform.translation.z = config.translation_z;

  tf2::Quaternion quat;
  quat.setRPY(config.rotation_x, config.rotation_y, config.rotation_z);
  quat.normalize();
  transformStamped_object.transform.rotation.x = quat.x();
  transformStamped_object.transform.rotation.y = quat.y();
  transformStamped_object.transform.rotation.z = quat.z();
  transformStamped_object.transform.rotation.w = quat.w();

  transformStamped_object.header.stamp = ros::Time::now();
  static_broadcaster.sendTransform(transformStamped_object);

  trasformPcForTaxim();
}

void InteractiveObjectController::trasformPcForTaxim(void)
{
  //  --- This function takes the actual pointcloud, transform it to process the right contact part and publish it
  // the messages are used from the package "taxim_pointcloud_simulator" ---

  // ! the trasformation is done based on the 0,0,0 reference frame, so the Sensor must be in that position!

  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix(transformStamped_object.transform, eigen_transform);
  pcl::transformPointCloud(*cloud_object, *cloud_firstFingerTaximPointcloud, eigen_transform);

  // Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
  // rotation_matrix(2, 2) = -1;
  // rotation_matrix(0, 0) = -1;
  // pcl::transformPointCloud(*cloud_firstFingerTaximPointcloud, *cloud_firstFingerTaximPointcloud, rotation_matrix);
  // pcl::transformPointCloud(*cloud_firstFingerTaximPointcloud, *cloud_firstFingerTaximPointcloud, Eigen::Affine3f(Eigen::Translation3f(0, 0, 500)));
  pcl::toROSMsg(*cloud_firstFingerTaximPointcloud, cloud_msg_firstFingerTaximPointcloud);
  cloud_msg_firstFingerTaximPointcloud.header.frame_id = "frame_object";
  pub_firstFingerContact.publish(cloud_msg_firstFingerTaximPointcloud);

  // if (!single_contact) return;

  // // --- Second finger contact pointcloud ---
  // Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
  // rotation_matrix(2, 2) = -1;
  // rotation_matrix(0, 0) = -1;

  // // Let's apply the rotation matrix to the first finger pointcloud
  // pcl::transformPointCloud(*cloud_firstFingerTaximPointcloud, *cloud_secondFingerTaximPointcloud, rotation_matrix);
  // pcl::toROSMsg(*cloud_secondFingerTaximPointcloud, cloud_msg_secondFingerTaximPointcloud);
  // cloud_msg_secondFingerTaximPointcloud.header.frame_id = "frame_object";
  // pub_secondFIngerContact.publish(cloud_msg_secondFingerTaximPointcloud);
}

void InteractiveObjectController::spinner()
{
  object_pub.publish(cloud_msg_object);
  sensor_pub.publish(cloud_msg_sensor);
}

// Other maybe usefull functions -----
void InteractiveObjectController::moveSensorReferenceFrame()
{
  if (cloud_sensor->empty())
  {
    ROS_ERROR("La nuvola di punti è vuota, impossibile trasformare.");
    return;
  }

  // Trova il punto con la coordinata x massima
  pcl::PointXYZ max_point = cloud_sensor->points[0];
  for (const auto &point : cloud_sensor->points)
  {
    if (point.y > max_point.y)
    {
      max_point = point;
    }
  }

  ROS_INFO("Il punto con la coordinata x massima è: (%f, %f, %f)", max_point.x, max_point.y, max_point.z);

  // Calcola la traslazione necessaria per portare max_point all'origine
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -max_point.x, -max_point.y, -max_point.z;

  // Applica la traslazione alla nuvola di punti
  pcl::transformPointCloud(*cloud_sensor, *cloud_sensor, transform);

  ROS_INFO("Nuvola di punti trasformata in modo che il punto con x massima sia all'origine.");

  float theta = M_PI / 2; // 90 gradi in radianti
  transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

  // Applica la rotazione alla nuvola di punti
  pcl::transformPointCloud(*cloud_sensor, *cloud_sensor, transform);

  ROS_INFO("Nuvola di punti ruotata di 90 gradi rispetto all'asse X.");
}

void InteractiveObjectController::saveAsPly(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  if (cloud->empty())
  {
    ROS_ERROR("La nuvola di punti è vuota, impossibile salvare.");
    return;
  }

  if (pcl::io::savePLYFile(file_path, *cloud) == -1)
  {
    ROS_ERROR("Impossibile salvare il file .ply: %s", file_path.c_str());
    return;
  }
  ROS_INFO("Nuvola di punti salvata con successo: %s", file_path.c_str());

  ROS_INFO("PLY SAVED!");
}
