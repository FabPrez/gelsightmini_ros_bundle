#include "../include/interactive_object_controller.hpp"

InteractiveObjectController::InteractiveObjectController(ros::NodeHandle &nh) : tf_listener(tf_buffer)
{
  this->nh = nh;
  nh.getParam("/interactive_object_controller_node/enable_debug", enable_debug);
  ROS_INFO("Enable debug: %s", enable_debug ? "true" : "false");

  object_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc", 1);
  sensor_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor", 1);
  taximPointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_contact_pointcloud", 1);
  save_pointcloud_client = nh.serviceClient<taxim_pointcloud_preparator::SavePointcloud>("save_pointcloud");

  cloud_object = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_sensor = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_taximPointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  loadObjectPly();
  initObjTransform();
  loadSensorPly();

  // Dynamic reconfigure stuff
  f_ = boost::bind(&InteractiveObjectController::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void InteractiveObjectController::loadObjectPly()
{
  // Load object .PLY and publish it
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
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

  pcl::toROSMsg(*cloud_object, cloud_msg_object);
  cloud_msg_object.header.frame_id = "frame_object";

  // obj_transform();
}

void InteractiveObjectController::loadSensorPly()
{
  // Load Sensor .PLY and publish it
  std::string sensor_ply_path;
  std::string data_folder_path = ros::package::getPath("data_folder"); // Nome del pacchetto
  std::string relative_path = "obj_ply/gsmini_pointcloud.ply";         // Percorso relativo all'interno del pacchetto
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
  pcl::toROSMsg(*cloud_sensor, cloud_msg_sensor);
  cloud_msg_sensor.header.frame_id = "frame_sensor";

  sensor_trasform();
}

void InteractiveObjectController::initObjTransform()
{
  transformStamped_object.header.frame_id = "world";
  transformStamped_object.child_frame_id = "frame_object";
  cloud_msg_taximPointcloud.header.frame_id = "frame_object";

  // Imposta la traslazione (posizione)
  transformStamped_object.transform.translation.x = 0;
  transformStamped_object.transform.translation.y = 0;
  transformStamped_object.transform.translation.z = 0;

  // Imposta la rotazione (orientazione)
  transformStamped_object.transform.rotation.x = 0.0;
  transformStamped_object.transform.rotation.y = 0.0;
  transformStamped_object.transform.rotation.z = 0.0;
  transformStamped_object.transform.rotation.w = 1.0; // No rotation
}

void InteractiveObjectController::sensor_trasform()
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  // Imposta l'header del messaggio TF
  transformStamped_sensor.header.stamp = ros::Time::now();
  transformStamped_sensor.header.frame_id = "world";       // Frame di riferimento
  transformStamped_sensor.child_frame_id = "frame_sensor"; // Frame del tuo oggetto

  // Imposta la traslazione (posizione)
  transformStamped_sensor.transform.translation.x = 0.0;
  transformStamped_sensor.transform.translation.y = 0.0;
  transformStamped_sensor.transform.translation.z = 0.0;

  // Imposta la rotazione (orientazione)
  transformStamped_sensor.transform.rotation.x = 0.0;
  transformStamped_sensor.transform.rotation.y = 0.0;
  transformStamped_sensor.transform.rotation.z = 0.0;
  transformStamped_sensor.transform.rotation.w = 1.0; // Nessuna rotazione

  transformStamped_sensor.header.stamp = ros::Time::now();
  static_broadcaster.sendTransform(transformStamped_sensor);
}

void InteractiveObjectController::configCallback(interactive_object_controller::object_controllerConfig &config, uint32_t level)
{
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

  // Call service to save the pcl
  // TODO
  trasformPcForTaxim();
  // taxim_pointcloud_preparator::SavePointcloud srv;
  // if (save_pointcloud_client.call(srv))
  // {
  //   ROS_INFO("SavePointcloud success: %d", srv.response.success);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service save_pointcloud");
  // }

  // Call service to let the gelsight simulator publish the relative image
  // TODO
}

void InteractiveObjectController::trasformPcForTaxim(void)
{
  geometry_msgs::TransformStamped transform;
  transform = transformStamped_object;
  std::string world_frame_id = "world";

  // try
  // {
  //   transform = tf_buffer.lookupTransform(world_frame_id, cloud_msg_object.header.frame_id, ros::Time(0));
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  //   return;
  // }

  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix(transform.transform, eigen_transform);

  pcl::transformPointCloud(*cloud_object, *cloud_taximPointcloud, eigen_transform);

  pcl::toROSMsg(*cloud_taximPointcloud, cloud_msg_taximPointcloud);
  
  taximPointcloud_pub.publish(cloud_msg_taximPointcloud);
  ROS_INFO("object_contact_pointcloud generated!");
}

void InteractiveObjectController::spinner()
{
  // obj_transform();
  object_pub.publish(cloud_msg_object);
  // sensor_trasform();
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
