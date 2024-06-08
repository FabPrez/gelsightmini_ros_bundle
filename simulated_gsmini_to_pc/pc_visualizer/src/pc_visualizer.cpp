#include "../include/pc_visualizer.hpp"

pc_visualizer::pc_visualizer(ros::NodeHandle &nh) : tf_listener(tf_buffer)
{
  ROS_INFO("Ros node partito!");
  this->nh = nh;
  // Inizializzazione dei publisher
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc", 1);
  object_pub_modified = nh.advertise<sensor_msgs::PointCloud2>("/object_pc_modified", 1);
  sensor_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor", 1);

  cloud_object = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_object_modified = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_sensor = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  loadObjectPly();
  // object_pub.publish(cloud_msg_object);

  loadSensorPly();
  // sensor_pub.publish(cloud_msg_sensor);
}

void pc_visualizer::obj_transform()
{
  // Imposta l'header del messaggio TF
  transformStamped_object.header.stamp = ros::Time::now();
  transformStamped_object.header.frame_id = "world";       // Frame di riferimento
  transformStamped_object.child_frame_id = "frame_object"; // Frame del tuo oggetto

  // Imposta la traslazione (posizione)
  transformStamped_object.transform.translation.x = 0; // Esempio: spostamento di 1 metro lungo l'asse x
  transformStamped_object.transform.translation.y = 70;
  transformStamped_object.transform.translation.z = 10;

  // Imposta la rotazione (orientazione)
  transformStamped_object.transform.rotation.x = 0.0;
  transformStamped_object.transform.rotation.y = 0.0;
  transformStamped_object.transform.rotation.z = 1.0;
  transformStamped_object.transform.rotation.w = 0.0; // Nessuna rotazione

  // Pubblica il messaggio TF
  br_object.sendTransform(transformStamped_object);

  // ROS_INFO("Messaggio object_frame TF pubblicato");
}

void pc_visualizer::sensor_trasform()
{
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

  // Pubblica il messaggio TF
  br_sensor.sendTransform(transformStamped_sensor);

  // ROS_INFO("Messaggio sensor_frame TF pubblicato con timestamp %f", transformStamped_sensor.header.stamp.toSec());
  // ROS_INFO("Messaggio sensor_frame TF pubblicato");
}

void pc_visualizer::loadObjectPly()
{
  // Load object .PLY and publish it
  std::string obj_ply_path;
  nh.getParam("/pc_visualizer_node/obj_ply_path", obj_ply_path);
  ROS_INFO("Obj .ply path: %s", obj_ply_path.c_str());

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(obj_ply_path, *cloud_object) == -1)
  {
    ROS_INFO("Impossibile caricare il file .ply dell'Oggetto");
    return;
  }

  ROS_INFO("PLY OGGETTO CONVERTITA IN PCD - Caricamento completato!");
  pcl::toROSMsg(*cloud_object, cloud_msg_object);
  cloud_msg_object.header.frame_id = "frame_object";

  // ros::Duration(1).sleep();
  obj_transform();
}

void pc_visualizer::loadSensorPly()
{
  // Load Sensor .PLY and publish it
  std::string sensor_ply_path;
  std::string data_folder_path = ros::package::getPath("data_folder"); // Nome del pacchetto
  std::string relative_path = "obj_ply/gsmini_pointcloud.ply";         // Percorso relativo all'interno del pacchetto
  std::string sensor_file_path = data_folder_path + "/" + relative_path;
  ROS_INFO("Sensor .ply path: %s", sensor_file_path.c_str());

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(sensor_file_path, *cloud_sensor) == -1)
  {
    ROS_INFO("Impossibile caricare il file .ply del Sensore");
    return;
  }

  ROS_INFO("PLY SENSORE CONVERTITA IN PCD - Caricamento completato!");
  // moveSensorReferenceFrame(); // use only if you want to move the pointcloud
  pcl::toROSMsg(*cloud_sensor, cloud_msg_sensor);
  cloud_msg_sensor.header.frame_id = "frame_sensor";

  // ros::Duration(1).sleep();
  sensor_trasform();
  // saveAsPly(sensor_file_path, cloud_sensor);
}

void pc_visualizer::moveSensorReferenceFrame()
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

void pc_visualizer::saveAsPly(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

void pc_visualizer::centralizeAndTransformPoints()
{

  // float dx = 0;
  // float dy = 0;
  // float img_width = 240.0;
  // float img_height = 320.0;
  // float pixmm = 0.0634;

  // *cloud_object_modified = *cloud_object;

  // if (cloud_object_modified->empty())
  // {
  //   ROS_ERROR("La nuvola di punti è vuota, impossibile trasformare.");
  //   return;
  // }

  // // Estrai i punti come un Eigen::MatrixXf
  // Eigen::MatrixXf points(cloud_object_modified->size(), 3);
  // for (size_t i = 0; i < cloud_object_modified->size(); ++i)
  // {
  //   points(i, 0) = cloud_object_modified->points[i].x;
  //   points(i, 1) = cloud_object_modified->points[i].y;
  //   points(i, 2) = cloud_object_modified->points[i].z;
  // }

  // // Calcola il centroide della nuvola di punti
  // Eigen::Vector3f centroid = points.colwise().mean();

  // // Centralizza i punti
  // points.rowwise() -= centroid.transpose();

  // // Trasforma le coordinate dei punti
  // Eigen::ArrayXf uu_float = (points.col(0).array() / pixmm) + img_width / 2.0f + dx;
  // Eigen::ArrayXf vv_float = (points.col(1).array() / pixmm) + img_height / 2.0f + dy;

  // // Cast the transformed coordinates to integers
  // Eigen::VectorXi uu = uu_float.cast<int>();
  // Eigen::VectorXi vv = vv_float.cast<int>();

  // // // Optional: Stampa per debug
  // // for (size_t i = 0; i < uu.size(); ++i) {
  // //     ROS_INFO("Point %zu: (uu, vv) = (%d, %d)", i, static_cast<int>(uu(i)), static_cast<int>(vv(i)));
  // // }

  // // Apply the transformation to the actual point cloud
  // for (size_t i = 0; i < cloud_object_modified->size(); ++i)
  // {
  //   cloud_object_modified->points[i].x = uu(i);
  //   cloud_object_modified->points[i].y = vv(i);
  //   // Assuming we don't change the z coordinate
  // }

  // // Filter points based on conditions
  // std::vector<bool> mask_u(uu.size());
  // std::vector<bool> mask_v(vv.size());
  // std::vector<bool> mask_z(points.rows());
  // std::vector<bool> mask_map(points.rows());

  // for (size_t i = 0; i < uu.size(); ++i)
  // {
  //   mask_u[i] = (uu(i) > 0) && (uu(i) < img_width);
  //   mask_v[i] = (vv(i) > 0) && (vv(i) < img_height);
  //   mask_z[i] = points(i, 2) > 0.2;
  //   mask_map[i] = mask_u[i] && mask_v[i] && mask_z[i];
  // }

  // // Apply the mask to the point cloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_modified(new pcl::PointCloud<pcl::PointXYZ>);
  // for (size_t i = 0; i < points.rows(); ++i)
  // {
  //   if (mask_map[i])
  //   {
  //     pcl::PointXYZ point;
  //     point.x = uu(i);
  //     point.y = vv(i);
  //     point.z = points(i, 2);
  //     cloud_object_modified->points.push_back(point);
  //   }
  // }

  // pcl::toROSMsg(*cloud_object_modified, cloud_msg_object_modified);
  // cloud_msg_object_modified.header.frame_id = "frame_object";

  // ros::Duration(1).sleep();
  // object_pub_modified.publish(cloud_msg_object_modified);
  // ROS_INFO("object_pub_modified pubblicato");
  float dx = 0;
  float dy = 0;
  float img_width = 240.0;
  float img_height = 320.0;
  float pixmm = 0.0634;

  if (cloud_object->empty())
  {
    ROS_ERROR("La nuvola di punti è vuota, impossibile trasformare.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_modified(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_object_modified = *cloud_object;

  // Extract points as an Eigen::MatrixXf
  Eigen::MatrixXf points(cloud_object_modified->size(), 3);
  for (size_t i = 0; i < cloud_object_modified->size(); ++i)
  {
    points(i, 0) = cloud_object_modified->points[i].x;
    points(i, 1) = cloud_object_modified->points[i].y;
    points(i, 2) = cloud_object_modified->points[i].z;
  }

  // Compute the centroid of the point cloud
  Eigen::Vector3f centroid = points.colwise().mean();

  // Centralize the points
  points.rowwise() -= centroid.transpose();

  // Transform the coordinates of the points
  Eigen::ArrayXf uu_float = (points.col(0).array() / pixmm) + img_width / 2.0f + dx;
  Eigen::ArrayXf vv_float = (points.col(1).array() / pixmm) + img_height / 2.0f + dy;

  // Cast the transformed coordinates to integers
  Eigen::VectorXi uu = uu_float.cast<int>();
  Eigen::VectorXi vv = vv_float.cast<int>();

  // Filter points based on conditions
  std::vector<bool> mask_u(uu.size());
  std::vector<bool> mask_v(vv.size());
  std::vector<bool> mask_z(points.rows());
  std::vector<bool> mask_map(points.rows());

  for (size_t i = 0; i < uu.size(); ++i)
  {
    mask_u[i] = (uu(i) > 0) && (uu(i) < img_width);
    mask_v[i] = (vv(i) > 0) && (vv(i) < img_height);
    mask_z[i] = points(i, 2) > 0.2;
    mask_map[i] = mask_u[i] && mask_v[i] && mask_z[i];
  }

  // Apply the mask to the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < points.rows(); ++i)
  {
    if (mask_map[i])
    {
      pcl::PointXYZ point;
      point.x = uu(i);
      point.y = vv(i);
      point.z = points(i, 2);
      filtered_cloud->points.push_back(point);
    }
  }

  // Publish the filtered point cloud
  pcl::toROSMsg(*filtered_cloud, cloud_msg_object_modified);
  cloud_msg_object_modified.header.frame_id = "frame_object";

  ros::Duration(1).sleep();
  object_pub_modified.publish(cloud_msg_object_modified);
  ROS_INFO("object_pub_modified pubblicato");
}

void pc_visualizer::prepareForSimulation()
{

  std::string sensor_frame = cloud_msg_sensor.header.frame_id;
  std::string object_frame = cloud_msg_object.header.frame_id;
  ROS_INFO("Frame del sensore: %s", sensor_frame.c_str());
  ROS_INFO("Frame dell'oggetto: %s", object_frame.c_str());

  geometry_msgs::TransformStamped transformStamped;
  // obj_transform();
  // sensor_trasform();

  try
  {
    transformStamped = tf_buffer.lookupTransform(sensor_frame, object_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Extract translation and rotation from geometry_msgs::TransformStamped
  Eigen::Translation3d translation(transformStamped.transform.translation.x,
                                   transformStamped.transform.translation.y,
                                   transformStamped.transform.translation.z);

  Eigen::Quaterniond rotation(transformStamped.transform.rotation.w,
                              transformStamped.transform.rotation.x,
                              transformStamped.transform.rotation.y,
                              transformStamped.transform.rotation.z);

  // Combine translation and rotation into an Eigen::Affine3d
  Eigen::Affine3d eigen_transform = translation * rotation;
  Eigen::Affine3f eigen_transform_f = eigen_transform.cast<float>();
  // Applica la trasformazione alla nuvola di punti
  pcl::transformPointCloud(*cloud_object, *cloud_object_modified, eigen_transform_f);

  cloud_msg_object_modified.header.frame_id = "frame_object";
  pcl::toROSMsg(*cloud_object_modified, cloud_msg_object_modified);
  object_pub_modified.publish(cloud_msg_object_modified);
  ROS_INFO("object_pub_modified pubblicato");
}

void pc_visualizer::spinner()
{
  obj_transform();
  object_pub.publish(cloud_msg_object);
  sensor_trasform();
  sensor_pub.publish(cloud_msg_sensor);
}
