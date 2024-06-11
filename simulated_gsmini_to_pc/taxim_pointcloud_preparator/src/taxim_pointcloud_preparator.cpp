#include "taxim_pointcloud_preparator.hpp"

TaximPointCloudPreparator::TaximPointCloudPreparator(ros::NodeHandle &nh) : tf_listener(tf_buffer)
{
  this->nh = nh;
  nh.getParam("/taxim_pointcloud_preparator_node/enable_debug", enable_debug);
  ROS_INFO("Enable debug: %s", enable_debug ? "true" : "false");

  sub_obj_pointclod = nh.subscribe("object_pc", 1, &TaximPointCloudPreparator::pointCloudCallback, this);
  service_save_pointcloud = nh.advertiseService("save_pointcloud", &TaximPointCloudPreparator::trasformAndSavePointCloud, this);
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_pc_modified", 1);
  
  firstFinger_pub = nh.advertise<sensor_msgs::PointCloud2>("object_contact_pointcloud", 1);

  if (enable_debug)
  {
    firstFinger_pub = nh.advertise<sensor_msgs::PointCloud2>("first_finger_pc", 1);
    secondFinger_pub = nh.advertise<sensor_msgs::PointCloud2>("second_finger_pc", 1);
  }
}

void TaximPointCloudPreparator::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  pcl::fromROSMsg(*input, *cloud_object);
  cloud_object_frame_id = input->header.frame_id;
}

bool TaximPointCloudPreparator::generateTwoFingerContact(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // It is used world because is the same as sensor frame but it has to be fixed
  geometry_msgs::TransformStamped transform;
  std::string world_frame_id = "world";

  try
  {
    transform = tf_buffer.lookupTransform(world_frame_id, cloud_object_frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // Prepare the actual pointcloud position to generate the two finger contact
  // This is for the first finger position
  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix(transform.transform, eigen_transform);

  PointCloud::Ptr firstFinger_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud_object, *firstFinger_cloud, eigen_transform);

  pcl::toROSMsg(*firstFinger_cloud, firstFinger_cloud_msg);
  firstFinger_cloud_msg.header.frame_id = "frame_object";
  firstFinger_pub.publish(firstFinger_cloud_msg);
  ROS_INFO("object_contact_pointcloud generated!");

  if (enable_debug)
  {
    // DEBUG: Publish the first finger transformed pointcloud
    pcl::toROSMsg(*firstFinger_cloud, firstFinger_cloud_msg);
    firstFinger_cloud_msg.header.frame_id = "frame_object";
    firstFinger_pub.publish(firstFinger_cloud_msg);
    ROS_INFO("First finger trasfomerd cloud published!");
  }

  // saveAsPly(firstFinger_cloud, "firstFinger"); // It will be sed for Taxim simulator

  // // This is for the second finger position
  // Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
  // rotation_matrix(2, 2) = -1;
  // rotation_matrix(0, 0) = -1;

  // // Let's apply the rotation matrix to the first finger pointcloud
  // PointCloud::Ptr secondFinger_cloud(new PointCloud());
  // pcl::transformPointCloud(*firstFinger_cloud, *secondFinger_cloud, rotation_matrix);

  // sensor_msgs::PointCloud2 secondFinger_cloud_msg;
  // if (enable_debug)
  // {
  //   // DEBUG: Pubblica la nuvola di punti trasformata del secondo dito
  //   pcl::toROSMsg(*secondFinger_cloud, secondFinger_cloud_msg);
  //   secondFinger_cloud_msg.header.frame_id = "frame_object";
  //   secondFinger_pub.publish(secondFinger_cloud_msg);
  //   ROS_INFO("Second finger transformed cloud published!");
  // }

  // saveAsPly(secondFinger_cloud, "secondFinger"); // It will be sed for Taxim simulator

  return true;
}

bool TaximPointCloudPreparator::trasformAndSavePointCloud(taxim_pointcloud_preparator::SavePointcloud::Request &req,
                                                          taxim_pointcloud_preparator::SavePointcloud::Response &res)
{
  saveAsPcd(cloud_object); // I need to save the actual position of the original pointcloud

  if (!generateTwoFingerContact(cloud_object))
  {
    ROS_ERROR("Impossibile generare il contatto a due dita.");
    return false;
  }
  res.success = true;
  return true;
}

void TaximPointCloudPreparator::saveAsPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fingerNumber)
{
  std::string taxim_obj_ply_path;
  nh.getParam("/taxim_pointcloud_preparator_node/taxim_obj_ply_path", taxim_obj_ply_path);
  // Aggiungi il nome del dito e riaggiungi l'estensione ".ply"
  taxim_obj_ply_path = taxim_obj_ply_path.substr(0, taxim_obj_ply_path.size() - 4) + "_" + fingerNumber + ".ply";

  if (enable_debug)
    ROS_INFO("----Obj .ply path: %s", taxim_obj_ply_path.c_str());

  if (cloud->empty())
  {
    ROS_ERROR("La nuvola di punti è vuota, impossibile salvare.");
    return;
  }

  if (pcl::io::savePLYFile(taxim_obj_ply_path, *cloud) == -1)
  {
    ROS_ERROR("Impossibile salvare il file .ply: %s", taxim_obj_ply_path.c_str());
    return;
  }
  ROS_INFO("Nuvola di punti PLY salvata con successo: %s", taxim_obj_ply_path.c_str());
}
void TaximPointCloudPreparator::saveAsPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::string obj_pcd_path;
  nh.getParam("/taxim_pointcloud_preparator_node/obj_pcd_path", obj_pcd_path);

  if (enable_debug)
    ROS_INFO("----Obj .pcd path: %s", obj_pcd_path.c_str());

  if (cloud->empty())
  {
    ROS_ERROR("La nuvola di punti è vuota, impossibile salvare.");
    return;
  }

  if (pcl::io::savePCDFileASCII(obj_pcd_path, *cloud) == -1)
  {
    ROS_ERROR("Impossibile salvare il file .pcd: %s", obj_pcd_path.c_str());
    return;
  }
  ROS_INFO("Nuvola di punti PCD salvata con successo: %s", obj_pcd_path.c_str());
}

void TaximPointCloudPreparator::spinner(void)
{
  ros::spin();
}