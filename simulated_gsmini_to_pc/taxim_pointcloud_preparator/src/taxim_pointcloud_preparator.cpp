#include "taxim_pointcloud_preparator.hpp"

TaximPointCloudPreparator::TaximPointCloudPreparator(ros::NodeHandle &nh) : tf_listener(tf_buffer)
{
  this->nh = nh;
  sub_obj_pointclod = nh.subscribe("object_pc", 1, &TaximPointCloudPreparator::pointCloudCallback, this);
  service_save_pointcloud = nh.advertiseService("save_pointcloud", &TaximPointCloudPreparator::trasformAndSavePointCloud, this);
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_pc_modified", 1);
}

void TaximPointCloudPreparator::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  pcl::fromROSMsg(*input, *cloud_object);
  cloud_object_frame_id = input->header.frame_id;
}

bool TaximPointCloudPreparator::trasformAndSavePointCloud(taxim_pointcloud_preparator::SavePointcloud::Request &req,
                               taxim_pointcloud_preparator::SavePointcloud::Response &res)
{
  std::string file_path = ros::package::getPath("taxim_pointcloud_preparator") + "/pointcloud.ply";

  geometry_msgs::TransformStamped transform;
  std::string world_frame_id = "world";
  try
  {
    transform = tf_buffer.lookupTransform(world_frame_id, cloud_object_frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    res.success = false;
    return false;
  }

  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix(transform.transform, eigen_transform);

  PointCloud::Ptr transformed_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud_object, *transformed_cloud, eigen_transform);


  // pubblico l'oggetto per visualizzarlo
  pcl::toROSMsg(*transformed_cloud, cloud_msg_object_modified);
  cloud_msg_object_modified.header.frame_id = "frame_object";
  object_pub.publish(cloud_msg_object_modified);
  ROS_INFO("Pubblicato oggetto trasformato");

  // Salva la point cloud
  if (pcl::io::savePLYFile(file_path, *transformed_cloud) == -1)
  {
    ROS_ERROR("Impossibile salvare il file .ply: %s", file_path.c_str());
    res.success = false;
    return false;
  }

  // Se il salvataggio è riuscito, imposta la risposta del servizio a true
  ROS_INFO("Nuvola di punti salvata con successo: %s", file_path.c_str());
  res.success = true;
  return true;
}
