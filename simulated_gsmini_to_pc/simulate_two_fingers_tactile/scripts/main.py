#!/home/fabioprez/gsmini_ws/src/gelsightmini_ros_bundle/gsmini_venv/bin/python

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..', '..', '..')))
sys.path.append("..")
print('path is ------: ',sys.path)
import rospkg
import rospy
from sensor_msgs.msg import PointCloud2
from Taxim.OpticalSimulation.simOptical_gsmini import simulator
import numpy as np
import cv2




def main():
    rospy.init_node('simulated_gsmini_to_pc')
    
    # Take the object from the folder "objects" in the package "simulated_gsmini_to_pc"
    obj_path = rospy.get_param('~obj_path', 'default_object')
    # rospy.loginfo('ROS obj path: %s', obj_path)
    gsminiSimulator = simulator(obj_path)
    sim_img, shadow_sim_img, heightMap = gsminiSimulator.generateSimulatedImages(1,0,0)
    
    # save images and heightmap in the folder "results" in the package "simulated_gsmini_to_pc"  
    result_path =  os.path.abspath(os.path.join(os.path.dirname(__file__), '..','results'))
    img_savePath = os.path.join(result_path,'_gsmini_sim.jpg')
    # rospy.loginfo('img_savePath : %s', img_savePath)
    shadow_savePath = os.path.join(result_path,'_gsmini_shadow.jpg')
    height_savePath = os.path.join(result_path,'_gsmini_height.npy')
    cv2.imwrite(img_savePath, sim_img)
    cv2.imwrite(shadow_savePath, shadow_sim_img)
    np.save(height_savePath, heightMap)
    
    rospy.loginfo('Images generated and stored in the folder "results"!!!')
    
    # pub = rospy.Publisher('gsmini_pointcloud_topic', PointCloud2, queue_size=10)
    # rate = rospy.Rate(10)  # 10 Hz
    # # load and publish the pointcloud

    # rospack = rospkg.RosPack()
    # config_path = rospack.get_path('simulated_gsmini_to_pc')
    # ply_path = os.path.join(config_path, 'config', 'gsmini_pointcloud.ply')
    # rospy.loginfo('ply_path : %s', ply_path)

    # cloud = pcl.load_XYZRGB(ply_path)  # Assuming your PLY file contains XYZRGB data
    
    # while not rospy.is_shutdown():
    #     # Convert PCL point cloud to ROS PointCloud2 message
    #     ros_cloud = pcl_to_ros(cloud)
    #     rospy.loginfo('Publishing PointCloud2 message')
    #     # Publish the ROS PointCloud2 message
    #     pub.publish(ros_cloud)
    #     rate.sleep()
    
    

if __name__ == '__main__':
    main()
    
    
def pcl_to_ros(pcl_cloud):
    """Convert PCL PointCloud to ROS PointCloud2"""
    cloud_msg = PointCloud2()
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = "world"  # Set your desired frame ID here
    cloud_msg.height = 1
    cloud_msg.width = pcl_cloud.size
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    cloud_msg.is_dense = True
    cloud_msg.data = np.asarray(pcl_cloud).tostring()
    return cloud_msg
