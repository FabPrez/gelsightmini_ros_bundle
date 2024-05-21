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
    pub = rospy.Publisher('/my_pointcloud_topic', PointCloud2, queue_size=10)
    
    # Esegui altre operazioni necessarie qui
    
    rospy.spin()

if __name__ == '__main__':
    main()
