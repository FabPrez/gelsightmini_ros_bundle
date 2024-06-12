#!/home/fabioprez/gsmini_ws/src/gelsightmini_ros_bundle/gsmini_venv/bin/python

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..', '..', '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'..', '..', '..','..')))
sys.path.append('.')
print('path is ------: ',sys.path)

import rospkg
import rospy
import numpy as np
import cv2

from Taxim.OpticalSimulation.simOptical_gsmini import simulator as gsmini_simulator
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import String

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image

class SimulatorNode:
    def __init__(self):
        print ("Ready to generate simulated images.")
        rospy.init_node('tactile_image_simulator_node')
        self.verts_num = 0
        self.vertices = None
        self.pc_subscriber = rospy.Subscriber('/first_finger_contact_pointcloud', PointCloud2, self.pc_callback)
        # self.simulate_serv = rospy.Service('simulated_gsmini', SimulatedGSmini, self.handle_simulated_gsmini)
        self.image_publisher = rospy.Publisher('/first_finger_simulated_image', Image, queue_size=10)
        self.bridge = CvBridge()
        
    def pc_callback(self, msg):
        # Convert PointCloud2 message to a numpy array
        points_list = []

        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        self.vertices = np.array(points_list)
        self.verts_num = len(self.vertices)
        gsminiSimulator = gsmini_simulator(self.vertices)
        sim_img, shadow_sim_img, heightMap = gsminiSimulator.generateSimulatedImages(2,0,0)
        
        # Converti l'immagine in un tipo supportato se necessario
        # Se l'immagine è a colori (3 canali), assicura l'encoding corretto
        if sim_img.ndim == 3 and sim_img.shape[2] == 3:
            if sim_img.dtype != np.uint8:
                sim_img = cv2.normalize(sim_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            image_msg = self.bridge.cv2_to_imgmsg(sim_img, encoding="bgr8")
        else:
            # Converti l'immagine in un tipo supportato se necessario
            if sim_img.dtype != np.uint8:
                sim_img = cv2.normalize(sim_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            image_msg = self.bridge.cv2_to_imgmsg(sim_img, encoding="mono8")

        # Pubblica il messaggio sul topic
        self.image_publisher.publish(image_msg)

    def spin(self):
        rospy.spin()
    
   

if __name__ == '__main__':
    simulator = SimulatorNode()
    simulator.spin()
    
    
