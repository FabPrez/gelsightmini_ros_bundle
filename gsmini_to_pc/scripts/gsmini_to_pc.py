import sys
import numpy as np
import cv2
import os
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import open3d
import copy
import gelSight_SDK.examples.gs3drecon as gs3drecon
import gelSight_SDK.examples.gsdevice as gsdevice

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import PointCloud2, Image
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import rospkg

class Gsmini_to_pc:
    def __init__(self):
        rospy.init_node('gmini_to_pc', anonymous=True)
        
        image_topic = rospy.get_param('~image_topic', "/null_image") #input topic
        pointcloud_image_topic = rospy.get_param('~pointcloud_image_topic', "/gsmini_pcd") #output topic
        is_simulated = rospy.get_param('~is_simulated')
        
        if is_simulated:
            rospy.loginfo("Ready to convert SIMULATED images to pointclud!")
        else:
            rospy.loginfo("Ready to convert REAL images to pointclud!")
        
        rospy.loginfo("Subscribing to: " + image_topic)
        rospy.loginfo("Publishing to: " + pointcloud_image_topic)
        
        # Set flags
        self.SAVE_VIDEO_FLAG = False
        self.GPU = True
        self.MASK_MARKERS_FLAG = False
        self.SHOW_3D_NOW = True
        self.IS_SIMULATED = is_simulated
        self.background_settled = False
        
        # Set the camera resolution
        self.mmpp = 0.0634  # mini gel 18x24mm at 240x320

        # This is meters per pixel that is used for ros visualization
        self.mpp = self.mmpp / 1000.
        
        # the device ID can change after changing the usb ports.
        # on linux run, v4l2-ctl --list-devices, in the terminal to get the device ID for camera
        self.dev = gsdevice.Camera("GelSight Mini")
        net_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'nnmini.pt')) 

        self.bridge = CvBridge()
        self.dev.connect()

        # ''' Load neural network '''
        model_file_path = '.'
        net_path = os.path.join(model_file_path, net_file_path)
        print('net path = ', net_path)

        if self.GPU:
            gpuorcpu = "cuda"
        else:
            gpuorcpu = "cpu"

        self.nn = gs3drecon.Reconstruction3D(self.dev)
        self.net = self.nn.load_nn(net_path, gpuorcpu)
            
        ''' ros point cloud initialization '''
        x = np.arange(self.dev.imgh) * self.mpp
        y = np.arange(self.dev.imgw) * self.mpp
        X, Y = np.meshgrid(x, y)
        self.points = np.zeros([self.dev.imgw * self.dev.imgh, 3])
        self.points[:, 0] = np.ndarray.flatten(X)
        self.points[:, 1] = np.ndarray.flatten(Y)
        Z = np.zeros((self.dev.imgh, self.dev.imgw))  # initialize self array with zero depth values
        self.points[:, 2] = np.ndarray.flatten(Z)
        self.gelpcd = open3d.geometry.PointCloud()
        self.gelpcd.points = open3d.utility.Vector3dVector(self.points)
        self.gelpcd_pub = rospy.Publisher(pointcloud_image_topic, PointCloud2, queue_size=10)
            
        ''' use this to plot just the 3d '''
        self.current_dm = None
        if self.SHOW_3D_NOW:
            self.vis3d = gs3drecon.Visualize3D(self.dev.imgh, self.dev.imgw, '', self.mmpp)
        
        if self.IS_SIMULATED:
            self.sub_image = rospy.Subscriber(image_topic, Image, self.image_to_pc_callback)
            rospack = rospkg.RosPack()
            
            data_folder_path = rospack.get_path('data_folder')
            background_path = os.path.join(data_folder_path,'simulated_backgroud','background.png')  
            rospy.loginfo("****background_path: " + background_path)
            
            self.simulated_background = cv2.imread(background_path)
        else:
            self.f0 = self.dev.get_raw_image()
        print('press q on image to exit')
        
        self.counter = 0

    def image_to_pc_callback(self, msg):
        self.f1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_pointcloud()

    def process_pointcloud(self):
        if not self.IS_SIMULATED:
            self.f1 = self.dev.get_image()
        else:
            self.counter +=1
            if self.counter <100:
                self.f1 = self.simulated_background
                self.counter +=1  
            else:
                self.background_settled = True
                
        bigframe = cv2.resize(self.f1, (self.f1.shape[1] * 2, self.f1.shape[0] * 2))
        # compute the depth map
        dm = self.nn.get_depthmap(self.f1, self.MASK_MARKERS_FLAG)

        dm_ros = copy.deepcopy(dm) * self.mpp
        ''' publish point clouds '''
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'gs_mini'
        self.points[:, 2] = np.ndarray.flatten(dm_ros)
        self.gelpcd.points = open3d.utility.Vector3dVector(self.points)
        gelpcdros = pcl2.create_cloud_xyz32(header, np.asarray(self.gelpcd.points))
        self.gelpcd_pub.publish(gelpcdros)
        
        ''' Store depth map for visualization '''
        self.current_dm = dm

    def run_visualization(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if self.current_dm is not None:
                self.vis3d.update(self.current_dm)
            rate.sleep()
        
    def spinner(self):
        rate = rospy.Rate(60)
        if self.IS_SIMULATED:
            while not self.background_settled:
                self.process_pointcloud()
                rate.sleep()
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                self.process_pointcloud()
                rate.sleep()

if __name__ == "__main__":
    gsmini_to_pc = Gsmini_to_pc()
    
    # Create a separate thread for ROS spinner
    ros_thread = threading.Thread(target=gsmini_to_pc.spinner)
    ros_thread.start()
    
    # Run visualization in the main thread
    gsmini_to_pc.run_visualization()
