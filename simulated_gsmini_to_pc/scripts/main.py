#!/home/fabioprez/gsmini_ws/src/gelsightmini_ros_bundle/simulated_gsmini_to_pc/gsmini_venv/bin/python

import rospkg
import rospy
from sensor_msgs.msg import PointCloud2

def main():
    rospy.init_node('my_node', anonymous=True)
    pub = rospy.Publisher('/my_pointcloud_topic', PointCloud2, queue_size=10)
    
    # Esegui altre operazioni necessarie qui
    
    rospy.spin()

if __name__ == '__main__':
    main()
