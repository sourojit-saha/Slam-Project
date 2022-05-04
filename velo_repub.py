#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
import numpy as np

def subscriberCallBack(msg):
    pub_msg = msg
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.frame_id = 'map'
    stream_pub.publish(pub_msg)
    
def listener():

    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("/cmu_rc3/velodyne_cloud_registered_imu", pc2, subscriberCallBack) #change topic name as required
    rospy.spin()


if __name__ == '__main__':
    stream_pub = rospy.Publisher("/cmu_rc3/velodyne_cloud_registered_map", pc2, queue_size=5)
    listener()