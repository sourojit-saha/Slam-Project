#!/usr/bin/env python
from glob import glob
from re import X
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np

global_xyz = np.zeros((0,3))
count = 0
iter = 0

def subscriberCallBack(msg):
    pub_msg = msg
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.frame_id = 'map'
    stream_pub.publish(pub_msg)
    global count
    global global_xyz
    global iter
    iter = iter + 1
    count = count + 1
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    print(np.shape(xyz_array))
    global_xyz = np.vstack((global_xyz, xyz_array))
    # print('count: ',count)
    if count == 5:
        count = 0
        ax = plt.axes()
        ax.set(facecolor = "black")
        row_idx = np.where(np.logical_and(global_xyz[:,2]>0.5, global_xyz[:,2]<1)) # play aroud with limits to include/exclude the floor/roof
        plt.scatter(global_xyz[row_idx,0], global_xyz[row_idx,1], s=0.001, c='red')
        # name = '/home/p43s/test_ws/src/kmz_test/scripts/' + 'recent_map' + '.png'   # path and name where image is stored. should be same as that in talker.py
        name = '/home/p43s/test_ws/src/kmz_test/scripts/' + 'nb-slam_run_2-' +str(iter) + '.png' 
        # plt.savefig(name, dpi = 300, bbox_inches='tight', pad_inches=0, facecolor='black')
        global_xyz = np.zeros((0,3))
    
def listener():

    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("/cmu_rc3/velodyne_cloud_registered_imu", pc2, subscriberCallBack) #change topic name as required
    rospy.spin()


if __name__ == '__main__':
    stream_pub = rospy.Publisher("/cmu_rc3/velodyne_cloud_registered_map", pc2, queue_size=5)
    listener()