#!/usr/bin/python
#-*- encoding: utf8 -*-

#读取.npy中已知大小、分辨率的二维数组，并以/nav_msgs/OccupancyGrid格式发布

from Tools import getPackagePath
import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid


class GridPublisherNode:
    
    def __init__(self, grid_array, res = 0.1, map_range = np.array([[0.0, 0.0], [0.0, 0.0]])):
        rospy.init_node('grid_publisher_node', anonymous=True)

        #地图信息
        map_1 = grid_array.copy()
        map_1[map_1<=0] = 0
        map_1[map_1>0] = 100
        self.map_result_list_ = map_1.T.flatten().tolist()
        for ele in self.map_result_list_:
            ele = int(ele)

        self.map_range_ = map_range
        self.res_ = res  # resolution
        self.map_size_ = ((self.map_range_[1]-self.map_range_[0])/self.res_).astype('int')

        self.gridPub_ = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=100)

        self.publishloop_timer_ = rospy.Timer(rospy.Duration(1), self.publishloopCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    def publishloopCallback(self, event):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.info.map_load_time = rospy.Time.now()
        grid_msg.info.resolution = self.res_
        grid_msg.info.width = self.map_size_[0]
        grid_msg.info.height = self.map_size_[1]
        grid_msg.info.origin.position.x = self.map_range_[0][0]
        grid_msg.info.origin.position.y = self.map_range_[0][1]
        grid_msg.info.origin.position.z = 0
        grid_msg.info.origin.orientation.x = 0
        grid_msg.info.origin.orientation.y = 0
        grid_msg.info.origin.orientation.z = 0
        grid_msg.info.origin.orientation.w = 1
        grid_msg.data = self.map_result_list_
        self.gridPub_.publish(grid_msg)


if __name__ == '__main__':
    filename = getPackagePath('gridmap')+'/map/map_processed_0.1.npy'
    map_1 = np.load(filename)
    res = 0.1
    map_range = np.array([[-50., -50.], [130., 50.]])
    da = GridPublisherNode(map_1, res, map_range)

