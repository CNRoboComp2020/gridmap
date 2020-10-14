#!/usr/bin/python
#-*- encoding: utf8 -*-

#接收来自激光雷达的信息并绘制栅格图

from Tools import getPackagePath
from scipy.spatial.transform import Rotation as R
import scipy.misc
import rospy
import numpy as np
import math
import sys
import os
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class MapDrawerNode:
    
    def __init__(self):
        rospy.init_node('map_drawer_node', anonymous=True)

        #地图信息
        self.map_range_ = np.array([[-50., -50.], [130., 50.]])
        self.z_range_ = (2., 3.)
        self.res_ = 0.1  # resolution
        self.map_size_ = ((self.map_range_[1]-self.map_range_[0])/self.res_).astype('int')
        self.map_1_ = np.zeros(self.map_size_.tolist(), dtype=np.int32)

        #传感器信息
        self.range_min_ = None
        self.range_max_ = None
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype = 'float')
        self.is_pose_init_ = False
        self.is_lidar_init_ = False

        self.gazeboposeSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazeboposeCallback)  #200Hz
        self.laserscanSub_ = rospy.Subscriber('/hokuyo_lidar/scan', LaserScan, self.laserscanCallback)  #50Hz, if using hokuyo_lidar_no_gravity

        self.gridPub_ = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=100)

        self.publishloop_timer_ = rospy.Timer(rospy.Duration(5), self.publishloopCallback)
        
        self.package_path_ = getPackagePath('gridmap')
        self.counter_ = 0

        #尝试从文件中读取数据
        if os.path.exists(self.package_path_+'/map/map_1.npy'):
            os.system('cp '+self.package_path_+'/map/map_1.npy '+self.package_path_+'/map/map_1_backup.npy')
            self.map_1_ = np.load(self.package_path_+'/map/map_1.npy').astype(np.int32).reshape(self.map_size_.tolist())
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    def mapIndex(self, coor):
        result = [True, 0, 0]
        result[1] = int(math.floor((coor[0]-self.map_range_[0][0])/self.res_))
        result[2] = int(math.floor((coor[1]-self.map_range_[0][1])/self.res_))
        if result[1] < 0 or result[1] >= self.map_size_[0]:
            result[0] = False
        if result[2] < 0 or result[2] >= self.map_size_[1]:
            result[0] = False
        if coor[2] < self.z_range_[0] or coor[2] > self.z_range_[1]:
            result[0] = False
        return result


    def gazeboposeCallback(self, msg):
        if 'hokuyo_lidar' in msg.name:
            i = msg.name.index('hokuyo_lidar')
            pose = msg.pose[i]
            self.t_wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            self.is_pose_init_ = True


    #由于是仿真状态，因此可以认为不会出现什么噪点
    def laserscanCallback(self, msg):
        if not self.is_pose_init_:
            return
        p_wl = self.t_wu_.copy()
        R_wl = self.R_wu_.as_dcm()

        if not self.is_lidar_init_:
            self.range_min_ = msg.range_min
            self.range_max_ = msg.range_max
            self.is_lidar_init_ = True
        rs = len(msg.ranges)  #ranges_size

        last_id = [-1, -1]
        for i in range(rs):
            theta = msg.angle_min + i*msg.angle_increment
            rad = msg.ranges[i]
            isinf = math.isinf(rad)
            if isinf:
                continue
            if rad < self.range_min_:
                continue

            p_lp = np.array([rad*math.cos(theta), rad*math.sin(theta), 0])
            p_wp = np.matmul(R_wl, p_lp)+p_wl
            mapIndexResult_p_wp = self.mapIndex(p_wp)
            if mapIndexResult_p_wp[0] and (last_id[0]!=mapIndexResult_p_wp[1] or last_id[1]!=mapIndexResult_p_wp[2]):
                self.map_1_[mapIndexResult_p_wp[1]][mapIndexResult_p_wp[2]] += 1
                last_id[0] = mapIndexResult_p_wp[1]
                last_id[1] = mapIndexResult_p_wp[2]

        pass


    def publishloopCallback(self, event):
        self.map_1_[self.map_1_<=40] = 0
        self.map_1_[self.map_1_>40] = 100
        map_result = self.map_1_.copy()
        
        scipy.misc.toimage(map_result, cmin=0, cmax=100).save('outfile'+str(self.counter_)+'.jpg')
        print('map_1.jpg saved.')
        self.counter_ += 1
        
        np.save(self.package_path_+'/map/map_1.npy', self.map_1_.astype(np.uint8))

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
        map_result_list = map_result.T.flatten().tolist()
        for ele in map_result_list:
            ele = int(ele)
        grid_msg.data = map_result_list
        self.gridPub_.publish(grid_msg)
        
        pass


if __name__ == '__main__':
    da = MapDrawerNode()

