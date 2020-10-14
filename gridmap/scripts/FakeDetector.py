#!/usr/bin/python
#-*- encoding: utf8 -*-

from scipy.spatial.transform import Rotation as R
import rospy
import numpy as np
import math
from gazebo_msgs.msg import ModelStates, LinkStates
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import OccupancyGrid

#输入两个点的坐标和地图分辨率、原点位置，输出一个list，按顺序包含连接这两点线段经过的所有格子坐标
def getCellList(pt1, pt2, res, origin):
    (x1, y1) = (pt1[:2] - origin[:2]) / res
    (x2, y2) = (pt2[:2] - origin[:2]) / res
    cell_list = []
    if abs(x1-x2) < 1e-6 or math.floor(x1) == math.floor(x2):
        if y1 <= y2:
            for y in range(int(math.floor(y1)), int(math.floor(y2))+1):
                cell_list.append([int(math.floor(x1)), y])
        else:
            for y in range(int(math.floor(y2)), int(math.floor(y1))+1):
                cell_list.append([int(math.floor(x1)), y])
            cell_list = list(reversed(cell_list))
        pass
    elif abs(y1-y2) < 1e-6 or math.floor(y1) == math.floor(y2):
        if x1 <= x2:
            for x in range(int(math.floor(x1)), int(math.floor(x2))+1):
                cell_list.append([x, int(math.floor(y1))])
        else:
            for x in range(int(math.floor(x2)), int(math.floor(x1))+1):
                cell_list.append([x, int(math.floor(y1))])
            cell_list = list(reversed(cell_list))
        pass
    else:
        #思路：求解线段与格线的所有交点，进而得到线段经过的所有格子
        #y=kx+b, x=ly+c
        if math.floor(x1) > math.floor(x2):
            inv = True
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        else:
            inv = False
        #此后均按照x1<x2情况处理，cell_list从左往右
        intersect_list = []  #该list的元素为含有三个元素的list，分别代表交点的x、y坐标以及哪个坐标为整数（0表示x，1表示y ）
        k = (y2-y1) / (x2-x1)
        b = (x2*y1-x1*y2) / (x2-x1)
        for x in range(int(math.floor(x1))+1, int(math.floor(x2))+1):
            intersect_list.append([float(x), k*float(x)+b, 0])
        if k > 0:
            for y in range(int(math.floor(y1))+1, int(math.floor(y2))+1):
                intersect_list.append([(float(y)-b)/k, float(y), 1])
        else:
            for y in range(int(math.floor(y2))+1, int(math.floor(y1))+1):
                intersect_list.append([(float(y)-b)/k, float(y), 1])
        intersect_list.sort(cmp = lambda p, q: 1 if p[0]>q[0] else (-1 if p[0]<q[0] else 0))
        for pt in intersect_list:
            if pt[2]:  #交线平行于x轴
                if k > 0:
                    cell_list.append([int(math.floor(pt[0])), int(pt[1])-1])
                else:
                    cell_list.append([int(math.floor(pt[0])), int(pt[1])])
            else:  #交线平行于y轴
                cell_list.append([int(pt[0])-1, int(math.floor(pt[1]))])
        cell_list.append([int(math.floor(x2)), int(math.floor(y2))])
        if inv:
            cell_list = list(reversed(cell_list))
        pass
    return cell_list


class FakeDetectorNode:

    def __init__(self):
        rospy.init_node('fake_detector_node', anonymous=True)
        
        #地图信息和相机、目标的位姿
        self.grid_msg_ = None
        self.t_wc_ = np.zeros([3], dtype = 'float')
        self.R_wc_ = R.from_quat([0, 0, 0, 1])
        self.t_wt_ = np.zeros([3], dtype = 'float')
        self.R_wt_ = R.from_quat([0, 0, 0, 1])
        self.is_camera_init_ = False
        self.is_target_init_ = False

        #相机、目标model的名字
        self.camera_name_ = 'iris_0::fpv_cam_modified::link'
        self.target_name_ = 'actor'

        #相机参数
        self.fx_ = 0.0
        self.fy_ = 0.0
        self.cx_ = 0.0
        self.cy_ = 0.0
        self.h_ = 0
        self.w_ = 0
        self.dist_range_ = [0.0, 50.0]
        self.is_intrinsics_init_ = False

        self.gridSub_ = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.occupancygridCallback)
        self.modelposeSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelposeCallback)
        self.linkposeSub_ = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkposeCallback)
        self.intrinsicsSub_ = rospy.Subscriber('/iris_0/usb_cam/camera_info', CameraInfo, self.intrinsicsCallback)

        self.publishloop_timer_ = rospy.Timer(rospy.Duration(0.5), self.publishloopCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    #判断目标是否能被相机检测到
    #必须同时满足三个条件
    #1.目标与相机的距离适中
    #2.目标在相机的视野中
    #3.目标与相机间没有障碍物
    def isVisible(self):
        if self.grid_msg_ is None:
            return False
        if not self.is_camera_init_ or not self.is_target_init_:
            return False
        if not self.is_intrinsics_init_:
            return False
        print(0)
        
        #条件1
        dist = np.linalg.norm(self.t_wc_ - self.t_wt_)
        if dist < self.dist_range_[0] or dist > self.dist_range_[1]:
            return False
        print(1)

        #条件2
        t_ct = np.matmul(self.R_wc_.inv().as_dcm(), (self.t_wt_ - self.t_wc_))
        #针孔相机模型下的坐标系需要进行转换
        X = -t_ct[1]
        Y = -t_ct[2]
        Z = t_ct[0]
        u = self.fx_*(X/Z) + self.cx_
        v = self.fy_*(Y/Z) + self.cy_
        if Z < 0 or u < 0 or u > float(self.w_) or v < 0 or v > float(self.h_):
            return False
        print(2)

        #条件3
        camera_xy = self.t_wc_[:2]
        target_xy = self.t_wt_[:2]
        map_width = self.grid_msg_.info.width
        map_height = self.grid_msg_.info.height
        map_res = self.grid_msg_.info.resolution
        map_origin = np.array([self.grid_msg_.info.origin.position.x, self.grid_msg_.info.origin.position.y])
        cell_list = getCellList(camera_xy, target_xy, map_res, map_origin)
        print('(%f,%f)' % (map_width, map_height))
        for ind in cell_list:
            if ind[0] < 0 or ind[0] >= map_width or ind[1] < 0 or ind[1] >= map_height:
                break
            if self.grid_msg_.data[ind[1]*map_width+ind[0]] > 0:
                return False
        print(3)

        return True
    

    def occupancygridCallback(self, msg):
        self.grid_msg_ = msg


    def modelposeCallback(self, msg):
        if self.target_name_ in msg.name:
            i = msg.name.index(self.target_name_)
            pose = msg.pose[i]
            self.t_wt_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wt_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            self.is_target_init_ = True


    def linkposeCallback(self, msg):
        if self.camera_name_ in msg.name:
            i = msg.name.index(self.camera_name_)
            pose = msg.pose[i]
            self.t_wc_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wc_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            self.is_camera_init_ = True


    def intrinsicsCallback(self, msg):
        self.fx_ = msg.K[0]
        self.fy_ = msg.K[4]
        self.cx_ = msg.K[2]
        self.cy_ = msg.K[5]
        self.h_ = msg.height
        self.w_ = msg.width
        self.is_intrinsics_init_ = True


    def publishloopCallback(self, event):
        print(self.isVisible())


if __name__ == '__main__':
    fd = FakeDetectorNode()







    
