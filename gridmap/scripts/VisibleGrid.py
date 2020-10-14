#!/usr/bin/python
#-*- encoding: utf8 -*-

#目前该节点效率较低，只能跑到2Hz
from FakeDetector import getCellList
from scipy.spatial.transform import Rotation as R
import rospy
import numpy as np
import math
import time
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import OccupancyGrid

'''
#获得以原点为圆心、一定半径、一定幅角范围内（保证圆心角不超过180度）圆内的所有整点
#NOTE: ang_range中可以包含超过两个元素，但是所有元素必须能被包含在一个圆心角不超过180度的扇形内
def getPointInCircle(rad, ang_range):
    ang_max = max(ang_range)
    ang_min = min(ang_range)
    is_m1_in_range = None # -1是否被包含在幅角范围内
    if ang_max - ang_min <= math.pi:
        is_m1_in_range = False
    else:
        is_m1_in_range = True
        ang_range_new = [(i+2*math.pi if i<0 else i) for i in ang_range]
        ang_min = max(ang_range_new)-2*math.pi
        ang_max = min(ang_range_new)
    pt_list = []
    if rad < 0:
        return pt_list
    for x in range(-int(rad), int(rad)+1):
        y_max = math.sqrt(rad*rad - x*x)
        for y in range(-int(y_max), int(y_max)+1):
            ang = np.angle(x+y*1j) #算幅角很慢
            if (is_m1_in_range and (ang<ang_min or ang>ang_max)) or \
                    (not is_m1_in_range and ang>ang_min and ang<ang_max):
                pt_list.append([x, y])
            pt_list.append([x, y])
    return pt_list
'''


class VisibleGridNode:

    def __init__(self):
        rospy.init_node('visible_grid_node', anonymous=True)
        
        #地图信息和相机、目标的位姿
        self.grid_msg_ = None
        self.res_ = None
        self.origin_ = None
        self.map_width_ = None
        self.map_height_ = None
        self.t_wc_ = np.zeros([3], dtype = 'float')
        self.R_wc_ = R.from_quat([0, 0, 0, 1])
        self.is_grid_init_ = False
        self.is_camera_init_ = False

        #相机、目标model的名字
        self.camera_name_ = 'iris_0::base_link'

        #相机参数
        self.fx_ = 0.0
        self.fy_ = 0.0
        self.cx_ = 0.0
        self.cy_ = 0.0
        self.h_ = 0
        self.w_ = 0
        self.is_intrinsics_init_ = False

        self.max_dist_ = 30.0  #最远距离
        self.visible_grid_ = None

        self.gridPub_ = rospy.Publisher('/available_grid', OccupancyGrid, queue_size=100)

        self.gridSub_ = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.occupancygridCallback)
        self.linkposeSub_ = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkposeCallback)
        self.intrinsicsSub_ = rospy.Subscriber('/iris_0/usb_cam/camera_info', CameraInfo, self.intrinsicsCallback)

        self.publishloop_timer_ = rospy.Timer(rospy.Duration(10.0), self.publishloopCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    def gridUpdate(self):
        t1 = time.clock()
        if not self.is_camera_init_ or not self.is_intrinsics_init_ or not self.is_grid_init_:
            return
        pix_list = [np.array([1.0, (self.cx_ - 0.0) / self.fx_, (self.cy_ - 0.0) / self.fy_]), \
                    np.array([1.0, (self.cx_ - 0.0) / self.fx_, (self.cy_ - self.h_) / self.fy_]), \
                    np.array([1.0, (self.cx_ - self.w_) / self.fx_, (self.cy_ - 0.0) / self.fy_]), \
                    np.array([1.0, (self.cx_ - self.w_) / self.fx_, (self.cy_ - self.h_) / self.fy_])]   #相机图像四角的归一化坐标
        yaw_list = []
        for p_c in pix_list:
            p_w = np.matmul(self.R_wc_.as_dcm(), p_c)
            yaw_list.append(np.angle(p_w[0]+p_w[1]*1j))
        #获取扫描角度范围
        yaw_max = max(yaw_list)
        yaw_min = min(yaw_list)
        if yaw_max - yaw_min <=math.pi:
            pass
        else:
            yaw_list_new = [(yaw+2*math.pi if yaw<0 else yaw) for yaw in yaw_list]
            yaw_min = min(yaw_list_new)
            yaw_max = max(yaw_list_new)
        #开始扫描
        t2 = time.clock()
        self.visible_grid_ = np.zeros([self.map_width_, self.map_height_], dtype = np.uint8)
        yaw = yaw_min
        yaw_step = self.res_ / self.max_dist_
        R_wc_mat = self.R_wc_.inv().as_dcm()
        t_wc_vec = self.t_wc_.copy()
        while yaw <= yaw_max:
            endpoint = self.t_wc_[:2] + self.max_dist_ * np.array([math.cos(yaw), math.sin(yaw)])
            cell_list = getCellList(self.t_wc_[:2], endpoint, self.res_, self.origin_)
            for cell in cell_list:
                if cell[0] < 0 or cell[0] >= self.map_width_ or cell[1] < 0 or cell[1] >= self.map_height_:
                    break
                if self.grid_msg_.data[cell[1]*self.map_width_+cell[0]] > 0:
                    break
                #判断该点是否在相机视野内
                '''
                t_wt = self.origin_ + (np.array(cell, dtype = 'float') + np.array([0.5, 0.5])) * self.res_
                t_wt = np.append(t_wt, 0)
                t_ct = np.matmul(self.R_wc_.inv().as_dcm(), (t_wt - self.t_wc_))
                X = -t_ct[1]
                Y = -t_ct[2]
                Z = t_ct[0]
                u = self.fx_*(X/Z) + self.cx_
                v = self.fy_*(Y/Z) + self.cy_
                '''
                
                t_wt_x = self.origin_[0] + (float(cell[0]) + 0.5) * self.res_
                t_wt_y = self.origin_[1] + (float(cell[1]) + 0.5) * self.res_
                t_ct = np.matmul(R_wc_mat, (np.array([t_wt_x, t_wt_y, 0.0]) - t_wc_vec))
                u = self.fx_*(-t_ct[1]/t_ct[0]) + self.cx_
                v = self.fy_*(-t_ct[2]/t_ct[0]) + self.cy_
                
                if t_ct[0] < 0 or u < 0 or u > float(self.w_) or v < 0 or v > float(self.h_):
                    continue
                
                self.visible_grid_[cell[0]][cell[1]] = 100
            yaw += yaw_step
        t3 = time.clock()
        print((t2-t1), (t3-t2))


    def occupancygridCallback(self, msg):
        if not self.is_grid_init_:
            self.grid_msg_ = msg
            self.res_ = msg.info.resolution
            self.origin_ = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            self.map_width_ = msg.info.width
            self.map_height_ = msg.info.height
            self.is_grid_init_ = True


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
        self.gridUpdate()
        if self.visible_grid_ is not None:
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = rospy.Time.now()
            grid_msg.info.map_load_time = rospy.Time.now()
            grid_msg.info.resolution = self.res_
            grid_msg.info.width = self.map_width_
            grid_msg.info.height = self.map_height_
            grid_msg.info.origin.position.x = self.origin_[0]
            grid_msg.info.origin.position.y = self.origin_[1]
            grid_msg.info.origin.position.z = 0
            grid_msg.info.origin.orientation.x = 0
            grid_msg.info.origin.orientation.y = 0
            grid_msg.info.origin.orientation.z = 0
            grid_msg.info.origin.orientation.w = 1
            map_result_list = self.visible_grid_.T.flatten().tolist()
            for ele in map_result_list:
                ele = int(ele)
            grid_msg.data = map_result_list
            self.gridPub_.publish(grid_msg)


if __name__ == '__main__':
    vg = VisibleGridNode()


