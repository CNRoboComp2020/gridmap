#!/usr/bin/python
#-*- encoding: utf8 -*-

#由于用iris_fpv_cam roslaunch时实在太卡，且实际图像高、宽似乎有bug，因此决定写一个假的CameraInfo的publisher，用来在不使用iris_fpv_cam时仍能收到相机内参等信息
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo


class FakeDetectorNode:

    def __init__(self):
        rospy.init_node('fake_camerainfo_publisher_node', anonymous=True)

        self.gridPub_ = rospy.Publisher('/iris_0/usb_cam/camera_info', CameraInfo, queue_size=100)

        self.publishloop_timer_ = rospy.Timer(rospy.Duration(0.5), self.publishloopCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    def publishloopCallback(self, event):
        # fpv_cam的参数
        msg = CameraInfo()
        msg.K = [277.191356, 0.0, 320.5, 0.0, 277.191356, 240.5, 0.0, 0.0, 1.0]
        msg.height = 480
        msg.width = 640
        self.gridPub_.publish(msg)


if __name__ == '__main__':
    fd = FakeDetectorNode()







    
