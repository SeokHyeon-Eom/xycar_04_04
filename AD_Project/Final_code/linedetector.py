# !/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.std_color = (144, 137, 132)
        self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])
        self.num_pixel = 9 * 259
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def recognition(self):
        gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        edge = cv2.Canny(blur, 60, 150)
        edge = edge[220: 370, :]
        height = edge.shape[0] - 1
        weight = edge.shape[1] - 1

        src = np.float32([[220, 0], [420, 0], [90, 100], [560, 100]])
        dst = np.float32([[0, 0], [640, 0], [0, 150], [640, 150]])

        M = cv2.getPerspectiveTransform(src, dst)
        warp = cv2.warpPerspective(edge.copy(), M, (640, 150))
        # cv2.imshow("asd", warp)
        # cv2.imshow('zxc', edge)
        #cv2.imshow('origin', self.cam_img)
        # cv2.waitKey(1)

        left = warp[:, : 320]
        right = warp[:, 320:]
        if cv2.countNonZero(left) < cv2.countNonZero(right):
            return "left"
        else:
            return "right"
    
    def check(self):
        blue = 0
        green = 0
        red = 0
        for i in range(300, 310):
            for j in range(210, 470):
                blue += self.cam_img[i][j][0]
                green += self.cam_img[i][j][1]
                red += self.cam_img[i][j][2]

        blue_avg = blue / self.num_pixel
        green_avg = green / self.num_pixel
        red_avg = red / self.num_pixel
        now = (blue_avg, green_avg, red_avg)
        print(now)
        if (self.std_color[0] - 40 < blue_avg < self.std_color[0] + 40) and (
                            self.std_color[1] - 40 < green_avg < self.std_color[1] + 40) and (
                            self.std_color[2] - 40 < red_avg < self.std_color[2] + 40):
            return True
        else:
            now = (0,0,0)
        return False
