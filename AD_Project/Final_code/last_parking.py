#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from ultrasonic import ultrasonic
from bar import BarDetector


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
	    self.ultra = ultrasonic()
        self.driver = MotorDriver('/xycar_motor_msg')
        self.cnt = 0
        self.ob = False
        self.direct = ""

    def stopsign(self):
        obstacle = self.ultra.obstruction_detect()
        print(obstacle)
        if obstacle == True:
            return 0

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        angle = 0

        if obs_m < 40 and not self.obn and self.line_detector.check():
            self.ob = True
            self.cnt = 1
            speed = 0
        speed = self.accelerate(obs_l, obs_m, obs_r)
        
        if self.cnt == 1 and obs_m > 50:
            self.direct = self.line_detector.recognition()
            print(self.direct)
            speed = 0
        
        if self.direct == "right":
            angle = self.turn_right()
        elif self.direct == "left":
            angle = self.turn_left()
        
        #on = self.stopsign()
        #if on == 0:
            #speed = 0
        self.driver.drive(angle + 90, speed + 90)

    def turn_right(self):
        if self.cnt < 20:
            self.cnt += 1
            return 30
        elif self.cnt < 35:
            self.cnt += 1
            return -20
        else:
            return 0

    def turn_left(self):
        if self.cnt < 20:
            self.cnt += 1
            return -30
        elif self.cnt < 35:
            self.cnt += 1
            return 30
        else:
            return 0

    def accelerate(self, left, mid, right):
        speed = 15
        if mid < 30:
            speed = 0
        return speed
    

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)