#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import *


class Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 2 단계
        rospy.init_node("wego_sub_node")  # 1단계 : 노드의 이름
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)  # 2단계 : 노드의 역할
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2단계 : 노드의 역할
        laser_mag = LaserScan()
        self.ctrl_msg = Twist()
        self.laser_flag = False
        self.degrees = []

    def callback(self, msg):
        num = 0
        steer = 0
        obstacle_degree = []
        if self.laser_flag == False:
            self.degrees = [(msg.angle_min + msg.angle_increment * i) * 180 / pi for i, v in enumerate(msg.ranges)]

        self.laser_flag = True
        for index, value in enumerate(msg.ranges):
            if 0 < msg.ranges[index] < 0.30 and abs(self.degrees[index]) < 7.5:
                num += 1
            else:
                if 0 < msg.ranges[index] < 0.5:
                    obstacle_degree.append(index)
        if num > 10:
            print(f"STOP:{num}")
            speed = 0
        else:
            if len(obstacle_degree) != 0:
                print(obstacle_degree)
                right_blank = obstacle_degree[0]
                left_blank = len(self.degrees) - obstacle_degree[-1]
                print(f"right_blank:{right_blank}")
                print(f"left_blank:{left_blank}")
                if right_blank > left_blank:
                    steer = -10
                else:
                    steer = 10

            speed = 0.3

        self.ctrl_msg.linear.x = speed
        self.ctrl_msg.angular.z = steer
        self.pub.publish(self.ctrl_msg)


def main():  # 4 단계
    name = Name()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
