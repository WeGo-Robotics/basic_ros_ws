#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from math import *
import os
import csv


# 클래스 생성
class NAME:
    # 초기화 및 초기 설정
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        self.imu_pub = rospy.Publisher("/imu_to_euler", Float32, queue_size=3)
        rospy.Subscriber("/imu/data", Imu, self.imu_CB)
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration.x
        self.yaw = 0.0

    # 함수 설정
    def imu_CB(self, msg):
        self.imu_msg = msg
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x, y, z, w)
        self.yaw = yaw_z * 180 / pi
        print(yaw_z)
        self.imu_pub.publish(self.yaw)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


def main():
    name = NAME()
    rospy.spin()


if __name__ == "__main__":
    main()
