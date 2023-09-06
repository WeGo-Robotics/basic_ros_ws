#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from math import *


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(msg)  # 메세지 읽기
        print(f"orientation:{msg.orientation}")  # 메세지 항목(orientation) - 데이터 값 읽기
        print(f"linear_acceleration:{msg.linear_acceleration}")  # 메세지 항목(linear_acceleration) - 데이터 값 읽기
        print(f"angular_velocity:{msg.angular_velocity}")  # 메세지 항목(angular_velocity) - 데이터 값 읽기

        roll_x_rad, pitch_y_rad, yaw_z_rad = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.w, msg.orientation.z)
        roll_x_degree = self.angle_to_degree(roll_x_rad)
        pitch_y_degree = self.angle_to_degree(pitch_y_rad)
        yaw_z_degree = self.angle_to_degree(yaw_z_rad)

        print(f"roll_x_degree:{roll_x_degree}")
        print(f"pitch_y_degree:{pitch_y_degree}")
        print(f"yaw_z_degree:{yaw_z_degree}")

    def quaternion_to_euler(x, y, w, z):
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

    def angle_to_degree(angle):
        degree = angle * 180 / pi
        return degree


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
