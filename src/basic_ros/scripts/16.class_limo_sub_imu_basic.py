#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(msg)  # 메세지 읽기
        print(f"orientation:{msg.orientation}")  # 메세지 항목(orientation) - 데이터 값 읽기
        print(f"linear_acceleration:{msg.linear_acceleration}")  # 메세지 항목(linear_acceleration) - 데이터 값 읽기
        print(f"angular_velocity:{msg.angular_velocity}")  # 메세지 항목(angular_velocity) - 데이터 값 읽기


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
