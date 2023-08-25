#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(msg)


def main():  # 클래스 4단계 : 메인 함수
    name = Name()
    while not rospy.is_shutdown():
        name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
