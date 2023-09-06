#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름
        self.pub = rospy.Publisher("/scan2", LaserScan, queue_size=5)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
        rospy.Subscriber("/scan", LaserScan, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        self.rate = rospy.Rate(10)  # ROS 3단계 : 퍼블리셔 - 주기 설정
        self.lidar_msg = LaserScan()  # 메세지 타입 설정 및 초기화

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(msg)  # 메세지 읽기
        print(msg.angle_min)  # 메세지 항목(angle_min) - 데이터 값 읽기
        print(msg.angle_max)  # 메세지 항목(angle_max) - 데이터 값 읽기
        print(msg.angle_increment)  # 메세지 항목(angle_increment) - 데이터 값 읽기
        print(msg.range_min)  # 메세지 항목(range_min) - 데이터 값 읽기
        print(msg.ranges_max)  # 메세지 항목(ranges_max) - 데이터 값 읽기
        print(msg.ranges)  # 메세지 항목(ranges) - 데이터 값 읽기
        self.lidar_msg.header = msg.header  # 메세지 항목(header) - 데이터 값 변경
        self.lidar_msg.ranges = msg.ranges + 1  # 메세지 항목(ranges) - 데이터 값 변경
        self.pub.publish(self.lidar_msg)  # ROS 4단계 : 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # ROS 5단계 : 퍼블리셔 - 주기 실행


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
