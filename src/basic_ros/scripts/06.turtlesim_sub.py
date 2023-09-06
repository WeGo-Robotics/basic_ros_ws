#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름


def callback(msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
    print(msg)  # 메세지 읽기
    print(msg.x)  # 메세지 항목(x) - 데이터 값 읽기
    print(msg.y)  # 메세지 항목(y) - 데이터 값 읽기


sub = rospy.Subscriber("/turtle1/pose", Pose, callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
rospy.spin()
