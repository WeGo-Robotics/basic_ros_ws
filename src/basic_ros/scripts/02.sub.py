#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름


def callback(msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
    print(f"msg:{msg}")  # 메세지 읽기
    print(f"msg.data:{msg.data}")  # 메세지 항목(데이터) - 데이터 값 읽기


sub = rospy.Subscriber("/counter", Int32, callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
rospy.spin()
