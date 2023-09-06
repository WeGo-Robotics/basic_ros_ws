#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("wego_pub_node")  # ROS 1단계 : 노드 이름

pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
rate = rospy.Rate(10)  # ROS 3단계 : 퍼블리셔 - 주기 설정
msg = Twist()  # 메세지 타입 설정 및 초기화

while not rospy.is_shutdown():
    pub.publish(msg)  # ROS 4단계 : 퍼블리셔 - 퍼블리시 실행
    rate.sleep()  # ROS 5단계 : 퍼블리셔 - 주기 실행
    msg.linear.x += 0.1  # 메세지 항목(linear.x) - 데이터 값 변경
