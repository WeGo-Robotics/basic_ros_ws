#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node")  # ROS 1단계 : 노드 이름
pub = rospy.Publisher("/counter", Int32, queue_size=1)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
rate = rospy.Rate(10)  # ROS 3단계 : 퍼블리셔 - 주기 설정
msg = 0
while not rospy.is_shutdown():
    pub.publish(msg)  # ROS 4단계 : 퍼블리셔 - 퍼블리시 실행
    msg += 1
    rate.sleep()  # ROS 5단계 : 퍼블리셔 - 주기 실행
