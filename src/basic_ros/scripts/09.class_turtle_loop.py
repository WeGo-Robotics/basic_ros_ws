#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_node")  # ROS 1단계 : 노드 이름
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        # self.rate = rospy.Rate(10)  # ROS 3단계 : 퍼블리셔 - 주기 설정
        self.msg = Twist()

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        self.msg.linear.x = 1
        if msg.x > 8:
            self.msg.angular.z = 3
        self.pub.publish(self.msg)  # ROS 4단계 : 퍼블리셔 - 퍼블리시 실행
        # self.rate.sleep()  # ROS 5단계 : 퍼블리셔 - 주기 실행
        print(msg)


def main():  # 클래스 4단계 : 메인 함수
    name = Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
