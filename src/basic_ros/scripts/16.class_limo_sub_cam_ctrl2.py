#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import *
import cv2


class Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계 : 노드 이름
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        self.pub = rospy.Publisher("/camera/gray/compressed", CompressedImage, queue_size=10)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
        self.cvbridge = CvBridge()
        # self.rate = rospy.Rate(10)  # ROS 3단계 : 퍼블리셔 - 주기 설정
        self.cam_flag = False

    def cam_CB(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        if msg != -1:
            self.msg = msg
            self.cam_flag = True
        else:
            self.cam_flag = False

    def think(self, img):
        img = self.cvbridge.compressed_imgmsg_to_cv2(self.msg)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img, gray_img

    def act(self, gray_img):
        gray_img_mag = self.cvbridge.cv2_to_compressed_imgmsg(gray_img)
        self.pub.publish(gray_img_mag)  # ROS 4단계 : 퍼블리셔 - 퍼블리시 실행
        # self.rate.sleep()  # ROS 5단계 : 퍼블리셔 - 주기 실행

    def show(self, img, gray_img):
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.namedWindow("gray_img", cv2.WINDOW_NORMAL)
        cv2.imshow("gray_img", gray_img)
        cv2.waitKey(1)

    def run(self):
        if self.cam_flag == True:
            img, gray_img = self.think(self.msg)
            self.act(gray_img)
            self.show(img, gray_img)


def main():  # 클래스 4단계 : 메인 함수
    name = Name()
    while not rospy.is_shutdown():
        name.run()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
