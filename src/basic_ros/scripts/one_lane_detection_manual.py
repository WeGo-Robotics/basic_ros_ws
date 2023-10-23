#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from math import *
from time import *


class ImageProcessor:
    def __init__(self):
        # CvBridge 인스턴스를 초기화하여 ROS 이미지를 OpenCV 이미지로 변환합니다.
        self.bridge = CvBridge()

        # ROS 노드를 초기화하고 노드 이름을 'image_processor_node'로 설정합니다.
        rospy.init_node("image_processor_node", anonymous=True)
        # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 Subscriber하고,
        # 새 메시지가 수신되면 self.image_callback 메서드를 호출합니다.
        rospy.Subscriber("/camera/rgb/image_raw/image_raw/compressed", CompressedImage, self.cam_CB)
        rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # 'cmd_vel' 토픽에 Twist 메시지를 발행할 퍼블리셔를 초기화합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.original_window = "original_image"
        self.cropped_window = "croped_image"
        self.control_window = "control"
        # Create the trackbars for the HSV ranges
        self.create_trackbar_flag = False

        self.cam_flag = False  # 이미지 메세지 수신 확인을 위한 변수 설정
        self.window_num = 6
        self.margin = 60
        self.previous_point = 0
        self.img_msg = CompressedImage()
        self.cmd_msg = Twist()

    def create_trackbar(self):
        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.cropped_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.control_window)
        self.hsv_track(True)
        self.control_track(True)
        cv2.createTrackbar("Low_H", self.cropped_window, 0, 179, self.hsv_track)
        cv2.createTrackbar("Low_S", self.cropped_window, 0, 255, self.hsv_track)
        cv2.createTrackbar("Low_V", self.cropped_window, 0, 255, self.hsv_track)
        cv2.createTrackbar("Up_H", self.cropped_window, 179, 179, self.hsv_track)
        cv2.createTrackbar("Up_S", self.cropped_window, 255, 255, self.hsv_track)
        cv2.createTrackbar("Up_V", self.cropped_window, 255, 255, self.hsv_track)
        cv2.createTrackbar("Left_or_Right", self.cropped_window, 0, 1, self.hsv_track)
        cv2.createTrackbar("Speed", self.control_window, 0, 100, self.control_track)
        cv2.createTrackbar("Lane_Distance", self.control_window, 50, 100, self.control_track)
        cv2.createTrackbar("Steering_GAIN", self.control_window, 100, 200, self.control_track)
        self.create_trackbar_flag = True

    def hsv_track(self, _):
        self.L_H_Value = cv2.getTrackbarPos("Low_H", self.cropped_window)
        self.L_S_Value = cv2.getTrackbarPos("Low_S", self.cropped_window)
        self.L_V_Value = cv2.getTrackbarPos("Low_V", self.cropped_window)
        self.U_H_Value = cv2.getTrackbarPos("Up_H", self.cropped_window)
        self.U_S_Value = cv2.getTrackbarPos("Up_S", self.cropped_window)
        self.U_V_Value = cv2.getTrackbarPos("Up_V", self.cropped_window)
        self.L_R_Value = cv2.getTrackbarPos("Left_or_Right", self.cropped_window)

    def control_track(self, _):
        self.Speed_Value = cv2.getTrackbarPos("Speed", self.control_window)
        self.Lane_Distance_Value = cv2.getTrackbarPos("Lane_Distance", self.control_window)
        self.Steering_GAIN_Value = cv2.getTrackbarPos("Steering_GAIN", self.control_window)

    def apply_mask(self, cv_img):
        # Convert the BGR image to HSV
        cvt_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # Define the BGR range using the trackbar values
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])

        # Create a mask using the HSV range
        mask = cv2.inRange(cvt_hsv, lower, upper)

        # Apply the mask to the original image to obtain the final result
        hsv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)
        return hsv_img

    def crop_img(self, hsv_img):
        if self.L_R_Value == 0:
            croped_img = hsv_img[hsv_img.shape[0] * 3 // 4 : hsv_img.shape[0], 0 : hsv_img.shape[1] // 2]
        else:
            croped_img = hsv_img[hsv_img.shape[0] * 3 // 4 : hsv_img.shape[0], hsv_img.shape[1] // 2 : hsv_img.shape[1]]
        croped_img_shape = croped_img.shape[0:2]
        return croped_img, croped_img_shape

    def binary(self, croped_img):
        bin = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)
        binary_img = np.zeros_like(bin)
        binary_img[bin != 0] = 1
        return binary_img

    def sliding_window(self, cv_img, croped_img, binary_img, cv_img_shape, croped_img_shape):
        crop_layer = croped_img_shape[0] // self.window_num  # 20
        threshold = crop_layer // 2
        histogram = []
        indices = []
        middle_point = []
        for i in range(0, self.window_num):  # i = 0~5
            original_window_top = cv_img_shape[0] - (i + 1) * crop_layer
            original_window_bottom = cv_img_shape[0] - (i) * crop_layer
            cropped_window_top = croped_img_shape[0] - (i + 1) * crop_layer
            cropped_window_bottom = croped_img_shape[0] - (i) * crop_layer
            histogram.append(np.sum(binary_img[cropped_window_top:cropped_window_bottom, :], axis=0))
            indices.append(np.where(histogram[i] > threshold)[0])
            try:
                middle_point.append((min(indices[i]) + max(indices[i])) // 2)
                cropped_window_left = middle_point[i] - self.margin
                cropped_window_right = middle_point[i] + self.margin
                left_window_left = cropped_window_left
                left_window_right = cropped_window_right
                right_window_left = cropped_window_left + cv_img_shape[1] // 2
                right_window_right = cropped_window_right + cv_img_shape[1] // 2
                cv2.rectangle(croped_img, [cropped_window_left, cropped_window_top], [cropped_window_right, cropped_window_bottom], [0, 0, 255], 2)
                cv2.circle(croped_img, [middle_point[i], (cropped_window_top + cropped_window_bottom) // 2], 3, [0, 255, 0], -1)
                if self.L_R_Value == 0:
                    cv2.rectangle(cv_img, [left_window_left, original_window_top], [left_window_right, original_window_bottom], [0, 0, 255], 2)
                    cv2.circle(cv_img, [middle_point[i], (original_window_top + original_window_bottom) // 2], 3, [0, 255, 0], -1)
                else:
                    cv2.rectangle(cv_img, [right_window_left, original_window_top], [right_window_right, original_window_bottom], [0, 0, 255], 2)
                    cv2.circle(cv_img, [middle_point[i] + cv_img_shape[1] // 2, (original_window_top + original_window_bottom) // 2], 3, [0, 255, 0], -1)
            except:
                print(f"line_not_detect")
        try:
            final_point = int(np.average(middle_point))
            self.previous_point = final_point
        except:
            final_point = self.previous_point
        return final_point

    def cal_angle_per_pixel(self, final_point, cv_img_shape):
        angle_per_pixel = pi / cv_img_shape[1]
        angle_resolution = angle_per_pixel * (final_point - 10)
        return angle_resolution

    def sense(self):
        img_msg = self.img_msg  # self.img_msg의 값을 img_msg 변수에 저장
        cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환
        if self.create_trackbar_flag == False:
            self.create_trackbar()
        return cv_img  # 이미지를 반환

    def think(self, cv_img):
        hsv_img = self.apply_mask(cv_img)
        cv_img_shape = cv_img.shape[0:2]
        croped_img, croped_img_shape = self.crop_img(hsv_img)
        binary_img = self.binary(croped_img)
        final_point = self.sliding_window(cv_img, croped_img, binary_img, cv_img_shape, croped_img_shape)
        angle_resolution = self.cal_angle_per_pixel(final_point, cv_img_shape)
        return croped_img, angle_resolution

    def act(self, angle_resolution):
        self.cmd_msg.linear.x = self.Speed_Value / 100
        steering_gain = self.Steering_GAIN_Value / 100
        lane_distance = (self.Lane_Distance_Value - 50) / 50
        self.cmd_msg.angular.z = (angle_resolution * steering_gain) + lane_distance
        self.pub.publish(self.cmd_msg)

    def cam_CB(self, msg):  # ROS 3단계(필수): 서브스크라이버 - 콜백 함수 설정
        if msg != -1:  # 메세지가 유효한 경우
            self.img_msg = msg  # 메세지를 self.img_msg로 저장
            self.cam_flag = True  # 메세지 수신 확인 변수를 True로 설정
        else:
            self.cam_flag = False  # 메세지 수신 확인 변수를 False로 설정

    def run(self):
        if self.cam_flag:
            cv_img = self.sense()
            croped_img, angle_resolution = self.think(cv_img)
            self.act(angle_resolution)
            cv2.imshow(self.original_window, cv_img)
            cv2.imshow(self.cropped_window, croped_img)
            cv2.waitKey(1)


def main():
    try:
        image_processor = ImageProcessor()
        while not rospy.is_shutdown():
            # ImageProcessor 인스턴스를 생성하고 run 메서드를 호출하여 프로그램을 실행합니다.
            image_processor.run()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
