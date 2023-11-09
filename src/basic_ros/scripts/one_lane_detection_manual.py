#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 위의 두 줄은 스크립트를 실행할 Python 버전과 문자 인코딩을 지정하는 표준 주석입니다.

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from math import *
from time import *

# 필요한 Python 모듈과 ROS 라이브러리를 임포트합니다.


class ImageProcessor:
    def __init__(self):
        # ImageProcessor 클래스의 초기화 메서드입니다.

        self.bridge = CvBridge()  # ROS 이미지와 OpenCV 이미지 간 변환을 위한 CvBridge 인스턴스를 생성합니다.

        rospy.init_node("image_processor_node", anonymous=True)  # ROS 노드를 초기화하고 이름을 'image_processor_node'로 설정합니다.

        rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", CompressedImage, self.cam_CB
        )  # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 구독하고 메시지가 수신되면 self.cam_CB 메서드를 호출합니다.
        rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 'cmd_vel' 토픽에 Twist 메시지를 발행할 퍼블리셔를 초기화합니다.

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # cmd_vel 토픽에 Twist 메시지를 발행할 퍼블리셔를 설정합니다.

        self.original_window = "original_image"
        self.cropped_window = "croped_image"
        self.control_window = "control"

        # HSV 범위를 조정하는 트랙바를 생성하는 데 필요한 변수와 설정을 초기화합니다.
        self.create_trackbar_flag = False  # 트랙바를 생성했는지 여부를 나타내는 플래그를 초기화합니다.
        self.cam_flag = False  # 이미지 메시지 수신 여부를 확인하는 변수를 초기화합니다.
        self.window_num = 6  # 이미지를 몇 개의 수평 레이어로 나눌 것인지를 설정합니다.
        self.margin = 60  # 슬라이딩 윈도우 주변의 여유 영역 크기를 설정합니다.
        self.previous_point = 0  # 이전 프레임에서 계산된 차선 위치를 저장하는 변수를 초기화합니다.
        self.img_msg = CompressedImage()  # 이미지 메시지를 저장할 변수를 초기화합니다.
        self.cmd_msg = Twist()  # 조향 및 속도 명령을 저장하는 Twist 메시지를 초기화합니다.

        # HSV 범위를 조정하는 트랙바의 초기값을 설정하는 변수들을 초기화합니다.
        self.L_H_Value = 0  # HSV 색상(Hue)의 하한(Hue, Saturation, Value)을 초기화합니다.
        self.L_S_Value = 0  # HSV 채도(Saturation)의 하한을 초기화합니다.
        self.L_V_Value = 0  # HSV 밝기(Value)의 하한을 초기화합니다.
        self.U_H_Value = 0  # HSV 색상의 상한을 초기화합니다.
        self.U_S_Value = 0  # HSV 채도의 상한을 초기화합니다.
        self.U_V_Value = 0  # HSV 밝기의 상한을 초기화합니다.
        self.L_R_Value = 0  # 좌측 또는 우측 뷰 선택을 나타내는 값을 초기화합니다.
        self.Stop_or_Go = 0  # 정지 또는 주행 명령을 나타내는 값을 초기화합니다.
        self.Speed_Value = 0  # 주행 속도를 나타내는 값을 초기화합니다.
        self.Steering_Standard = 0  # 스티어링 표준 위치를 나타내는 값을 초기화합니다.

    def create_trackbar_init(self, cv_img):
        # HSV 범위를 조절하기 위한 트랙바를 생성하는 메서드입니다.
        # 이 메서드는 트랙바를 사용하여 이미지 처리 매개변수를 조정할 수 있도록 초기 설정을 수행합니다.

        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'original_image'로 설정합니다.
        cv2.namedWindow(self.cropped_window, cv2.WINDOW_NORMAL)  # OpenCV 창을 생성하고 창 이름을 'croped_image'로 설정합니다.
        cv2.namedWindow(self.control_window)  # OpenCV 창을 생성하고 창 이름을 'control'로 설정합니다.

        def hsv_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. HSV 범위 트랙바의 현재 값을 업데이트합니다.
            self.L_H_Value = cv2.getTrackbarPos("Low_H", self.cropped_window)
            self.L_S_Value = cv2.getTrackbarPos("Low_S", self.cropped_window)
            self.L_V_Value = cv2.getTrackbarPos("Low_V", self.cropped_window)
            self.U_H_Value = cv2.getTrackbarPos("Up_H", self.cropped_window)
            self.U_S_Value = cv2.getTrackbarPos("Up_S", self.cropped_window)
            self.U_V_Value = cv2.getTrackbarPos("Up_V", self.cropped_window)
            self.L_R_Value = cv2.getTrackbarPos("Left/Right view", self.cropped_window)

            if self.L_R_Value != 1:
                # 왼쪽 뷰 선택 시 스티어링 표준 위치를 조절합니다.
                init_left_standard = self.init_standard // 2
                cv2.setTrackbarPos("Steering Standard", self.control_window, init_left_standard)
            else:
                # 오른쪽 뷰 선택 시 스티어링 표준 위치를 조절합니다.
                init_right_standard = self.init_standard + self.init_standard // 2
                cv2.setTrackbarPos("Steering Standard", self.control_window, init_right_standard)

        def control_track(value):
            # 트랙바 값이 변경될 때 호출되는 콜백 함수입니다. Stop/Go 및 Speed 트랙바 값 및 Steering_Standard를 업데이트합니다.
            self.Stop_or_Go = cv2.getTrackbarPos("Stop/Go", self.control_window)
            self.Speed_Value = cv2.getTrackbarPos("Speed", self.control_window)
            self.Steering_Standard = cv2.getTrackbarPos("Steering Standard", self.control_window)

        # 다양한 HSV 범위를 조정하기 위한 트랙바 생성
        cv2.createTrackbar("Low_H", self.cropped_window, 0, 179, hsv_track)  # H의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_S", self.cropped_window, 0, 255, hsv_track)  # S의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Low_V", self.cropped_window, 0, 255, hsv_track)  # V의 최소 임계 값 트랙바 생성
        cv2.createTrackbar("Up_H", self.cropped_window, 179, 179, hsv_track)  # H의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_S", self.cropped_window, 255, 255, hsv_track)  # S의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Up_V", self.cropped_window, 255, 255, hsv_track)  # V의 최대 임계 값 트랙바 생성
        cv2.createTrackbar("Left/Right view", self.cropped_window, 0, 1, hsv_track)  # 좌/우 뷰 선택 트랙바 생성
        cv2.createTrackbar("Stop/Go", self.control_window, 0, 1, control_track)  # 정지/주행 트랙바 생성
        cv2.createTrackbar("Speed", self.control_window, 0, 10, control_track)  # 속도 트랙바 생성
        cv2.createTrackbar("Steering Standard", self.control_window, self.init_standard, cv_img.shape[1] // 2, control_track)  # 스티어링 표준 위치 트랙바 생성
        self.create_trackbar_flag = True  # 트랙바가 생성되었음을 표시합니다.

    def apply_mask(self, cv_img):
        # BGR 이미지를 HSV 이미지로 변환합니다.
        cvt_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # HSV 범위를 트랙바의 값으로 정의합니다.
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])  # HSV의 하한 범위를 정의합니다.
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])  # HSV의 상한 범위를 정의합니다.

        # HSV 범위를 사용하여 이미지에 마스크를 생성합니다.
        mask = cv2.inRange(cvt_hsv, lower, upper)

        # 원본 이미지에 마스크를 적용하여 차선만 있는 부분을 남긴 최종 결과 이미지를 생성합니다.
        hsv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)
        return hsv_img  # 마스크를 적용한 이미지를 반환합니다.

    def crop_img(self, hsv_img):
        # 이미지를 자르는 메서드로, 차량 운행 중 관심 영역만을 추출합니다.

        if self.L_R_Value != 1:
            croped_img = hsv_img[hsv_img.shape[0] * 3 // 4 : hsv_img.shape[0], 0 : hsv_img.shape[1] // 2]  # 좌측에 있는 차선 영역을 추출합니다.
        else:
            croped_img = hsv_img[hsv_img.shape[0] * 3 // 4 : hsv_img.shape[0], hsv_img.shape[1] // 2 : hsv_img.shape[1]]  # 우측에 있는 차선 영역을 추출합니다.
        croped_img_shape = croped_img.shape[0:2]  # 추출된 이미지의 높이와 너비 정보를 저장합니다.
        return croped_img, croped_img_shape  # 추출된 이미지와 이미지 크기 정보를 반환합니다.

    def binary(self, croped_img):
        # 이진 이미지를 생성하는 메서드입니다. 이진 이미지는 차선을 강조하는 데 사용됩니다.

        bin = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)  # BGR 이미지를 그레이스케일로 변환합니다.
        binary_img = np.zeros_like(bin)  # 크기가 같은 빈 이진 이미지를 생성합니다.
        binary_img[bin != 0] = 1  # 이진 이미지에서 차선 픽셀을 1로 설정합니다.
        return binary_img  # 이진 이미지를 반환합니다.

    def sliding_window(self, cv_img, croped_img, binary_img, croped_img_shape):
        # 이미지에서 슬라이딩 윈도우 알고리즘을 사용하여 차선 검출을 수행하는 메서드입니다.

        crop_layer = croped_img_shape[0] // self.window_num  # 이미지 높이를 수평 레이어로 나눈 크기를 계산합니다.
        threshold = crop_layer // 2  # 차선으로 판단할 픽셀 수를 정의합니다.
        histogram = []  # 히스토그램을 저장할 리스트를 초기화합니다.
        indices = []  # 픽셀 인덱스를 저장할 리스트를 초기화합니다.
        middle_point = []  # 각 레이어의 중심 픽셀을 저장할 리스트를 초기화합니다.

        for i in range(0, self.window_num):  # 이미지를 여러 레이어로 나누어 검사합니다.
            original_window_top = cv_img.shape[0] - (i + 1) * crop_layer  # 원본 이미지에서 현재 레이어의 위쪽 경계를 계산합니다.
            original_window_bottom = cv_img.shape[0] - (i) * crop_layer  # 원본 이미지에서 현재 레이어의 아래쪽 경계를 계산합니다.
            cropped_window_top = croped_img_shape[0] - (i + 1) * crop_layer  # 자른 이미지에서 현재 레이어의 위쪽 경계를 계산합니다.
            cropped_window_bottom = croped_img_shape[0] - (i) * crop_layer  # 자른 이미지에서 현재 레이어의 아래쪽 경계를 계산합니다.

            histogram.append(np.sum(binary_img[cropped_window_top:cropped_window_bottom, :], axis=0))  # 레이어별로 히스토그램을 계산하고 저장합니다.
            indices.append(np.where(histogram[i] > threshold)[0])  # 히스토그램에서 차선으로 판단된 픽셀의 인덱스를 저장합니다.

            try:
                middle_point.append((min(indices[i]) + max(indices[i])) // 2)  # 차선의 중심 픽셀을 계산하고 저장합니다.
                cropped_window_left = middle_point[i] - self.margin  # 자른 이미지에서 왼쪽 경계를 계산합니다.
                cropped_window_right = middle_point[i] + self.margin  # 자른 이미지에서 오른쪽 경계를 계산합니다.

                # 차선 영역을 사각형으로 표시하고 중심점을 원으로 표시합니다.
                cv2.rectangle(croped_img, [cropped_window_left, cropped_window_top], [cropped_window_right, cropped_window_bottom], [0, 0, 255], 2)
                cv2.circle(croped_img, [middle_point[i], (cropped_window_top + cropped_window_bottom) // 2], 3, [0, 255, 0], -1)

                if self.L_R_Value != 1:
                    original_window_left = cropped_window_left
                    original_window_right = cropped_window_right
                    circle_point = middle_point[i]
                else:
                    original_window_left = cropped_window_left + cv_img.shape[1] // 2
                    original_window_right = cropped_window_right + cv_img.shape[1] // 2
                    circle_point = middle_point[i] + cv_img.shape[1] // 2

                # 원본 이미지에서 차선 영역을 사각형으로 표시하고 중심점을 원으로 표시합니다.
                cv2.rectangle(cv_img, [original_window_left, original_window_top], [original_window_right, original_window_bottom], [0, 0, 255], 2)
                cv2.circle(cv_img, [circle_point, (original_window_top + original_window_bottom) // 2], 3, [0, 255, 0], -1)
            except:
                print(f"line_not_detect:{[i]}")  # 예외가 발생하면 "line_not_detect" 메시지를 출력합니다.

        try:
            weight_point = np.zeros_like(middle_point)
            for index, point in enumerate(middle_point):
                weight_point[index] = middle_point[0] - (middle_point[0] - point) * (len(middle_point) - index) / len(middle_point)
            avg_point = int(np.average(weight_point))  # 가중 평균 중심점을 계산합니다.
            self.previous_point = avg_point
        except:
            avg_point = self.previous_point  # 평균 중심점을 계산할 수 없는 경우 이전 중심점을 사용합니다.

        if self.L_R_Value != 1:
            steering_standard = self.Steering_Standard
        else:
            steering_standard = croped_img_shape[1] - self.Steering_Standard
        cv2.rectangle(
            cv_img,
            (steering_standard, cv_img.shape[0] - croped_img_shape[0]),
            (cv_img.shape[1] - steering_standard, cv_img.shape[0]),
            (255, 0, 0),
            5,
        )
        cv2.line(croped_img, (self.Steering_Standard, 0), (self.Steering_Standard, croped_img_shape[0]), (255, 0, 0), 3)
        str_standard_point = "standard x:" + str(self.Steering_Standard)  # 차선의 표준 위치를 문자열로 생성합니다.
        str_avg_point = "avg point:" + str(avg_point)  # 평균 중심점을 문자열로 생성합니다.

        # 이미지에 표준 위치와 평균 중심점을 표시합니다.
        cv2.putText(cv_img, str_standard_point, (cv_img.shape[1] // 8, cv_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_img, str_avg_point, (cv_img.shape[1] // 8, cv_img.shape[0] // 4), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        return avg_point  # 평균 중심점을 반환합니다.

    def cal_angle_per_pixel(self, avg_point, cv_img):
        # 화면 상의 픽셀당 각도를 계산하는 메서드입니다.
        angle_resolution = pi / cv_img.shape[1]
        return angle_resolution

    def sense(self):
        # 로봇이 이미지를 "감지(sense)"하는 함수입니다.
        img_msg = self.img_msg  # 로봇으로부터 수신한 이미지 메세지를 변수에 저장합니다.
        cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환합니다.

        if self.create_trackbar_flag == False:
            self.init_standard = cv_img.shape[1] // 4  # 로봇 스티어링의 기준점을 설정합니다.
            self.create_trackbar_init(cv_img)  # 트랙바 초기화 함수를 호출합니다.

        return cv_img  # 처리된 이미지를 반환합니다.

    def think(self, cv_img):
        # 로봇이 이미지를 분석하고 "생각(think)"하는 함수입니다.
        hsv_img = self.apply_mask(cv_img)  # 이미지에서 마스크를 적용하여 HSV 이미지로 변환합니다.
        croped_img, croped_img_shape = self.crop_img(hsv_img)  # 이미지를 자르고 자른 이미지의 모양을 반환합니다.
        binary_img = self.binary(croped_img)  # 이진 이미지로 변환합니다.
        avg_point = self.sliding_window(cv_img, croped_img, binary_img, croped_img_shape)  # 이미지 슬라이딩 윈도우를 사용하여 평균 포인트를 계산합니다.
        angle_resolution = self.cal_angle_per_pixel(avg_point, cv_img)  # 이미지 각도 해상도를 계산합니다.

        return croped_img, avg_point, angle_resolution

    def act(self, cv_img, avg_point, angle_resolution):
        # 로봇의 동작 및 제어 동작을 수행하는 함수입니다.
        if self.Stop_or_Go == 0:
            # 로봇의 정지 또는 주행 상태를 확인하고 설정합니다.
            self.cmd_msg.linear.x = 0  # 선속도(linear.x)를 0으로 설정하여 정지합니다.
            self.cmd_msg.angular.z = 0  # 각속도(angular.z)를 0으로 설정하여 스티어링을 중앙에 위치시킵니다.
        else:
            self.cmd_msg.linear.x = self.Speed_Value / 10  # 선속도(linear.x)를 설정합니다. 사용자 설정된 값에서 10으로 나눈 값을 사용합니다.

            if self.L_R_Value != 1:
                self.cmd_msg.angular.z = -angle_resolution * abs(avg_point - self.Steering_Standard)
            else:
                self.cmd_msg.angular.z = angle_resolution * abs(avg_point - self.Steering_Standard)

        # 로봇의 제어 명령을 이미지에 표시합니다.
        str_speed = "speed:" + str(self.cmd_msg.linear.x)
        str_steering = "steer:" + str(round(self.cmd_msg.angular.z, 2))
        cv2.putText(
            cv_img, str_speed, (cv_img.shape[1] // 2 + cv_img.shape[1] // 8, cv_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA
        )
        cv2.putText(
            cv_img, str_steering, (cv_img.shape[1] // 2 + cv_img.shape[1] // 8, cv_img.shape[0] // 4), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA
        )
        self.pub.publish(self.cmd_msg)  # 로봇의 제어 명령을 게시(publish)합니다.

    def cam_CB(self, msg):
        # 이미지 메세지를 받아오기 위한 콜백 함수입니다.
        if msg != -1:
            self.img_msg = msg  # 유효한 메세지인 경우, 메세지를 self.img_msg로 저장합니다.
            self.cam_flag = True  # 이미지 메세지 수신 확인 변수를 True로 설정합니다.
        else:
            self.cam_flag = False  # 이미지 메세지 수신 확인 변수를 False로 설정합니다.

    def run(self):
        # 로봇의 메인 루프 함수로 이미지 처리 및 제어 동작이 수행됩니다.
        if self.cam_flag:
            cv_img = self.sense()  # 이미지를 감지하고 가져옵니다.

            if self.create_trackbar_flag:
                croped_img, avg_point, angle_resolution = self.think(cv_img)  # 이미지 분석 및 "생각" 함수를 호출합니다.
                self.act(cv_img, avg_point, angle_resolution)  # 동작과 제어 함수를 호출합니다.

                cv2.imshow(self.original_window, cv_img)  # 원본 이미지를 OpenCV 창에 표시합니다.
                cv2.imshow(self.cropped_window, croped_img)  # 처리된 이미지를 OpenCV 창에 표시합니다.
                cv2.waitKey(1)  # 이미지 표시를 위한 대기시간입니다.


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
    main()  # 주요 프로그램이 시작됩니다.
