
import rospy
from sensor_msgs.msg import LaserScan
from random import *
from math import *
from morai_msgs.msg import CtrlCmd
class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")
        rospy.Subscriber("/scan",LaserScan,self.callback)
        self.ctrl_pub = rospy.Publisher("ctrl_cmd",CtrlCmd,queue_size=1)
        self.cmd_msg = CtrlCmd()
        
    def callback(self,msg:LaserScan):        
        degree_min = degrees(msg.angle_min)
        degree_max = degrees(msg.angle_max)
        degree_increment = degrees(msg.angle_increment)
        obstacle = []
        side_degrees = []
        cal_degrees = [degree_min+degree_increment*index for index,value in enumerate(msg.ranges)]

        min_diff_minus_90=inf
        min_diff_plus_90=inf
        mid_space = 0
        mid_avg_degree = 0
        for i,v in enumerate(msg.ranges):

            diff_minus_90 = abs(cal_degrees[i] + 90)  # Distance from -90 degrees
            diff_plus_90 = abs(cal_degrees[i] - 90)   # Distance from 90 degrees
            if diff_minus_90 < min_diff_minus_90:
                min_diff_minus_90 = diff_minus_90
                closest_minus_90_idx = i

            if diff_plus_90 < min_diff_plus_90:
                min_diff_plus_90 = diff_plus_90
                closest_plus_90_idx = i

            if abs(cal_degrees[i])< 90 and 0<msg.ranges[i]<3:
                obstacle.append(i)
                if len(obstacle)>=2:
                    if 100<obstacle[-1]-obstacle[-2]:
                        mid_space = obstacle[-1]-obstacle[-2]
                        mid_avg_degree = cal_degrees[ (obstacle[-1]+obstacle[-2]) // 2 ]
                        print(obstacle)
            else :
                pass

        print(f"closest_plus_90_idx:{closest_plus_90_idx}")
        print(f"closest_minus_90_idx:{closest_minus_90_idx}")
        if obstacle != []:
            print("장애물 있음")

            left_space =  closest_plus_90_idx - obstacle[-1]
            right_space = obstacle[0]-closest_minus_90_idx

            left_avg_degree = cal_degrees[ (closest_plus_90_idx+obstacle[-1]) // 2 ]
            right_avg_degree = cal_degrees[ (obstacle[0]+closest_minus_90_idx) // 2 ]
            print(left_avg_degree,right_avg_degree)
            if max([left_space,mid_space,right_space])==left_space:
                print("왼쪽 턴")
                steer = radians(left_avg_degree)
            elif max([left_space,mid_space,right_space])==right_space :    
                print("오른쪽 턴")
                steer = radians(right_avg_degree)
            else :
                print("미드 턴")
                steer = radians(mid_avg_degree)
            print([left_space,mid_space,right_space])
            self.cmd_msg.steering = steer 
            print(degrees(self.cmd_msg.steering))
        else :
            print("장애물 없음")
            self.cmd_msg.steering =0.0

        self.cmd_msg.accel = 0.1
        self.cmd_msg.brake = 0.0
        self.ctrl_pub.publish(self.cmd_msg)

        print("-"*30)

        # print(cal_degrees)
sub_class = Sub_class()
rospy.spin()