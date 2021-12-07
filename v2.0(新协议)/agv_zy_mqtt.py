#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
import time

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import random
import time
import threading
import ast

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.MQTTHOST = "192.168.3.6"	# 服务器ip
        self.MQTTPORT = 50000	# 服务器端口
        self.mqttClient = mqtt.Client()
        self.AD_flag, self.TD_flag, self.action0, self.action2 = False, False, False, False
        self.sub_name, self.sub_dir, self.sub_action, self.sub_msg, self.sub_feedback  = None, None, None, None, None
        self.sub_dir1, self.sub_dir2 = None, None

	# 初始化数据
        self.nav_id_result, self.odom_x, self.odom_y, self.sub_msg =  None, None, None, None
        self.sub_name, self.sub_dir, self.sub_action =  None, None, None
        self.i, self.cm = 0, 0.0
        rospy.init_node('send_goals_node', anonymous=False)
        self.and_id = 1
        self.td_id = 1
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/agv_nav_point', Int64 , self.nav_id) 
        rospy.Subscriber('/Odom', Odometry, self.agv_odom)
	
	#连接MQTT服务器
        self.on_mqtt_connect()

        # 订阅陀螺仪数据作为一直在跑的一个线程
        rospy.Subscriber('/IMU', Imu, self.flag_data)
        self.on_subscribe()
	
        rospy.spin()

    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.MQTTHOST, self.MQTTPORT, 60)
        self.mqttClient.loop_start()

    # publish 消息
    def on_publish(self, topic, payload, qos):
        #print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 对mqtt订阅消息处理函数
    def on_message_come(self, lient, userdata, msg):
	# python3 bytes转str
        self.sub_msg = str(msg.payload, 'utf-8')
	    # python3 str转字典
        self.sub_msg = ast.literal_eval(self.sub_msg)
        if len(self.sub_msg) == 6:
            self.sub_name,  self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['ation'], self.sub_msg['feedback']
            self.sub_dir1, self.sub_dir2 = self.sub_msg['dir1'], self.sub_msg['dir2']
        elif len(self.sub_msg) == 5:
            self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['dir'], self.sub_msg['ation'], self.sub_msg['feedback']
        else:
            pass

    # mqtt subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_CAR/CAR_MSG", 0)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    # 向岸吊发送请求装货指令
    def AGV_AD_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AGV"
        json_dict["dir"] = "AD"
        json_dict["ation"] = "loading"
        json_dict["error"] = "null"
        json_dict["feedback"] = 0
        param = json.dumps(json_dict)
        return param

    # 向⻔吊发送卸货指令
    def AGV_TD_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AGV"
        json_dict["dir"] = "TD"
        json_dict["ation"] = "unloading"
        json_dict["error"] = "null"
        json_dict["feedback"] = 0
        param = json.dumps(json_dict)
        return param

    # 向岸吊回复收到装货完成指令
    def AGV_AD_reply(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AGV"
        json_dict["dir"] = "AD"
        json_dict["ation"] = "loading complete"
        json_dict["error"] = "null"
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param

    # 向岸吊回复收到装货完成指令
    def AGV_TD_reply(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AGV"
        json_dict["dir"] = "TD"
        json_dict["ation"] = "unloading complete"
        json_dict["error"] = "null"
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param


    def agv_odom(self, pose):
        self.odom_x = pose.pose.pose.position.x
        self.odom_y = pose.pose.pose.position.y
	# 订阅mqtt服务器的消息
        # self.on_subscribe()
        print(self.sub_msg)
        if self.AD_flag:
            self.on_publish("/HG_CAR/CAR_MSG", self.AGV_AD_cmd(), 0)
            time.sleep(1)
        # 收到回复时停止发送
        if self.sub_name == "AD" and  self.sub_action == "loading" and self.sub_feedback == 1:
            self.AD_flag = False
            
        if self.TD_flag:
            self.on_publish("/HG_CAR/CAR_MSG", self.AGV_TD_cmd(), 0)
            time.sleep(1)
        # 收到回复时停止发送
        if self.sub_name == "TD" and  self.sub_action == "unloading" and self.sub_feedback == 1:
            self.TD_flag = False

    def nav_id(self, data):
        self.nav_id_result = data.data

    # 导航到目标点
    def flag_data(self, data):
        #print(self.sub_name, self.sub_dir, self.sub_action)
        #print(type(self.sub_name), type(self.sub_dir), type(self.sub_action))
        if self.nav_id_result == 0:
                self.and_goal(self.i)
                if self.i == 3:
                    self.nav_id_result = -1
                    self.i = 0
                    self.odom_and = self.odom_y
                    #	向岸吊发送上料申请
                    #self.on_publish("/HG_CAR/CAR_MSG", self.and_object_cmd(), 2)
                    self.AD_flag = True
                    #self.AD_flag_id = 0

	# 接收到装货完成进行搬运任务
        elif self.nav_id_result == 1 or  self.action0:
                self.AD_flag = False
                if self.and_id:
                    self.and_back_goal(self.i)
                if self.i == 1:
                    self.nav_id_result = 2
                    self.i, self.action0, self.action1 = 0, False, True

        elif (self.nav_id_result == 2 and self.action1):
                self.md_goal(self.i)
                if self.i == 2:
                    self.nav_id_result = -1
                    self.i, self.action1 = 0, False
                    self.odom_md = self.odom_y
                    self.TD_flag = True

	# 接收到装货完成回到原点
        elif self.nav_id_result == 3 or self.action2:
                if self.td_id:
                    self.md_back_goal(self.i)
                if self.i == 1:
                    self.nav_id_result = 4
                    self.i, self.action2, self.action3 = 0, False, True
                    self.sub_name, self.sub_action = None, None

        elif self.nav_id_result == 4 and self.action3:
                self.goHome_goal(self.i)
                if self.i == 2:
                    self.nav_id_result = -1
                    self.i, self.action3 = 0, False

        else:
                self.i = 0

        #   从AGV上面卸货（集装箱）
        if self.sub_name == "AD" and self.sub_dir == "AGV" and self.sub_action == "loading complete" and self.sub_feedback == 0:
            self.action0 = True
            if self.sub_feedback == 0:
                # ⻔吊向 AGV 回复收到卸货指令
                for i in range(0, 4):
                    self.on_publish("/HG_CAR/CAR_MSG", self.AGV_AD_reply(), 0)

        #   门吊装货完成
        if self.sub_name == "TD" and self.sub_dir1 == "AGV" and self.sub_action == "unloading complete" and self.sub_feedback == 0:
            self.action2 = True
            if self.sub_feedback == 0:
                # ⻔吊向 AGV 回复收到卸货指令
                for i in range(0, 4):
                    self.on_publish("/HG_CAR/CAR_MSG", self.AGV_TD_reply(), 0)

    #   去岸吊导航函数
    def and_goal(self, i): 
        waypointsx = list([0.39805161953, 0.683694541454, -0.440528422594])
        waypointsy = list([1.05252337456, 2.63965296745, 2.79990077019])

        waypointsaw = list([0.629090530925, 0.998381476877, 0.99678816435])
        waypointsw = list([0.777332042245, 0.0568720197451, 0.0800834278229])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def and_back_goal(self, i): 
        move_cmd = Twist()
        while(self.and_id):
            if ((self.odom_and - self.odom_y) < 1.4 and (self.odom_and - self.odom_y) > 0) or ((self.odom_and - self.odom_y) > -1.4 and (self.odom_and - self.odom_y) < 0):
                move_cmd.linear.x = -0.2
                self.cmd_vel.publish(move_cmd) 
            else:
                move_cmd.linear.x = 0.0
                self.i = 1
                self.cmd_vel.publish(move_cmd) 
                self.and_id = 0
        
    #   门吊（塔吊）导航函数
    def md_goal(self, i): 
        waypointsx = list([0.825305879116, 2.07601952553])
        waypointsy = list([2.50716161728, 2.21167135239])

        waypointsaw = list([-0.108289180707, -0.0384742591575])
        waypointsw = list([0.994119436155, 0.999259591589])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def md_back_goal(self, i): 
        move_cmd = Twist()
        while(self.td_id):
            if ((((self.odom_y - self.odom_md) < 1.3) and ((self.odom_y - self.odom_md) > 0))  or (((self.odom_y - self.odom_md) > -1.3) and ((self.odom_y - self.odom_md) < 0))):
                move_cmd.linear.x = -0.2
                self.cmd_vel.publish(move_cmd) 
            else:
                move_cmd.linear.x = 0.0
                self.i = 1
                self.cmd_vel.publish(move_cmd) 
                self.td_id = 0
       
    #   起始点导航函数
    def goHome_goal(self, i): 
        waypointsx = list([0.79198884964, 1.73415076733])
        waypointsy = list([2.087038517, 0.87123799324])

        waypointsaw = list([-0.736561001492, -0.746177819979])
        waypointsw = list([0.676371119343, 0.6657466943])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    def move(self, goal):
        self.ac.send_goal(goal)

        # 设定5分钟的时间限制
        finished_within_time = self.ac.wait_for_result(rospy.Duration(300))

        # 如果5分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.i += 1
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Cancel any active goals
        self.ac.cancel_goal()
        #rospy.sleep(16)

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
