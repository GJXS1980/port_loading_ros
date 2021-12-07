#!/usr/bin/python
# -*- coding:utf8 -*-

import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import rospy
import random
import time
import threading
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Image
import ast
import math

class mqtt_AT:
    def __init__(self):
        rospy.init_node("an_node")
        self.mqtt_host = "192.168.3.6"
        self.mqtt_port = 50000
        self.mqttClient = mqtt.Client()
        self.on_mqtt_connect()

        self.sub_name, self.sub_dir, self.sub_action, self.sub_msg = None, None, None, None
        self.AD_grasp_flag, self.AD_flag = True, False
        self.send_time = 0
        self.wait_time = 0
        self.an_current_x, self.an_current_y, self.an_current_z = 0.0, 0.0, 0.0
        self.an_grasp_state = 0
        self.an_target_pose = [0.0, 0.0, 0.0]
        self.action0_state, self.action1_state, self.action2_state, self.action3_state = True, False, False, False
        
        # 发布坐标数据话题
        self.an_control = rospy.Publisher('/Pall_POS_SET', Float32MultiArray, queue_size=5)
        #  订阅查询当前的位置
        self.an_control_pos = rospy.Subscriber("/Pall_CURR_POS", Float32MultiArray, self.get_an_pose)

        # mqtt控制
        rospy.Subscriber("/usb_cam/image_raw", Image, self.ad_mqtt)

        #  发布电磁铁状态话题
        self.an_grasp_control = rospy.Publisher('/Pall_Grasp_Topic', Int32, queue_size=5)
        #  订阅电磁铁状态话题
        self.an_grasp_state = rospy.Subscriber("/Pall_Grasp_Topic", Int32, self.get_an_control)
        rospy.spin()

    # 获取当前xyz坐标位置
    def get_an_pose(self, data):
        if data.data[0] == None or abs(data.data[0]) > 1800:
            pass
        else:
            self.an_current_x = data.data[0]
        if data.data[1] == None or  abs(data.data[1]) > 1800:
            pass
        else:
            self.an_current_y = data.data[1]
        if data.data[2] == None or  abs(data.data[2]) > 1800:
            pass
        else:
            self.an_current_z = data.data[2]
        #print(data.data)

    # 主函数
    def ad_mqtt(self, data):
        self.on_subscribe()
        if self.sub_name == "AGV" and self.sub_dir == "AD" and self.sub_action == 1 and self.AD_grasp_flag:
            # if self.set_an_pose([150.0, 650.0, 510.0]):  # 抓取点
            while self.action0_state:
            # 抓取船上的集装箱
                self.set_an_xy_pose([151.8, 656.7]) # 抓取点
                if self.set_an_xy_pose_state([151.8, 656.7]) :
                    self.set_an_z_pose(510.0)
                    if self.set_an_z_pose_state(510.0):
                        self.set_an_grasp_control(1)
                        self.action0_state = False
                        self.action1_state = True
                    
            # 往上提集装箱
            while self.action1_state:
                self.set_an_z_pose(300.0)
                if self.set_an_z_pose_state(300.0):
                    self.action1_state = False
                    self.action2_state = True            
                                
            while self.action2_state:
                # 抓取船上的集装箱
                self.set_an_xy_pose([166.0, 34.0]) # 抓取点
                if self.set_an_xy_pose_state([166.0, 34.0]) :
                    self.set_an_z_pose(490.0)
                    if self.set_an_z_pose_state(490.0):
                        self.set_an_grasp_control(0)
                        self.action2_state = False
                        self.action3_state = True

            # 往上提集装箱
            while self.action3_state:
                self.set_an_z_pose(100.0)
                if self.set_an_z_pose_state(100.0):
                    self.action3_state = False
                    self.AD_grasp_flag = False
                    self.AD_flag = True       
        if self.AD_flag:
            self.on_publish("/HG_CAR/CAR_MSG", self.AD_cmd(), 2)
        elif self.sub_name == "AGV" and self.sub_dir == "TD" and self.sub_action == 0:
            self.AD_flag = False

    # 岸吊控制程序(xy方向)
    def set_an_xy_pose(self,  data):
        move_xy = [data[0], data[1], self.an_current_z]
        target_pose = Float32MultiArray()
        target_pose.data = move_xy
        self.an_control.publish(target_pose)
        time.sleep(math.ceil(data[1]/20.97))
        # # 发表坐标话题
        # for i in range(1):
        #     self.an_control.publish(target_pose)
        #     time.sleep(1)

    # xy状态判断
    def set_an_xy_pose_state(self,  data):
        if abs(self.an_current_x -data[0]) < 5 and abs(self.an_current_y - data[1]) < 5:
            return True

    # 岸吊控制程序(z方向)
    def set_an_z_pose(self, data):
        move_z = [self.an_current_x, self.an_current_y, data]
        target_pose = Float32MultiArray()
        target_pose.data = move_z
	self.an_control.publish(target_pose)
	time.sleep(math.ceil(data/80))
        # 发表坐标话题
        #for i in range(1):
         #   self.an_control.publish(target_pose)
          #  time.sleep(6)

    # z状态判断
    def set_an_z_pose_state(self,  data):
        if (abs(self.an_current_z - data) < 15):
            return True

    # def set_an_pose(self, data):
    #     if data[0] != 0.0 or data[1] != 0.0:  # x,y不为0,先移动x,y再移动z
    #         move_xy = [data[0], data[1], self.an_current_z]
    #         target_pose = Float32MultiArray()
    #         target_pose.data = move_xy
    #         print(target_pose)
    #         for i in range(4):
    #             self.an_control.publish(target_pose)
    #         while True:
    #             if abs(self.an_current_x -data[0]) < 5 and abs(self.an_current_y - data[1]) < 5:
    #                 break
    #         move_z = [data[0], data[1], data[2]]
    #         target_pose = Float32MultiArray(data=move_z)
    #         print(target_pose)
    #         for i in range(4): 
    #             self.an_control.publish(target_pose)
    #         while True:
    #             if (abs(self.an_current_z - data[2]) < 30):
    #                 self.wait_time = 0
    #                 break
    #             else:
    #                 self.wait_time += 1
    #                 time.sleep(1.0)
    #             if self.wait_time > 15:
    #                 self.wait_time = 0
    #                 break
    #         return True
    #     else:  # x,y为0则单独控制z
    #         move_z = [self.an_current_x, self.an_current_y, data[2]]
    #         target_pose = Float32MultiArray(data=move_z)
    #         for i in range(4):
    #             self.an_control.publish(target_pose)
    #         while True:
    #             if (abs(self.an_current_z - data[2]) < 30):
    #                 time.sleep(2.0)
    #                 break
    #         return True

    # 获取电磁铁当前状态
    def get_an_control(self, data):
        self.an_grasp_state = data.data

    # 控制电爪吸合
    def set_an_grasp_control(self, data):
        self.an_grasp_control.publish(data)
        time.sleep(2.0)
        # for i in range(1):
        #     self.an_grasp_control.publish(data)
        #     time.sleep(1.0)
        return True
        
    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqttClient.loop_start()

    # publish 消息
    def on_publish(self, topic, payload, qos):
        print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, lient, userdata, msg):
        #print("from: " + msg.topic + " " + ":" + msg.payload)
        self.sub_msg = json.loads(msg.payload.encode('utf-8'))
        self.sub_name, self.sub_dir, self.sub_action = self.sub_msg['name'], self.sub_msg['dir'], self.sub_msg['ation']

    # 门吊（塔吊）下料申请（AGV）
    def AD_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AD"
        json_dict["dir"] = "AGV"
        json_dict["ation"] = 0
        json_dict["error"] = "null"
        param = json.dumps(json_dict)
        return param

    # subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_CAR/CAR_MSG", 2)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

if __name__ == '__main__':
    an_start = mqtt_AT()
    #an_start.start_mqtt()
    #rospy.spin()
