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

        self.sub_name, self.sub_dir, self.sub_action, self.sub_msg, self.sub_feedback  = None, None, None, None, None
        self.AD_grasp_flag, self.AD_flag = False, False
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
        self.on_subscribe()
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
        # self.on_subscribe()
        if self.sub_name == "AGV" and self.sub_dir == "AD" and self.sub_action == "loading" and self.sub_feedback == 0:
            self.AD_grasp_flag = True
            if self.sub_feedback == 0:
                # 岸吊向 AGV 回复请求装货指令已经收到
                for i in range(0, 4):
                    self.on_publish("/HG_CAR/CAR_MSG", self.AD_AGV_reply(), 0)

        if self.AD_grasp_flag:
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
                # 平移集装箱并放到AGV上面
                self.set_an_xy_pose([166.0, 34.0]) # 抓取点
                if self.set_an_xy_pose_state([166.0, 34.0]) :
                    self.set_an_z_pose(490.0)
                    if self.set_an_z_pose_state(490.0):
                        self.set_an_grasp_control(0)
                        self.action2_state = False
                        self.action3_state = True

            # 往上提末端
            while self.action3_state:
                self.set_an_z_pose(100.0)
                if self.set_an_z_pose_state(100.0):
                    self.action3_state = False
                    self.AD_grasp_flag = False
                    self.AD_flag = True       
        if self.AD_flag :
            # 岸吊向 AGV 发送装货完成指令
            self.on_publish("/HG_CAR/CAR_MSG", self.AD_cmd(), 0)
            time.sleep(1)
        # 收到回复时停止发送
        if self.sub_name == "AGV" and self.sub_dir == "AD" and self.sub_action == "loading complete" and self.sub_feedback == 1:
            self.AD_flag = False

    # 岸吊控制程序(xy方向)
    def set_an_xy_pose(self,  data):
        move_xy = [data[0], data[1], self.an_current_z]
        target_pose = Float32MultiArray()
        target_pose.data = move_xy
        self.an_control.publish(target_pose)
        time.sleep(math.ceil(data[1]/20.97))

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

    # z状态判断
    def set_an_z_pose_state(self,  data):
        if (abs(self.an_current_z - data) < 15):
            return True

    # 获取电磁铁当前状态
    def get_an_control(self, data):
        self.an_grasp_state = data.data

    # 控制电爪吸合
    def set_an_grasp_control(self, data):
        self.an_grasp_control.publish(data)
        time.sleep(2.0)
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
        self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['dir'], self.sub_msg['ation'], self.sub_msg['feedback']

    # 岸吊向 AGV 发送装货完成指令
    def AD_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AD"
        json_dict["dir"] = "AGV"
        json_dict["ation"] = "loading complete"
        json_dict["error"] = "null"
        json_dict["feedback"] = 0
        param = json.dumps(json_dict)
        return param

    # 岸吊向 AGV 回复请求装货指令已经收到
    def AD_AGV_reply(self):
        json_dict = OrderedDict()
        json_dict["name"] = "AD"
        json_dict["dir"] = "AGV"
        json_dict["ation"] = "loading"
        json_dict["error"] = "null"
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param

    # subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_CAR/CAR_MSG", 0)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

if __name__ == '__main__':
    an_start = mqtt_AT()

