#!/usr/bin/env python
# -*- coding:utf8 -*-

import rospy
import math
import time
import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import random
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from hunter_msgs.msg import HunterStatus

class Logistics_vehicle:
	pose_x = 0.0
	pose_y = 0.0
	rotat_w = 0.0
	recv_flag = 1

	def __init__(self):
		self.MQTTHOST = "192.168.3.6"
		self.MQTTPORT = 50000
		self.mqttClient = mqtt.Client()
		mqtt_msg = dict()
		self.id = 1
		self.sub_name, self.sub_dir, self.sub_action, self.sub_msg, self.sub_feedback  = None, None, None, None, None
		self.sub_dir1, self.sub_dir2 = None, None
		rospy.init_node("logistics_vehicle", anonymous = True)
		self.Auto_flag = False
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback)
		rospy.Subscriber('hunter_status', HunterStatus, self.td_callback)		
		#t1 = threading.Thread(target=self.ros_spin)
		#t1.start()
		self.td_auto1, self.td_auto2 = 1, 1
		self.on_mqtt_connect()
		print("waiting for mqtt msg")
		mod = 1
		while not rospy.is_shutdown():
			recv_msg = []

	# 连接MQTT服务器
	def on_mqtt_connect(self):
		self.mqttClient.connect(self.MQTTHOST, self.MQTTPORT, 60)
		self.mqttClient.loop_start()

	# publish 消息
	def on_publish(self, topic, payload, qos):
		print("pub msg: %s to %s" % (payload, topic))
		self.mqttClient.publish(topic, payload, qos)

	# 消息处理函数
	def on_message_come(self, lient, userdata, msg):
		recv_flag = 1
		#print("from: "+ msg.topic + " " + ":" + msg.payload)
		self.sub_msg = json.loads(msg.payload.encode('utf-8'))
		# # self.sub_name, self.sub_dir, self.sub_action = str(self.sub_msg["name"]), str(self.sub_msg["dir"]), self.sub_msg["ation"]
        # self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = str(self.sub_msg['name']), str(self.sub_msg['dir']), str(self.sub_msg['ation']), self.sub_msg['feedback']
        # self.sub_dir1, self.sub_dir2 = str(self.sub_msg['dir1']), str(self.sub_msg['dir2'])

		if len(self.sub_msg) == 6:
			self.sub_name,  self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['ation'], self.sub_msg['feedback']
			self.sub_dir1, self.sub_dir2 = self.sub_msg['dir1'], self.sub_msg['dir2']
		elif len(self.sub_msg) == 5:
			self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['dir'], self.sub_msg['ation'], self.sub_msg['feedback']
		else:
			pass

	def get_mqtt_msg(self):
		return self.mqtt_msg

	# subscribe 消息
	def on_subscribe(self):
		# mqttClient.subscribe("/HG_DEV/TD_MSG", 2)
		self.mqttClient.subscribe("/HG_CAR/CAR_MSG", 0)
		self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    # 向⻔吊发送请求装货指令
	def Auto_TD_cmd(self):
		json_dict = OrderedDict()
		json_dict["name"] = "Auto"
		json_dict["dir"] = "TD"
		json_dict["ation"] = "loading"
		json_dict["error"] = "null"
		json_dict["feedback"] = 0
		param = json.dumps(json_dict)
		return param

    # 向⻔吊发送收到卸货完成指令
	def Auto_TD_reply(self):
		json_dict = OrderedDict()
		json_dict["name"] = "Auto"
		json_dict["dir"] = "TD"
		json_dict["ation"] = "unloading complete"
		json_dict["error"] = "null"
		json_dict["feedback"] = 1
		param = json.dumps(json_dict)
		return param

    # 向⻔吊发送收到装货完成指令
	def Auto_TD_LC_reply(self):
		json_dict = OrderedDict()
		json_dict["name"] = "Auto"
		json_dict["dir"] = "TD"
		json_dict["ation"] = "loading complete"
		json_dict["error"] = "null"
		json_dict["feedback"] = 1
		param = json.dumps(json_dict)
		return param

	def dist2(self, px, py):
		dx = px - self.pose_x
		dy = py - self.pose_y
		distance = math.sqrt((dx*dx) + (dy*dy))
		return  distance

	def forward(self, v, d):
		cmd_vel = Twist()
		pose_x_ = self.pose_x
		pose_y_ = self.pose_y
		# cmd_vel.linear.x = v
		cmd_vel.linear.x = 0.2
		flag = 1
		print("forward")
		while(flag):
			if self.dist2(pose_x_, pose_y_) < d :
				flag = 1
				self.cmd_pub.publish(cmd_vel)
			else :
				flag = 0
				self.id = 0	
			
	def backward(self, v, d):
		cmd_vel = Twist()
		pose_x_ = self.pose_x
		pose_y_ = self.pose_y
		# cmd_vel.linear.x = -v
		cmd_vel.linear.x = -0.2
		flag = 1
		print("backward")
		while(flag):
			if self.dist2(pose_x_, pose_y_) < d :
				flag = 1
				self.cmd_pub.publish(cmd_vel)
			else :
				flag = 0

	def approach(self):
		cmd_vel = Twist()
		print("approach")

		while self.pose_x < 3.83:
			if self.pose_x <= 1.9:
				linear_x = 0.3
				angular_z = 0.0
				# print("forward")
				cmd_vel.linear.x = linear_x

			if (self.pose_x > 1.9 and self.rotat_w > 0.78):
				linear_x = 0.3
				angular_z = 1.0
				# print("trun left   :" + str(self.rotat_w))
				cmd_vel.linear.x = linear_x
				cmd_vel.angular.z = angular_z

			self.cmd_pub.publish(cmd_vel)
			time.sleep(0.05)
		
		#self.on_publish("/HG_CAR/CAR_MSG", self.action1(), 2)

	def departure(self):
		cmd_vel = Twist()
		print("departure")
		
		for i in range(3):
			if i == 0:
				print("  backward")
				while self.pose_y >0:
					linear_x = -0.2
					angular_z = 0.0
					# print("backward")
					cmd_vel.linear.x = linear_x
					self.cmd_pub.publish(cmd_vel)
					time.sleep(0.1)
			elif i == 1:
				print("  turn left")
				while (self.pose_x > 2.6 and self.pose_x <3.8):
					linear_x = 0.2
					angular_z = 1.0
					cmd_vel.linear.x = linear_x		
					cmd_vel.angular.z = angular_z
					self.cmd_pub.publish(cmd_vel)
					time.sleep(0.1)
			elif i == 2:
				print("  turn right")
				while (self.pose_x > 1.0 and self.pose_y <4.3 and self.rotat_w < 0.775):
					linear_x = 0.15
					angular_z = -1.0
					cmd_vel.linear.x = linear_x		
					cmd_vel.angular.z = angular_z
					self.cmd_pub.publish(cmd_vel)
					time.sleep(0.1)
			elif i == 3:
				self.wait()

	def wait(self):
		cmd_vel = Twist()
		print("wait")
		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.linear.z = 0
		cmd_vel.angular.x = 0
		cmd_vel.angular.y = 0
		cmd_vel.angular.z = 0
		self.cmd_pub.publish(cmd_vel)

	def socket_callback(self, mod):
		if mod == 0:
			self.wait()
		elif mod == 1:
			while self.td_auto1:
			    self.approach()
			    self.wait()
			    time.sleep(2)
			    self.forward(0.2, 1.5)
			    self.wait()
			    time.sleep(3)
			    if self.id == 0:
			        self.td_auto1 = 0
				self.id = 1

		elif mod == 2:
			while self.td_auto2:
			    self.departure()
			    self.wait()
			    time.sleep(1)

			    self.forward(0.2, 0.5)
			    self.wait()
			    time.sleep(3)
			    if self.id == 0:
			        self.td_auto2 = 0
				self.id = 1
		else:
			self.wait()

	def callback(self, data):
		self.pose_x = data.pose.pose.position.x
		self.pose_y = data.pose.pose.position.y
		self.rotat_w = data.pose.pose.orientation.w

	def td_callback(self, data):
		self.on_subscribe()
		if (self.sub_name == "TD") and (self.sub_dir2 == "Auto") and (self.sub_action == "unloading complete") and self.sub_feedback == 0:
			for i in range(0, 4):
				self.on_publish("/HG_CAR/CAR_MSG", self.Auto_TD_reply(), 0)
			while self.td_auto1:
				self.socket_callback(1)
			if self.td_auto1 == 0:
				self.Auto_flag = True
		elif (self.sub_name == "TD") and (self.sub_dir == "Auto") and (self.sub_action == "loading complete") and self.sub_feedback == 0:
			for i in range(0, 4):
				self.on_publish("/HG_CAR/CAR_MSG", self.Auto_TD_LC_reply(), 0)
			while self.td_auto2:
				self.socket_callback(2)
		else:
			pass

		if self.Auto_flag:
			self.on_publish("/HG_CAR/CAR_MSG", self.Auto_TD_cmd(), 0)
			time.sleep(1)
        # 收到回复时停止发送
		if self.sub_name == "TD" and  self.sub_action == "loading" and self.sub_feedback == 1:
			self.Auto_flag = False


if __name__ == "__main__" :
	Logistics_vehicle()
