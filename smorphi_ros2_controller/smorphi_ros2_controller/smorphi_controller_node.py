#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *

import time
import math
import tf_transformations
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial

class SmorphiController(Node):
	def __init__(self):
		super().__init__("smorphi_controller_node")
		#self.get_logger().info("Hello")
		self.imu_flag = 0
		self.create_subscription(Imu, "imu_data", self.imucallback,50)
		self.wheel_odom_pub = self.create_publisher(Odometry, "wheel_odom", 50)
		self.create_subscription(Twist, "cmd_vel", self.writespeed, 50)
		
		self.create_subscription(Int32, "shape_need", self.shpneed, 10)
		#self.shape_pub = self.create_publisher(Int32, "current_shape", 50)
		self.ser = serial.Serial('/dev/smorphi_mb',115200, timeout=0.005)
		
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.w = 0.0
		#self.odomBroadcaster = TransformBroadcaster()
		self.imu_msg = Imu()
		self.shape_need = 0
		self.cur_shape = 0
		self.xdot = 0
		self.thetadot = 0
		self.xdot1 = 0
		self.thetadot1 = 0
		self.roll = 0
		self.pitch = 0
		self.prev_yaw = 0
		self.last_time = self.get_clock().now().to_msg()
		self.timer = self.create_timer(0.05, self.encoder_read)
		
		#self.rate = rclpy.Rate(50)

	def writespeed(self, msg):
		cm_Vx = "{:.2f}".format(float(msg.linear.x))
		cm_Vy = "{:.2f}".format(float(msg.linear.y))
		cm_Wz = "{:.2f}".format(float(msg.angular.z))
		cm_shape = self.shape_need
		cmd_vel = str(cm_Vx)+","+str(cm_Vy)+","+str(cm_Wz)+","+str(cm_shape)+"\n"
		#print ("cmd_vel")
		self.ser.write(cmd_vel.encode())

	def shpneed(self, msg2):
		self.shape_need = msg2.data

	def imucallback(self, msg):
		#self.get_logger().info("imu_subscribed")
		self.imu_msg = msg
		if self.imu_flag == 0:
			(self.roll, self.pitch, self.prev_yaw) = tf_transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
			print("imu_init=")
			print(self.prev_yaw)
			self.imu_flag = 1
            
	def encoder_read(self):
		
		shape_liz = ["i","o","l"]
		encoder_readings = str(self.ser.readline())

		if len(encoder_readings)>5:
			encoder_readings = encoder_readings[2:-5].split(",")
			if (len(encoder_readings) == 7):
				encoder_values = encoder_readings[:3]
				#print(encoder_readings)
				#cur_shape = shape_liz.index(encoder_readings[3])+1
				
				try:
				    encoder_values = [float(a) for a in encoder_values]
				except:
				    pass
				else:  
				    encoder_values = [float(a) for a in encoder_values]
				    self.vx = encoder_values[0]
				    self.vy = encoder_values[1]
				    #self.w = encoder_values[2]
			else:
				print("waiting")
				
			#print(self.vx, self.vy, self.w)
		#b = "0"
		(r, p, yaw) = tf_transformations.euler_from_quaternion([self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w])
		current_time = self.get_clock().now().to_msg()
		ct = current_time.sec + (current_time.nanosec/1e+9)
		lt = self.last_time.sec + (self.last_time.nanosec/1e+9)
		self.w = yaw - self.prev_yaw
		#print("new imu= ")
		#print(self.w)
		dt = (ct - lt)
		#print(dt)
		#dt = (current_time - self.last_time).to_sec()
		delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
		delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
		self.prev_yaw = yaw
		delta_th = self.w * dt

		self.x += delta_x
		self.y += delta_y
		self.th += self.w
		[quat_x, quat_y, quat_z, quat_w] = tf_transformations.quaternion_from_euler(0, 0, self.th)
		odom = Odometry()
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"

		# set the position
		#odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = quat_x
		odom.pose.pose.orientation.y = quat_y
		odom.pose.pose.orientation.z = quat_z
		odom.pose.pose.orientation.w = quat_w
		

		# set the velocity
		#odom.child_frame_id = "base_link"
		#odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.w))
		odom.twist.twist.linear.x = self.vx
		odom.twist.twist.linear.y = self.vy
		odom.twist.twist.angular.z = self.w

		#print(odom)
		# set the velocity
		
		#odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.w))

		# publish the message
		self.wheel_odom_pub.publish(odom)
		
		#self.shape_pub.publish(self.cur_shape)
		self.last_time = current_time

		#print(odom)
		# set the velocity
		
		#odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.w))

		# publish the message
		#self.wheel_odom_pub.publish(odom)
		
		#self.shape_pub.publish(self.cur_shape)
		#self.last_time = current_time




def main(args=None):
    rclpy.init(args=args)
    node_ = SmorphiController()
    #rate = node_.create_rate(50)
    while rclpy.ok():
    	rclpy.spin(node_)
    	#rclpy.spin(node_.encoder_read())
    	#rate.sleep()
    	#node_.rate.sleep()
    node_.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
