import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32

from geometry_msgs.msg import Twist,Pose2D,TransformStamped
from rclpy.time import Time

import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import tf2_ros

from rclpy.qos import qos_profile_sensor_data

class Robot(Node):
	def __init__(self):
		super().__init__('lapras_sim_support_node')
		
		self.vel_sub = self.create_subscription(Odometry, 'odom/raw', self.odom_callback,10)
		self.odom_pub = self.create_publisher(Odometry, 'odom',10)
		
		self.clock_sub = self.create_subscription(Clock, 'clock', self.clock_callback,10)
		self.scan_sub = self.create_subscription(LaserScan, 'scan/raw', self.scan_callback,qos_profile=qos_profile_sensor_data)
		self.scan_pub = self.create_publisher(LaserScan, 'scan',10)
		#TF
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		
		self.time = self.get_clock().now()
		
		self.scan_msg = LaserScan()
		self.get_logger().info("ready to publish transform")
		
	def scan_callback(self,sc):
		self.scan_msg.header.frame_id = 'laser_frame'
		self.scan_msg.header.stamp = self.get_clock().now().to_msg()
		self.scan_msg.angle_min = sc.angle_min
		self.scan_msg.angle_max = sc.angle_max
		self.scan_msg.angle_increment = sc.angle_increment
		self.scan_msg.range_min = sc.range_min
		self.scan_msg.range_max = sc.range_max
		self.scan_msg.ranges = sc.ranges
		self.scan_msg.intensities = sc.intensities
		
		self.scan_pub.publish(self.scan_msg)
		
	def clock_callback(self,t):
		self.time = t.clock
		
	
	def odom_callback(self,odom):
    		ts = self.get_clock().now()
    		
    		#publish odometry
	    	od = Odometry()
	    	od.header.stamp = self.get_clock().now().to_msg()
	    	od.header.frame_id = "odom"
	    	od.child_frame_id = "base_link"
	    	#Add pose to odom
	    	od.pose.pose.position.x = odom.pose.pose.position.x
	    	od.pose.pose.position.y = odom.pose.pose.position.y
	    	od.pose.pose.position.z = 0.0
	    	od.pose.pose.orientation.x = odom.pose.pose.orientation.x
	    	od.pose.pose.orientation.y = odom.pose.pose.orientation.y
	    	od.pose.pose.orientation.z = odom.pose.pose.orientation.z
	    	od.pose.pose.orientation.w = odom.pose.pose.orientation.w 
	    	
	    	
	    	self.odom_pub.publish(od)
	    	
	    	#broadcast transform
	    	t = TransformStamped()
	    	t.header.frame_id = "odom"
	    	t.child_frame_id= "base_link"
	    	t.header.stamp = self.get_clock().now().to_msg()
	    	t.transform.translation.x = odom.pose.pose.position.x
	    	t.transform.translation.y = odom.pose.pose.position.y
	    	t.transform.translation.z = 0.0
	    	t.transform.rotation.x = odom.pose.pose.orientation.x
	    	t.transform.rotation.y = odom.pose.pose.orientation.y
	    	t.transform.rotation.z = odom.pose.pose.orientation.z
	    	t.transform.rotation.w = odom.pose.pose.orientation.w
	    	self.tf_broadcaster.sendTransform(t)
	    	
	    	
	    	self.last_time = ts
	    	
	    	#Debug Zone
	    	#self.get_logger().info('rtick: "%s"' %tick["l"])
	    	#self.get_logger().info('prev_tick: "%s"' %self.left_tick["bf"])
	    	#self.get_logger().info('DR: "%s"' %DR)
	    	#self.get_logger().info('DL: "%s"' %DL)
	    	#self.get_logger().info('DC: "%s"' %DC)
	    	#self.get_logger().info('prev pose: "%s"' %prev_robot_pose)
	    	#self.get_logger().info('dx: "%s"' %self.dx)
	    	#self.get_logger().info('dy: "%s"' %self.dy)
	    	#self.get_logger().info('dth: "%s"' %self.dtheta)
		
def main():
	rclpy.init()
	
	rb = Robot()
	rclpy.spin(rb)
	
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()
