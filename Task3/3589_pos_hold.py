#!/usr/bin/env python

'''
* Team Id : 3589
* Author List : Rishabh Tyagi, Madhav Sharma, Apoorv Kotnala, Bharat Jain
* Filename: task_3_pos_hold.py
* Theme: Hungry Birds
* Functions: __init__(), disarm(), arm(), whycon_callback(), altitude_set_pid(), pitch_set_pid(), roll_set_pid(), yaw_set_pid(), yaw_set(), crash(), crash_avoid(), pid()
* Global Variables: None
'''

# Importing the required libraries
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import cv2


class Edrone():
	'''
	* Function Name: __init__
	* Input: self
	* Output: None
	* Logic: This is called when an object is created from class. It is the constructor method for class. It is used to initialize the attributes of a class. Variables are defined here.
	* Example Call:
	'''
	def __init__(self):
		
		# initializing ros node with name drone_control
		rospy.init_node('drone_control')	
		
		# drone_position: This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# 				  [x,y,z,yaw_value]
		self.drone_position = [-5.0,4.0,0.0,0.0]	

		# setpoint: [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		# self.setpoint = [2.0,-2.0,23.0,0.0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [0.0,0.0,20.0,0.0]

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX1 = 0
		self.cmd.rcAUX2 = 0
		self.cmd.rcAUX3 = 0
		self.cmd.rcAUX4 = 0
		self.cmd.plutoIndex = 0
		
		# pitch, roll, throttle, yaw
		# self.Kp = [251*0.06,256*0.06,310*0.06,220*0.06]
		# self.Ki = [0,0,0,1]
		# self.Kd = [660*0.3,640*0.3,1218*0.3,650*0.3]

		self.Kp = [200*0.06,200*0.06,3300*0.06,0]
		self.Ki = [0,0,0,0]
		self.Kd = [705*0.3,705*0.3,2500*0.3,0]

		# self.Kp = [0,0,3500*0.06,0]
		# self.Ki = [0,0,0,0]		
		# self.Kd = [0,0,1800*0.3,0]

		# error: It stores the error values in each axis
		self.error = [0,0,0,0]
		# previous_error: 
		self.previous_error = [0,0,0,0]

		# error_sum: 
		self.error_sum = [0,0,0,0]
		
		# pid_output: Variable for pid output required for each axis 
		self.pid_output = [0,0,0,0]

		# p_value:
		self.p_value = [0,0,0,0]
		# i_value: 
		self.i_value = [0,0,0,0]
		# d_value:
		self.d_value = [0,0,0,0]

		# max_values: stores maximum range
		self.max_values = [1800,1800,1800,1800]
		# min_values: stores minimum range
		self.min_values = [1200,1200,1200,1200]


		# Boundary Values 
		# [positive axis, negative axis]
		range_of_x = [14,-11] 
		range_of_y = [9,-8]
		range_of_z = [35,15]


		# sample_time: This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds
		# self.sample_time = 0.01

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line_pub = rospy.Publisher('/zero_line', Int16, queue_size=1)


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)		
		rospy.Subscriber('/pid_tuning_pitch', PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw', PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw', Float64, self.yaw_set)

		# ARMING THE DRONE
		self.arm() 

	'''
	* Function Name: disarm
	* Input: self
	* Output: Publishes self.cmd on the topic /drone_command
	* Logic: It assigns the value of throttle and AUX4 which will disarms the drone
	* Example Call: e_drone.disarm()
	'''
	def disarm(self):
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1000
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
	
	'''
	* Function Name: arm
	* Input: self
	* Output: Publihes self.cmd on the topic /drone_command
	* Logic: It assigns values of roll, pitch ,throttle and AUX4 such that the drone arms
	* Example Call: e_drone.arm()
	'''
	def arm(self):
		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		print("armed")
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)

	'''
	* Function Name: whycon_callback
	* Input: self, msg
	* Output: None
	* Logic: The function gets executed each time when /whycon node publishes /whycon/poses 
	* Example Call: e_drone.whycon_callback(msg)
	'''
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	'''
	* Function Name: altitude_set_pid
	* Input: self, alt
	* Output: None
	* Logic: This function sets pid values from pid_tunning package for altitude which allows to set different values of kp,kd and ki values.
	* Example Call: e_drone.altitude_set_pid(msg)
	'''
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	'''
	* Function Name: pitch_set_pid
	* Input: self, msg
	* Output: None
	* Logic: This function sets pid values from pid_tunning package for pitch which allows to set different values of kp,kd and ki values.
	* Example Call: e_drone.pitch_set_pid(msg)
	'''
	def pitch_set_pid(self, msg):
		self.Kp[0] = msg.Kp * 0.06
		self.Ki[0] = msg.Ki * 0.008
		self.Kd[0] = msg.Kd * 0.3

	'''
	* Function Name: roll_set_pid
	* Input: self, msg
	* Output: None
	* Logic: This function sets pid values from pid_tunning package for roll which allows to set different values of kp,kd and ki values.
	* Example Call: e_drone.roll_set_pid(msg)
	'''
	def roll_set_pid(self, msg):
		self.Kp[1] = msg.Kp * 0.06
		self.Ki[1] = msg.Ki * 0.008
		self.Kd[1] = msg.Kd * 0.3

	'''
	* Function Name: yaw_set_pid
	* Input: self, msg
	* Output: None
	* Logic: This function sets pid values from pid_tunning package for yaw which allows to set different values of kp,kd and ki values
	* Example Call: e_drone.yaw_set_pid(msg)
	'''
	def yaw_set_pid(self, msg):
		self.Kp[3] = msg.Kp * 0.06
		self.Ki[3] = msg.Ki * 0.008
		self.Kd[3] = msg.Kd * 0.3

	'''
	* Function Name: yaw_set
	* Input: self, msg
	* Output: None
	* Logic: It sets yaw value from the message it receives
	* Example Call: e_drone.yaw_set()
	'''
	def yaw_set(self, msg):
		self.drone_position[3] = msg.data

	'''
	* Function Name: crash
	* Input: self
	* Output: Returns crash_index 
	* Logic: This function calculates crash_index and determines if the drone is within boundary values.
	* Example Call:
	'''
	def crash(self):

		# crash_index: It signifies if the drone is out of boundary or not. It is 0 by default i.e. not crashing. 
		crash_index = 0 

		if self.drone_position[0] < range_of_x[0] and self.drone_position[0] > range_of_x[1]:
			if self.drone_position[1] < range_of_y[0] and self.drone_position[1] > range_of_y[1]:
				if self.drone_position[2] < range_of_z[0] and self.drone_position[2] > range_of_z[1]:
					crash_index = 0
				else:
					crash_index = 1
			else:
				crash_index = 1
		else:
			crash_index = 1

		return crash_index

	'''
	* Function Name: crash
	* Input: self
	* Output: Publish self.cmd
	* Logic: This function assigns value to rcPitch, rcRoll, rcThrottle by checking the exceeded boundary values in x,y and z axis
	* Example Call: e_drone.crash_avoid()
	'''
	def crash_avoid(self):
		self.rcThrottle -= 10
		if self.drone_position[1] > range_of_y[0]:
			self.cmd.rcRoll -= 10
			if self.drone_position[0] > range_of_x[0]:
				self.cmd.rcPitch -= 10
			elif self.drone_position[0] < range_of_x[1]:
				self.cmd.rcPitch += 10
		elif self.drone_position[1] < range_of_y[1]:
			self.cmd.rcRoll += 10
			if self.drone_position[0] > range_of_x[0]:
				self.cmd.rcPitch -= 10
			elif self.drone_position[0] < range_of_x[1]:
				self.cmd.rcPitch += 10
		else:
			if self.drone_position[0] > range_of_x[0]:
				self.cmd.rcPitch -= 10
			elif self.drone_position[0] < range_of_x[1]:
				self.cmd.rcPitch += 10
		if self.drone_position[2] > 25:
			self.disarm() 

		self.command_pub.publish(self.cmd)


	'''
	* Function Name: pid
	* Input: self
	* Output: Publishes self.cmd and self.error
	* Logic: This functions check crash_index. If it is 0 then pid() will run, which calculates self.pid_output using self.error and self.KP, self.Kd, and self.Ki values.
			 If the crash_index is 1 then crash_avoid function will run.
	* Example Call: e_drone.pid()
	'''
	def pid(self):
		# if self.crash() == 1:

		for ix in range(len(self.error)):

			# Computing error in each axis
			self.error[ix] = self.drone_position[ix] - self.setpoint[ix]

			# Computing proportional term in error
			self.p_value[ix] = self.Kp[ix] * self.error[ix]
			# Computing derivative term in error
			self.d_value[ix] = self.Kd[ix] * (self.error[ix] - self.previous_error[ix])
			# Computing integral term in error
			self.i_value[ix] = (self.error_sum[ix]) * self.Ki[ix]

			# Calculating pid output for each axis
			self.pid_output[ix] = self.p_value[ix] + self.d_value[ix] + self.i_value[ix]
			
			
		self.out_pitch, self.out_roll, self.out_throttle, self.out_yaw = self.pid_output

		self.cmd.rcPitch = 1500 - self.out_pitch
		self.cmd.rcRoll = 1500 - self.out_roll
		self.cmd.rcThrottle = 1500 + self.out_throttle
		self.cmd.rcYaw = 1500 - self.out_yaw
		
		# maximum and minimum condition
		if self.cmd.rcPitch > self.max_values[0]:
			self.cmd.rcPitch = self.max_values[0] 	

		elif self.cmd.rcPitch < self.min_values[0]:
			self.cmd.rcPitch = self.min_values[0] 	

		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]

		elif self.cmd.rcRoll < self.min_values[1]:
			self.cmd.rcRoll = self.min_values[1]

		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]

		elif self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]

		if self.cmd.rcYaw > self.max_values[3]:
			self.cmd.rcYaw = self.max_values[3]
		
		elif self.cmd.rcYaw < self.min_values[3]:
			self.cmd.rcYaw = self.min_values[3]
		

		for ix in range(len(self.error)):
			# Updating previous_error
			self.previous_error[ix] = self.error[ix]
			# Add error_sum
			self.error_sum[ix] = self.error_sum[ix] + self.error[ix]

		rospy.sleep(self.sample_time)

		
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[0]) 
		self.roll_error_pub.publish(self.error[1])
		self.yaw_error_pub.publish(self.error[3]) 
		self.zero_line_pub.publish(0)

		# elif self.crash() == 1:
		# 	self.crash_avoid
		




if __name__ == '__main__':

	e_drone = Edrone()
	e_drone.arm()
	now = time.time()
	timer = 0  
	while timer != 65:		
		e_drone.pid()
		end = time.time()
		timer = round(end-now)
	# SHUTTING DOWN DRONE	
	# e_drone.setpoint = [0,0,25,0]
	# while e_drone.drone_position[2] < 25:
	# 	e_drone.pid()
	# while not rospy.is_shutdown():
	# 	e_drone.pid()
	e_drone.disarm()
