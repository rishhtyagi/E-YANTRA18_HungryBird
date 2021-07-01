#!/usr/bin/env python

'''
* Team Id : 3589
* Author List : Rishabh Tyagi, Madhav Sharma, Apoorv Kotnala, Bharat Jain
* Filename: 3589_progress_task.py
* Theme: Hungry Birds
* Functions: __init__(), disarm(), arm(), whycon_callback(), extract_waypoints(), initial_pid(), calc_error(),
			 activation_key(), crash(), crash_avoid(), pid(), drone_shutdown()
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
		self.drone_position = [0.0, 0.0, 0.0, 0.0]	

		# setpoint: [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		# self.setpoint = [2.0,-2.0,23.0,0.0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [-4.088, -1.008, 19.757, 0]

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

		# key_value: stores input key value
		self.key_value = None

		# Boundary Values 
		# [positive axis, negative axis]
		range_of_x = [14,-11] 
		range_of_y = [9,-8]
		range_of_z = [35,15]

		# compute_path_flag
		self.compute_path_flag = 0

		# sample_time: This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds
		# self.sample_time = 0.01

		self.initial_waypoint = [0,0,0,0]

		self.num_of_whycon = 2

		self.initialized = 1

		self.waypoints_flag = 0

		self.waypoints=[]

		self.new_msg = []
		for i in range(self.num_of_whycon):
			self.new_msg.append(0)

		self.stable_point = [0,0,0,0]

		self.test = 0

		self.secure_initial_waypoint = [0,0,0,0]

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.path_flag_pub = rospy.Publisher('/path_flag', Int16, queue_size=1)

		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/vrep/waypoints', PoseArray, self.extract_waypoints)
		rospy.Subscriber('/input_key', Int16, self.activation_key)		
		#rospy.Subscriber('/drone_yaw', Float64, self.yaw_set)

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
	* Function Name: Initial_Whycon_Assignment
	* Input: self, msg
	* Output: None
	* Logic: Stores intialize whycon values 
	* Example Call: Initial_Whycon_Assignment(msg)
	'''
	# def Initial_Whycon_Assignment(self,msg):
	# 	j = 1
	# 	for i in range(self.num_of_whycon):
	# 		if msg.poses[i].position.z >= 29:
	# 			self.new_msg[0] = msg.poses[i].position
	# 		else:
	# 			self.new_msg[j] = msg.poses[i].position
	# 			j += 1


	'''
	* Function Name: Check_Whycon_Inidices
	* Input: self, msg
	* Output: None
	* Logic: Check whycon indices finds drone whycon index and update new_msg values 
	* Example Call: Check_Whycon_Inidices(msg)
	'''
	# def Check_Whycon_Inidices(self,msg):
	# 	idx = []
	# 	for k in range(self.num_of_whycon):
	# 		idx.append(k)
	# 	for i in range(self.num_of_whycon):
	# 		for j in range(1,len(self.new_msg)):
	# 			if (msg.poses[i].position.x <= self.new_msg[j].x + 0.1) and (msg.poses[i].position.x >= self.new_msg[j].x- 0.1) and (msg.poses[i].position.y <= self.new_msg[j].y + 0.1) and (msg.poses[i].position.y >= self.new_msg[j].y- 0.1):
	# 				self.new_msg[j] = msg.poses[i].position
	# 				idx.remove(i)
	# 		if len(idx) == 1:
	# 			break
	# 	droneidxval = idx[0]		
	# 	self.new_msg[0] = msg.poses[droneidxval].position		

	'''
	* Function Name: whycon_callback
	* Input: self, msg
	* Output: None
	* Logic: The function gets executed each time when /whycon node publishes /whycon/poses 
	* Example Call: e_drone.whycon_callback(msg)
	'''
	def whycon_callback(self,msg):
		if self.key_value == 600:
			# self.Initial_Whycon_Assignment(msg)
			self.initial_waypoint = [msg.poses[0].position.x, msg.poses[0].position.y, 18.656, 0]
			self.secure_initial_waypoint = [msg.poses[0].position.x, msg.poses[0].position.y, 18.656, 0]
			# self.initialized = 0 
			print "initial_waypoint",self.initial_waypoint
		# self.Check_Whycon_Inidices(msg)
		# self.drone_position[0] = self.new_msg[0].x #msg.poses[1].position.x
		# self.drone_position[1] = self.new_msg[0].y #msg.poses[1].position.y
		# self.drone_position[2] = self.new_msg[0].z #msg.poses[1].position.z
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	'''
	* Function Name: extract_waypoints
	* Input: self, msg
	* Output: None
	* Logic: It is a Function to store waypoints obtained from message to list waypoints[] like this- [[x1,y1,z1], [x2,y2,z2], ...]
	* Example Call: e_drone.extract_waypoints(msg)
	'''
	def extract_waypoints(self, msg):	
		# print len(msg.poses)
		for ix in range(len(msg.poses)):
			coords = [msg.poses[ix].position.x ,msg.poses[ix].position.y, msg.poses[ix].position.z]
			self.waypoints.append(coords)
		self.waypoints_flag = 1	
		#print "Waypoints Recieved, length = ",len(self.waypoints)
	

	'''
	* Function Name: pid
	* Input: self
	* Output: None
	* Logic: It is a Function to make drone reach the initial_waypoint
	* Example Call: e_drone.initial_pid()
	'''
	def initial_pid(self):
		#self.setpoint = [msg.poses[0].position.x, msg.poses[0].position.y, 21.723]
	    #self.setpoint = [-4.088, -1.008, 19.757, 0]
	    self.setpoint = self.initial_waypoint
	    now = rospy.get_time() #time.time()
	    timer = 0
	    while timer != 8:
	    	self.pid()
	    	end = rospy.get_time() #time.time()
	    	timer = round(end-now) 
	    return 0 		


	'''
	* Function Name: pid
	* Input: self
	* Output: None
	* Logic: It is a Function to calculate error in each axis at the instant and Updating its value
	* Example Call: e_drone.calc_error()
	'''
	def calc_error(self):
		for ix in range(len(self.error)):
			# Computing error in each axis
			self.error[ix] = self.drone_position[ix] - self.setpoint[ix]


	'''
	* Function Name: activation_key
	* Input: self, msg
	* Output: returns self.key_value
	* Logic: This function sets value of key_value to the value of input key 
	* Example Call: e_drone.activation_key(msg)
	'''		
	def activation_key(self,msg):
		self.key_value = msg.data
		if self.key_value == 800:
			self.drone_shutdown()
		return self.key_value		
		
	'''
	* Function Name: crash
	* Input: self
	* Output: Returns crash_index 
	* Logic: This function calculates crash_index and determines if the drone is within boundary values.
	* Example Call:
	'''
	# def crash(self):

	# 	# crash_index: It signifies if the drone is out of boundary or not. It is 0 by default i.e. not crashing. 
	# 	crash_index = 0 
	# 	# Checking if crash_index is 1 or 0
	# 	if self.drone_position[0] < range_of_x[0] and self.drone_position[0] > range_of_x[1]:
	# 		if self.drone_position[1] < range_of_y[0] and self.drone_position[1] > range_of_y[1]:
	# 			if self.drone_position[2] < range_of_z[0] and self.drone_position[2] > range_of_z[1]:
	# 				crash_index = 0
	# 			else:
	# 				crash_index = 1
	# 		else:
	# 			crash_index = 1
	# 	else:
	# 		crash_index = 1

	# 	return crash_index


	'''
	* Function Name: crash
	* Input: self
	* Output: Publish self.cmd
	* Logic: This function assigns value to rcPitch, rcRoll, rcThrottle by checking the exceeded boundary values in x,y and z axis
	* Example Call: e_drone.crash_avoid()
	'''
	# def crash_avoid(self):
	# 	self.rcThrottle -= 10
	# 	if self.drone_position[1] > range_of_y[0]:
	# 		self.cmd.rcRoll -= 10
	# 		if self.drone_position[0] > range_of_x[0]:
	# 			self.cmd.rcPitch -= 10
	# 		elif self.drone_position[0] < range_of_x[1]:
	# 			self.cmd.rcPitch += 10
	# 	elif self.drone_position[1] < range_of_y[1]:
	# 		self.cmd.rcRoll += 10
	# 		if self.drone_position[0] > range_of_x[0]:
	# 			self.cmd.rcPitch -= 10
	# 		elif self.drone_position[0] < range_of_x[1]:
	# 			self.cmd.rcPitch += 10
	# 	else:
	# 		if self.drone_position[0] > range_of_x[0]:
	# 			self.cmd.rcPitch -= 10
	# 		elif self.drone_position[0] < range_of_x[1]:
	# 			self.cmd.rcPitch += 10

	# 	self.command_pub.publish(self.cmd)	


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
		
		# maximum and minimum condition for Pitch
		if self.cmd.rcPitch > self.max_values[0]:
			self.cmd.rcPitch = self.max_values[0] 	

		elif self.cmd.rcPitch < self.min_values[0]:
			self.cmd.rcPitch = self.min_values[0] 	

		# maximum and minimum condition for Roll	
		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]

		elif self.cmd.rcRoll < self.min_values[1]:
			self.cmd.rcRoll = self.min_values[1]

		# maximum and minimum condition for Throttle	
		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]

		elif self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]

		# maximum and minimum condition for Yaw
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

		# elif self.crash() == 1:
		# 	self.crash_avoid


	'''
	* Function Name: drone_shutdown
	* Input: self
	* Output: Publishes self.cmd 
	* Logic: This function is used to shut down drone by reducing throttle and thus reducing height of flight
			 so it comes to a safe height and then disarm it.
	* Example Call: e_drone.drone_shutdown()
	'''	
	def drone_shutdown(self):
		# Reducing the Drone height of flight slowly so it does not crash into the ground 	
		while e_drone.drone_position[2] < 30:
			if self.cmd.rcThrottle <= 1200:
				self.cmd.rcThrottle = 1200
			self.cmd.rcThrottle -= 5
			self.cmd.rcPitch = 1500
			self.cmd.rcRoll = 1500
			self.cmd.rcYaw = 1500
			self.cmd.rcAUX4 = 1500
			self.command_pub.publish(e_drone.cmd)
		# Disarming the drone after it comes to safe height	
		self.disarm()		




if __name__ == '__main__':

	# Initialization of Edrone() class
	e_drone = Edrone()
	while True:
		if e_drone.key_value == 900:
			e_drone.key_value = None
			# Arming the Drone
			e_drone.arm()
			e_drone.arm()
			# Using intial_pid() function to make the drone reach initial_waypoint
			e_drone.initial_pid()
			# Path Planning and running of drone
			while not rospy.is_shutdown():
			#while True:
				# Publishing value of compute_path_flag on the topic '/path_flag'
				e_drone.path_flag_pub.publish(e_drone.compute_path_flag)
				

				while(e_drone.waypoints_flag!=1):
				# for k2 in range(15):
					e_drone.pid()
			
				if e_drone.compute_path_flag == 1:
					e_drone.setpoint[0] = e_drone.waypoints[0][0]
					e_drone.setpoint[1] = e_drone.waypoints[0][1]
					e_drone.setpoint[2] = e_drone.waypoints[0][2]
					while(abs(e_drone.error[0])>=0.7 or abs(e_drone.error[1])>=0.7 or abs(e_drone.error[2])>=7):
						# For moving the drone towards waypoint
						e_drone.pid()
						# Updating Error value
						e_drone.calc_error()

					for k in range(50):
						e_drone.pid()	


				for ix in range(len(e_drone.waypoints)):
					# print ("Waypoint no-", ix)
					# Setting waypoints as Setpoint(destination) for drone
					e_drone.setpoint[0] = e_drone.waypoints[ix][0]
					e_drone.setpoint[1] = e_drone.waypoints[ix][1]
					e_drone.setpoint[2] = e_drone.waypoints[ix][2]
					e_drone.calc_error()
					while(abs(e_drone.error[0])>=0.7 or abs(e_drone.error[1])>=0.7 or abs(e_drone.error[2])>=7):
					#for k in range(20):
						# For moving the drone towards waypoint
						e_drone.pid()
						# Updating Error value
						e_drone.calc_error()

				
				# Incrementing compute_path_flag value after a target is reached to initiate next target's path planning
				e_drone.compute_path_flag = e_drone.compute_path_flag + 1
				e_drone.waypoints_flag = 0
				e_drone.waypoints = []
				# if e_drone.compute_path_flag<2:
				# 	print "Next target"
				
				# Condition for diarming the drone
				if e_drone.compute_path_flag>=2:

					# e_drone.initial_waypoint = e_drone.secure_initial_waypoint
					e_drone.setpoint = e_drone.secure_initial_waypoint
					print "-------"
					print e_drone.setpoint
					print e_drone.initial_waypoint
					e_drone.calc_error()
					while(abs(e_drone.error[0])>=0.7 or abs(e_drone.error[1])>=0.7 or abs(e_drone.error[2])>=7):
					#for k in range(50):
						# For moving the drone towards waypoint
						e_drone.pid()
						# Updating Error value
						e_drone.calc_error()
					#e_drone.setpoint = e_drone.initial_waypoint
					for k in range(50):
						# For moving the drone towards waypoint
						e_drone.pid()	
					# SHUTTING DOWN DRONE
					e_drone.drone_shutdown()
					e_drone.disarm()
					print "All targets reached"
					break			