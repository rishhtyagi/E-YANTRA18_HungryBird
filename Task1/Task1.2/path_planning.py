#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
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
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		# self.setpoint = [-5.61,1.89,33.40,0.0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [5.66,-1.91,33.40,0.0]	

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [0,0,0,0]
		# self.Ki = [0,0,0,0]
		# self.Kd = [0,0,0,0]
		
		# AFTER TUNING AND COMPUTING CHANGING PARAMETERS
		self.Kp = [251*0.06,256*0.06,310*0.06,220*0.06]
		self.Ki = [0,0,0,1]
		self.Kd = [660*0.3,640*0.3,1218*0.3,650*0.3]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		# Variables for error and previous error in each axis
		self.error = [0,0,0,0]
		self.previous_error = [0,0,0,0]

		# Variable for Sum of errors in each axis
		self.error_sum = [0,0,0,0]
		
		# Variable for pid output required for each axis 
		self.pid_output = [0,0,0,0]

		self.p_value = [0,0,0,0]
		self.i_value = [0,0,0,0]
		self.d_value = [0,0,0,0]

		# Maximum and Minimum ranges
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]

		self.waypoints = []
		self.compute_path_flag = 0


		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds 0.060





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line_pub = rospy.Publisher('/zero_line', Int16, queue_size=1)
		self.path_flag_pub = rospy.Publisher('/path_flag', Int16, queue_size=1)

		#-----------------------------------------------------------------------------------------------------------





		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		
		rospy.Subscriber('/pid_tuning_pitch', PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw', PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw', Float64, self.yaw_set)
		rospy.Subscriber('/vrep/waypoints',PoseArray, self.extract_waypoints)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		# print msg.poses[0].position.x,msg.poses[0].position.y,msg.poses[0].position.z

		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	def pitch_set_pid(self, msg):
		self.Kp[0] = msg.Kp * 0.06
		self.Ki[0] = msg.Ki * 0.008
		self.Kd[0] = msg.Kd * 0.3

	def roll_set_pid(self, msg):
		self.Kp[1] = msg.Kp * 0.06
		self.Ki[1] = msg.Ki * 0.008
		self.Kd[1] = msg.Kd * 0.3

	def yaw_set_pid(self, msg):
		self.Kp[3] = msg.Kp * 0.06
		self.Ki[3] = msg.Ki * 0.008
		self.Kd[3] = msg.Kd * 0.3

	def yaw_set(self, msg):
		self.drone_position[3] = msg.data

	# Function to store waypoints obtained from message to list waypoints[] like this- [[x1,y1,z1], [x2,y2,z2], ...] 
	def extract_waypoints(self, msg):
		self.waypoints=[]
		# print len(msg.poses)
		for ix in range(len(msg.poses)):
			coords = [msg.poses[ix].position.x ,msg.poses[ix].position.y, msg.poses[ix].position.z]
			self.waypoints.append(coords)
		#print "Waypoints Recieved, length = ",len(self.waypoints)


	# Function to make drone reach the initial_waypoint
	def initial_pid(self):
	    now=time.time()
	    timer = 0
	    while timer != 10:
		self.pid()
	        end = time.time()
	        timer = round(end-now)
	    return 0 		

	#Function to calculate error at the instant and Updating its value
	def calc_error(self):
		for ix in range(len(self.error)):
			# Computing error in each axis
			self.error[ix] = self.drone_position[ix] - self.setpoint[ix]

	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		# print "Inside PID()"

		self.calc_error()

		for ix in range(len(self.error)):

			# Computing error in each axis
			# self.error[ix] = self.drone_position[ix] - self.setpoint[ix]

			# Computing proportional term in error
			self.p_value[ix] = self.Kp[ix] * self.error[ix]
			# Computing derivative term in error
			self.d_value[ix] = self.Kd[ix] * (self.error[ix] - self.previous_error[ix])
			# Computing integral term in error
			self.i_value[ix] = (self.error_sum[ix]) * self.Ki[ix]

			# Calculating pid output for each axis
			self.pid_output[ix] = self.p_value[ix] + self.d_value[ix] + self.i_value[ix]
			
			
		self.out_pitch, self.out_roll, self.out_throttle, self.out_yaw = self.pid_output

		self.cmd.rcPitch = 1500 + self.out_pitch
		self.cmd.rcRoll = 1500 + self.out_roll
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



	#------------------------------------------------------------------------------------------------------------------------

		
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[0]) 
		self.roll_error_pub.publish(self.error[1])
		self.yaw_error_pub.publish(self.error[3]) 
		self.zero_line_pub.publish(0)




if __name__ == '__main__':

	e_drone = Edrone()

	# Using intial_pid() function to make the drone reach initial_waypoint
	e_drone.initial_pid()


	while not rospy.is_shutdown():
		# Publishing value of compute_path_flag on the topic '/path_flag'
		e_drone.path_flag_pub.publish(e_drone.compute_path_flag)
		for jx in range(10):
				e_drone.pid()

		for ix in range(len(e_drone.waypoints)):
			# print ("Waypoint no-", ix)
			# Setting waypoints as Setpoint(destination) for drone
			e_drone.setpoint[0] = e_drone.waypoints[ix][0]
			e_drone.setpoint[1] = e_drone.waypoints[ix][1]
			e_drone.setpoint[2] = e_drone.waypoints[ix][2]
			e_drone.calc_error()
			# Checking the condition if error is less than 0.5 in each axis
			while(abs(e_drone.error[0])>=0.5 or abs(e_drone.error[1])>=0.5 or abs(e_drone.error[2])>=0.5):
				# For moving the drone towards waypoint
				e_drone.pid()
				# Updating Error value
				e_drone.calc_error()

		if e_drone.compute_path_flag<3:
			print "Next target"
		
		e_drone.compute_path_flag += 1
		
		# Condition for diarming the drone
		if e_drone.compute_path_flag==3:
			e_drone.disarm()
			print "All targets reached"
			break
