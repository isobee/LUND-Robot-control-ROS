#! /usr/bin/env python
import rospy, math
import numpy as np
import time, threading
import sys, termios, tty, select, os, threading
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16


def controlStateToString(x):
  return {
    0:'UNDEFINED',
    1:'MANUAL',
    2:'PARK',
  }.get(x,'UNKNONW_VALUE')



#define	AM_STATE_UNDEFINED     0x0
#define	AM_STATE_MANUAL        0x1
#define	AM_STATE_PARK          0x2

class Driver(object):
	move_cmd_bindings = {'q':np.array([1,1]),
				'w':np.array([1,0]),
				'e':np.array([1,-1]),
				'a':np.array([0,1]),
				'd':np.array([0,-1]),
				'z':np.array([-1,-1]),
				'x':np.array([-1,0]),
				'c':np.array([-1,1]),
				's':np.array([0,0]),
				'\x1b[A':np.array([1,0]),
				'\x1b[B':np.array([-1,0]),
				'\x1b[C':np.array([0,-1]),
				'\x1b[D':np.array([0,1]),
				}

	speed_cmd_bindings = { 't':np.array([1,1]),
					'b':np.array([-1,-1]),
					'y':np.array([1,0]),
					'n':np.array([-1,0]),
					'u':np.array([0,1]),
					'm':np.array([0,-1]),
					}

	def init(self):
		#save terminal settings		
		self.settings = termios.tcgetattr(sys.stdin)


		# set initial values
		self.inc_ratio = 0.1
		self.speed = np.array([0.3, 1.0])
		self.command = np.array([0,0])
		self.update_rate = 10
		self.statuses = []
		self.prev_time = time.time()

		#Set publisher topics
		self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)

		#Used for printing
		self.last_terminalWidth = 0

	def fini(self):
		# Restore terminal settings
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

# Actually running the robot
	def run(self):
		try:
			self.init()
			#Enter manual mode
			mode = UInt16()
			mode.data = 0x90
			self.pub_mode.publish(mode)
			self.command = np.array([0, 0])

			#print info
			self.print_usage()
			self.show_status()			
			r = rospy.Rate(self.update_rate)

			while not rospy.is_shutdown():
				ch = self.get_key()
				self.process_key(ch)
				self.update()
				#if enough time has passed, print a new status
				if (abs(self.prev_time - time.time()) > 0.5) and (ch != 'g'):
					self.prev_time = time.time()
					self.show_status()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()

# Functions for getting and processing the keys
	def get_key(self):
		tty.setraw(sys.stdin.fileno())		
		#check if there is data ready to be read in stdin (i.e. if a key has been pressed) 
		#The int is the amount of time it will check for		
		rlist, _, _ = select.select([sys.stdin], [], [], 1)
		if rlist:			
			key = sys.stdin.read(1)
			#this is what allows the arrow keys to be taken in, as their escape codes are three chars		
			if key == '\x1b':		
				key = key + sys.stdin.read(2)
				return key 
			else:
				return key.lower()
		else:
			return ''


	def process_key(self, ch):
		# AM_DRIVER COMMANDS	
		if ch in self.move_cmd_bindings.keys():
			self.command = self.move_cmd_bindings[ch]
		elif ch in self.speed_cmd_bindings.keys():
			self.speed = self.speed * (1 + self.speed_cmd_bindings[ch]*self.inc_ratio)
		elif ch == 'g':
			print('\n Quitting \r')
			# Stop the robot
			twist = Twist()
			self.pub_twist.publish(twist)

			# Stop following loop!
			mode = UInt16()
			mode.data = 0x17
			self.pub_mode.publish(mode)
			rospy.signal_shutdown('Shutdown')
		else:
			self.command = np.array([0, 0])

	def update(self):
		if rospy.is_shutdown():
			return
		twist = Twist()
		cmd = self.speed * self.command
		twist.linear.x = cmd[0]
		twist.angular.z = cmd[1]
		self.pub_twist.publish(twist)


# Printing

# Used to print instructions
	def print_usage(self):
		msg = """
		HRP Teleop keys that Publish to /cmd_vel /cmd_mode
		-------------------------------------------------------
		Moving around:     Adjust Speed:    Arrow Keys:
		Q   W   E          T  Y  U               ^
		A   S   D	                       <- ->
		Z   X   C          B  N  M		 

		G :   Quit
		--------------------------------------------------------
		"""
		print (msg)

# Used to print teleop status
	def show_status(self):
		msg = 'Status:\tlinear speed %.2f\tangular speed %.2f\tlinear direction %.2f\tangular direction %.2f'% (self.speed[0],self.speed[1], self.command[0], self.command[1])	
		if len(self.statuses) > 9:
			self.statuses = self.statuses[1:]
			
			#this should move the cursor up the same number of lines that there are statuses 
			sys.stdout.write(u"\u001b["+ str(len(self.statuses)+1) + "A" + "\r")	
		else: 
			#requires one less to print properly if the maximum length of statuses isn't reached
			sys.stdout.write(u"\u001b["+ str(len(self.statuses)) + "A" + "\r")

		self.statuses.append(msg)
		for status in self.statuses:				
			print(status + "\r")



if __name__ == '__main__':
	rospy.init_node('HRP_keyboard_teleop')
	teleop = Driver()
	teleop.run()
