#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os, threading
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16


class Driver(object):
	move_cmd_bindings = {'q':np.array([1,1]),
					'w':np.array([1,0]),
					'e':np.array([1,-1]),
					'a':np.array([0,1]),
					'd':np.array([0,-1]),
					'z':np.array([-1,-1]),
					'x':np.array([-1,0]),
					'c':np.array([-1,1]),
					's':np.array([0,0])
					}

	speed_cmd_bindings = { 't':np.array([1,1]),
					'b':np.array([-1,-1]),
					'y':np.array([1,0]),
					'n':np.array([-1,0]),
					'u':np.array([0,1]),
					'm':np.array([0,-1])
					}

	def init(self):
		#save terminal settings
		self.settings = termios.tcgetattr(sys.stdin)
		# set initial values
		self.inc_ratio = 0.1
		self.speed = np.array([0.3, 1.0])
		self.command = np.array([0,0])
		self.update_rate = 10

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
				self.show_status()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()

# Functions for getting and processing the keys
	def get_key(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		return key.lower()


	def process_key(self, ch):
		# AM_DRIVER COMMANDS
		if ch in self.move_cmd_bindings.keys():
			self.command = self.move_cmd_bindings[ch]
		elif ch in self.speed_cmd_bindings.keys():
			self.speed = self.speed * (1 + self.speed_cmd_bindings[ch]*self.inc_ratio)
		elif ch == 'g':
			self.loginfo('Quitting')
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
# Used to print items to screen, while terminal is in funky mode
	def loginfo(self, str):
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		print(str)
		tty.setraw(sys.stdin.fileno())

# Used to print instructions
	def print_usage(self):
		msg = """
		HRP Teleop that Publish to /cmd_vel /cmd_mode
		-------------------------------------------------------
		Moving around:     Adjust Speed:    
		Q   W   E          T  Y  U        
		A   S   D	                        
		Z   X   C          B  N  M

		G :   Quit
		--------------------------------------------------------
	
		"""
		self.loginfo(msg)

# Used to print teleop status
	def show_status(self):
		msg = 'Status:\tlinear %.2f\tangular %.2f' % (self.speed[0],self.speed[1])
		self.loginfo(msg)
		self.move_cursor_one_line_up()

	# Used to move cursor one line up, to enable printing status message repeatidly on the same line
	def move_cursor_one_line_up(self):
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		print(u"\u008D\r"), tty.setraw(sys.stdin.fileno())



if __name__ == '__main__':
	rospy.init_node('HRP_keyboard_teleop')
	teleop = Driver()
	teleop.run()
