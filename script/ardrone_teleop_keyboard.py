#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------

x : stay stationary
z/d : move forward/backward
q/d : rotate left/right
o/l : move up/down

t/g : takeoff/land

y/h : increase/decrease max speeds by 10
u/j : increase/decrease only linear speed by 10%
i/k : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {

	'z':(0.2,0,0),
	's':(-0.2,0,0),
	'x':(0,0,0),
	'q':(0,1,0),
	'd':(0,-1,0),
	'o':(0,0,1),
	'l':(0,0,-1),
}

actionBindings = {
	't':(1),
	'g':(0),
}

speedBindings={
	'y':(1.1,1.1),
	'h':(.9,.9),
	'u':(1.1,1),
	'j':(.9,1),
	'i':(1,1.1),
	'k':(1,.9),
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = 0.5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size = 10)
	pub_land = rospy.Publisher('ardrone/land', Empty, queue_size = 10)
	rospy.init_node('teleop_keyboard')
	r = rospy.Rate(10) # 10hz

	x = 0
	y = 0
	z = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while (1):
			
			key = getKey()
			if key in actionBindings.keys():
				if(actionBindings[key] == 1):

					rospy.loginfo("Take off")
					pub_takeoff.publish() # std_msgs/Empty
					r.sleep()
					continue
					
				else:

					rospy.loginfo("Land")
					pub_land.publish() # std_msgs/Empty
					r.sleep()
					continue

			elif key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				rospy.loginfo("Move")

			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				rospy.loginfo("Change settings")

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				rospy.loginfo("Stay stationary")
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = z*speed
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = y*turn
			pub.publish(twist)
			r.sleep()			

	except rospy.ROSInterruptException as e:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)
		r.sleep()

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


