#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import curses
import os

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('kill_switch', anonymous=True)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)

stdscr.refresh()

key = ''
while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_DC:
		em_pub.publish(True)
		stdscr.addstr(5, 20, "Emergency STOP!!!!!")
		# Kill all nodes
		nodes = os.popen("rosnode list").readlines()
		for i in range(len(nodes)):
		    nodes[i] = nodes[i].replace("\n","")
		for node in nodes:
		    print(node) 
		    os.system("rosnode kill "+ node)
	elif key == curses.KEY_HOME:
		em_pub.publish(False)
		stdscr.addstr(5, 20, "Normal Operation :)")

curses.endwin()
