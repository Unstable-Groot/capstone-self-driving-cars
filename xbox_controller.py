#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading
import curses

stdscr = curses.initscr()

# Throttle should be bounded between [-20, +20]
MAX_MANUAL_THROTTLE_FORWARD = 22
MAX_MANUAL_THROTTLE_REVERSE = 65

# Steering should be bounded between [-100, +100]

steering = 0
throttle = 0
toggle = 0

def joy_callback(data):
	global steering, throttle, toggle

	steering = data.axes[2] * 70 * -1 + 6
	throttle = data.axes[1] * 100

	if throttle < -MAX_MANUAL_THROTTLE_REVERSE:
		throttle = -MAX_MANUAL_THROTTLE_REVERSE

	elif throttle > MAX_MANUAL_THROTTLE_FORWARD:
		throttle = MAX_MANUAL_THROTTLE_FORWARD

	if data.buttons[0] == 1:
		toggle = 1
	else:
		toggle = 0

	## ------------
	## YOUR CODE
	## ------------

def main(args=None):

	rclpy.init(args=args)
	node = Node("xbox_controller_node")

	## ------------
	## YOUR CODE
	# Subscribe to the 'joy' topic
	# Create publishers for the manual_throttle and manual_steering commands
	## ------------

	sub = node.create_subscription(Joy, "/joy", joy_callback, 1)

	man_throttle = node.create_publisher(Int64, "/manual_throttle",1)
	man_steering = node.create_publisher(Int64, "/manual_steering",1)
	a_toggle = node.create_publisher(Int64, "/manual_a_toggle",1)
	

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
        
	while rclpy.ok():

		try:
			## ------------
			## YOUR CODE
			# Publish actuation commands
			## ------------
			wrapped_throttle = Int64()
			wrapped_steering = Int64()
			wrapped_toggle = Int64()
			wrapped_throttle.data = int (throttle)
			wrapped_steering.data = int (steering)
			wrapped_toggle.data = int (toggle)

			man_throttle.publish(wrapped_throttle)
			man_steering.publish(wrapped_steering)
			a_toggle.publish(wrapped_toggle)

			stdscr.refresh()
			stdscr.addstr(1, 25, 'Xbox Controller       ')
			stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
			stdscr.addstr(3, 25, 'Steering: %.2f  ' % steering)
			stdscr.addstr(4, 25, 'Toggle: %d' % toggle)

			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()

# 10.138.194.207
