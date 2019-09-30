#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
	# declares that your node is publishing to the chatter topic 
	# using the message type String. 
	# queue_size argument limits the amount of queued messages 
	# if any subscriber is not receiving them fast enough.
	pub = rospy.Publisher('chatter', String, queue_size=10)

	# initializer the node
	# it tells rospy the name of your node -- until rospy has this information, 
	# it cannot start communicating with the ROS Master. 
	# In this case, your node will take on the name talker. 
	# NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)  # 10Hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s " % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == "__main__":
	# this catches a rospy.ROSInterruptException exception, 
	# which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. 
	#The reason this exception is raised is so that you don't accidentally continue executing code after the sleep().
	try:
		talker()
	except rospy.ROSInterruptException:
		pass