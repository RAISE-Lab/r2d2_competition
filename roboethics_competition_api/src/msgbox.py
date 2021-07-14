#!/usr/bin/env python
import rospy
rospy.init_node("msgbox",anonymous=True)
from std_msgs.msg import String

def printer(msg):
    print(msg.data)

sub = rospy.Subscriber("/msg",String,printer)
rospy.spin()

