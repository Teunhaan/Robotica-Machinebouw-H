#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

def callback(data):
    pub = rospy.Publisher('/avans/leds/state', UInt8,  queue_size=1)
    value = data.data
    if value == "Start":
        # Gelezen waarde is Start upload 1000 
        rospy.loginfo("Start signaal ontvangen")
        pub.publish(1)
    elif value == "Start Single":
        # Gelezen waarde is Start Single upload 0100
        rospy.loginfo("Start Single signaal ontvangen")
        pub.publish(2)
    elif value == "Stop":
        # Gelezen waarde is Stop upload 0010
        rospy.loginfo("Stop signaal ontvangen")
        pub.publish(4)
    elif value == "Noodstop":
        # Gelezen waarde is Noodstop upload 0001
        rospy.loginfo("Noodstop signaal ontvangen")
        pub.publish(8)
    else:
        rospy.logwarn("Received unexpected value: %s", value)

def listener():
    # Start ros node
    rospy.init_node('Lampen_listener')
    
    # Subscriber op '/Signaal' met inhoud string
    rospy.Subscriber('/Signaal', String, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
