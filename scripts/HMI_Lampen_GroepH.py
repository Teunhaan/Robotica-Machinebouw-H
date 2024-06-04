#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

def callback(data):
    pub = rospy.Publisher('/avans/leds/state', UInt8,  queue_size=1)
    value = data.data
    if value == "Start":
        # Gelezen waarde is 1 waarde Start wordt geupload naar topic /Signaal 
        rospy.loginfo("Start signaal ontvangen")
        pub.publish('1')
    elif value == "Start Single":
        # Gelezen waarde is 2 waarde Start Single wordt geupload naar topic /Signaal 
        rospy.loginfo("Start Single signaal ontvangen")
        pub.publish('2')
    elif value == "Stop":
        # Gelezen waarde is 4 waarde Stop wordt geupload naar topic /Signaal 
        rospy.loginfo("Stop signaal ontvangen")
        pub.publish('4')
    elif value == "Noodstop":
        # Gelezen waarde is 8 waarde Noodstop wordt geupload naar topic /Signaal 
        rospy.loginfo("Noodstop signaal ontvangen")
        pub.publish('7')
    else:
        rospy.logwarn("Received unexpected value: %d", value)

def listener():
    # Initialize the ROS node
    rospy.init_node('Lampen_listener')
    
    # Subscriber op '/avans/buttons/state' met inhoud integer
    rospy.Subscriber('/Signaal', String, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
