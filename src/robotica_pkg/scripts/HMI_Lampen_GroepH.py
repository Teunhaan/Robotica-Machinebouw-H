#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

def callback(data):
    pub = rospy.Publisher('/avans/leds/state', UInt8,  queue_size=1)
    value = data.data

    # Poort D01 op de arduino werkt niet naar behoren dus er wordt begonnen op poort D02

    if value == "Geel":
        # Gelezen waarde is Geel upload 000100
        rospy.loginfo("Geel Single signaal ontvangen")
        pub.publish(4)
    elif value == "Rood":
        # Gelezen waarde is Rood upload 001000
        rospy.loginfo("Rood signaal ontvangen")
        pub.publish(8)
    elif value == "Piep":
        # Gelezen waarde is Piep upload 010000
        rospy.loginfo("Piep signaal ontvangen")
        pub.publish(16)
    elif value == "Groen":
        # Gelezen waarde is Groen upload 100000 
        rospy.loginfo("Groen signaal ontvangen")
        pub.publish(32)
    else:
        rospy.logwarn("Received unexpected value: %s", value)

def listener():
    # Start ros node
    rospy.init_node('Lampen_listener')
    
    # Subscriber op '/Signaal' met inhoud string
    rospy.Subscriber('/Lampen', String, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
