#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String
from robotica_pkg.srv import lampen, lampenResponse

def callback(data):
    pub = rospy.Publisher('/avans/leds/state', UInt8,  queue_size=1)
    value = data.data

    # Poort D01 op de arduino werkt niet naar behoren dus er wordt begonnen op poort D02

    if value == "WachtOpStart":
        # Gelezen waarde is WachtOpStart upload 100000
        print("WachtOpStart signaal ontvangen")
        pub.publish(32)
    elif value == "InBedrijf":
        # Gelezen waarde is Rood upload 001000
        print("InBedrijf signaal ontvangen")
        pub.publish(16)
    elif value == "Storing":
        # Gelezen waarde is Storing upload 110000
        print("Storing signaal ontvangen")
        pub.publish(48)
    elif value == "Fout":
        # Gelezen waarde is Fout upload 001000
        print("Fout signaal ontvangen")
        pub.publish(8)
    elif value == "Piep":
        # Gelezen waarde is Piep upload 000100
        print("Piep signaal ontvangen")
        pub.publish(4)
    elif value == "Noodstop":
        # Gelezen waarde is Noodstop upload 001100 
        print("Noodstop signaal ontvangen")
        pub.publish(56)
    else:
        print("Received unexpected value: %s", value)

    return lampenResponse('Uploaden is voltooid')

def lampen_server():
    # Start ROS node
    rospy.init_node('Lampen_listener')
    
    rospy.Service('Lampen', lampen, callback)
    rospy.loginfo("Ready to compute.")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        lampen_server()
    except rospy.ROSInterruptException:
        pass
