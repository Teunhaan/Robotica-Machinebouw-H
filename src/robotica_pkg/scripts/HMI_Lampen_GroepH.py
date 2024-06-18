#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

def callback(data):
    #Publisher aanmaken naar de juiste locatie voor rosserial.
    pub = rospy.Publisher('/avans/leds/state', UInt8,  queue_size=1)
    value = data.data

    # Poort D01 op de arduino werkt niet naar behoren dus er wordt begonnen op poort D02

    if value == "WachtOpStart":
        # Gelezen waarde is WachtOpStart upload 100000
        rospy.loginfo("WachtOpStart signaal ontvangen")
        pub.publish(32)
    elif value == "InBedrijf":
        # Gelezen waarde is Rood upload 000100
        rospy.loginfo("InBedrijf signaal ontvangen")
        pub.publish(16)
    elif value == "Storing":
        # Gelezen waarde is Storing upload 100100
        rospy.loginfo("Storing signaal ontvangen")
        pub.publish(48)
    elif value == "Fout":
        # Gelezen waarde is Fout upload 001000
        rospy.loginfo("Fout signaal ontvangen")
        pub.publish(8)



    # Buiten de opdracht nog mogelijk te gebruiken
    elif value == "Piep":
        # Gelezen waarde is Piep upload 010000
        rospy.loginfo("Piep signaal ontvangen")
        pub.publish(16)
    elif value == "Noodstop":
        # Gelezen waarde is Noodstop upload 011000 
        rospy.loginfo("Noodstop signaal ontvangen")
        pub.publish(24)
    else:
        rospy.logwarn("Received unexpected value: %s", value)

def listener():
    # Start ros node
    rospy.init_node('Lampen_listener')
    
    # Subscriber op '/Lampen' met inhoud string
    rospy.Subscriber('/Lampen', String, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


    
