#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

def callback(data):
    pub = rospy.Publisher('/Signaal', String,  queue_size=1)
    value = data.data
    if value > 0:
        if value == 1:
            # Gelezen waarde is 1 waarde Start wordt geupload naar topic /Signaal 
            rospy.loginfo("Waarde 1: Start")
            pub.publish('Start')
        elif value == 2:
            # Gelezen waarde is 2 waarde Start Single wordt geupload naar topic /Signaal 
            rospy.loginfo("Waarde 2: Start Cyclus")
            pub.publish('StartCyclus')
        elif value == 4:
            # Gelezen waarde is 4 waarde Stop wordt geupload naar topic /Signaal 
            rospy.loginfo("Waarde 4: Stop")
            pub.publish('Stop')
        else:
            rospy.logwarn("Onjuiste waarde ontvangen: %d", value)
    else:
        pass

def listener():
    # Start de ROS node
    rospy.init_node('Knoppen_listener')
    
    # Subscriber op '/avans/buttons/state' met inhoud integer
    rospy.Subscriber('/avans/buttons/state', UInt8, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
