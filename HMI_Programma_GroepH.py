#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, String

def callback(data):
    pub = rospy.Publisher('/Signaal', String,  queue_size=1)
    value = data.data
    if value == 1:
        # Gelezen waarde is 1 waarde Start wordt geupload naar topic /Signaal 
        rospy.loginfo("Received 1: Case 1")
        pub.publish('Start')
    elif value == 2:
        # Gelezen waarde is 2 waarde Start Single wordt geupload naar topic /Signaal 
        rospy.loginfo("Received 2: Case 2")
        pub.publish('Start Single')
    elif value == 4:
        # Gelezen waarde is 4 waarde Stop wordt geupload naar topic /Signaal 
        rospy.loginfo("Received 4: Case 4")
        pub.publish('Stop')
    elif value == 8:
        # Gelezen waarde is 8 waarde ?? wordt geupload naar topic /Signaal 
        rospy.loginfo("Received 8: Case 8")
        pub.publish('??')
    else:
        rospy.logwarn("Received unexpected value: %d", value)

def listener():
    # Initialize the ROS node
    rospy.init_node('listener')
    
    # Subscriber op '/avans/buttons/state' met inhoud integer
    rospy.Subscriber('/avans/buttons/state', Int32, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
