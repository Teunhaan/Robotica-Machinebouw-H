#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8, String

class SignalPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/Signaal', String, queue_size=1)
        self.last_value = None
        self.stop_active = False

    def callback(self, data):
        value = data.data
        if value > 0 and value != self.last_value:
            if value == 1:
                # Gelezen waarde is 1 waarde Start wordt geupload naar topic /Signaal 
                rospy.loginfo("Waarde 1: Start")
                self.pub.publish('Start')
                self.stop_active = False  # Stop het "Stop" signaal als "Start" wordt ontvangen
            elif value == 2:
                # Gelezen waarde is 2 waarde Start Single wordt geupload naar topic /Signaal 
                rospy.loginfo("Waarde 2: Start Cyclus")
                self.pub.publish('StartCyclus')
                self.stop_active = False  # Stop het "Stop" signaal als "StartCyclus" wordt ontvangen
            elif value == 4:
                # Gelezen waarde is 4 waarde Stop wordt geupload naar topic /Signaal 
                rospy.loginfo("Waarde 4: Stop")
                self.pub.publish('Stop')
                self.stop_active = False  # Houd het "Stop" signaal actief
            else:
                rospy.logwarn("Onjuiste waarde ontvangen: %d", value)
            self.last_value = value
        elif value == 0:
            self.last_value = None


    def listener(self):
        # Start de ROS node
        rospy.init_node('Knoppen_listener')
        
        # Subscriber op '/avans/buttons/state' met inhoud integer
        rospy.Subscriber('/avans/buttons/state', UInt8, self.callback)
        
        # Loop om het "Stop" signaal actief te houden als er geen "Start" of "StartCyclus" wordt ontvangen
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.stop_active:
                rospy.loginfo("Stop signaal blijft actief...")
                self.pub.publish('Stop')
            rate.sleep()

if __name__ == '__main__':
    try:
        signal_publisher = SignalPublisher()
        signal_publisher.listener()
    except rospy.ROSInterruptException:
        pass
