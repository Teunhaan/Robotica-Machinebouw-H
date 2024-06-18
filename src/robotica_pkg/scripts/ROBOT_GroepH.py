#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import moveit_commander
import threading
import sys
from xarm_msgs.srv import SetInt16, SetInt16Request, SetInt16Response
from geometry_msgs.msg import Pose
import time

# Global variables
operation_in_progress = False
repeat_count = 0
repeat_limit = 1
stop_requested = False  # Added variable to track stop button status

# Initialize MoveIt
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('arm')

# Set speed limits
group.set_max_velocity_scaling_factor(0.2)  # Adjust this value as needed
group.set_max_acceleration_scaling_factor(0.2)  # Adjust this value as needed

def open_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 0  # Assuming 0 opens the gripper
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call gripper service: %s", e)

def sluit_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 1  # Assuming 1 closes the gripper
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call gripper service: %s", e)

def signaal(data):
    global operation_in_progress, repeat_limit, repeat_count, stop_requested

    if data.data == "Start" or data.data == "StartCyclus":
        repeat_limit = 1 if data.data == "Start" else 4
        repeat_count = 0
        stop_requested = False  # Reset the stop button status
        rospy.loginfo("Start signal received. Beginning the action...")

        if not operation_in_progress:
            operation_in_progress = True
            threading.Thread(target=bewegingen).start()
            lamp_pub.publish('InBedrijf')

    elif data.data == "Stop":
        stop_requested = True
        rospy.loginfo("Stop signal received. Stopping operation...")

def bewegingen():
    global repeat_count, stop_requested, operation_in_progress

    while repeat_count < repeat_limit and not stop_requested:
        print("== ga naar home ==")
        target_values = group.get_named_target_values("home")
        group.go(target_values, wait=True)
        open_gripper()

        print("== ga naar rechts ==")
        target_values = group.get_named_target_values("right")
        group.go(target_values, wait=True)

        rospy.loginfo('tandenborstel pakken')
        oppakken(-0.4, 0.600, 0.15)

        # Movement to top left
        rospy.loginfo('Moving to top left...')
        plaatsen(-0.2, 0.15, 0.13)
        rospy.loginfo('At position 1')

        # Movement to top right
        rospy.loginfo('Moving to top right...')
        plaatsen(-0.31, 0.15, 0.13)
        rospy.loginfo('At position 2')

        # Movement to bottom left
        rospy.loginfo('Moving to bottom left...')
        plaatsen(-0.2, 0.25, 0.06)
        rospy.loginfo('At position 3')

        # Movement to bottom right
        rospy.loginfo('Moving to bottom right...')
        plaatsen(-0.31, 0.25, 0.06)
        rospy.loginfo('At position 4')

        print("== ga naar home ==")
        target_values = group.get_named_target_values("home")
        group.go(target_values, wait=True)
        open_gripper()

        repeat_count += 1

    operation_in_progress = False  # Reset operation_in_progress after completion
    lamp_pub.publish('WachtOpStart')

    if not stop_requested:
        rospy.loginfo('Stopped due to stop button')

def oppakken(x, y, z):
    # Configure target position
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y 
    pose_target.position.z = z + 0.06  # Initial z position

    # Set orientation (assuming it remains constant for gripping)
    pose_target.orientation.x = 0.999999999037
    pose_target.orientation.y = -1.31321999012e-06
    pose_target.orientation.z = -1.02368657662e-05
    pose_target.orientation.w = 4.26455450865e-05

    # Move to the initial z position + 0.06
    group.set_pose_target(pose_target)
    group.go(wait=True)
    
    # Lower the gripper to the specified z position
    pose_target.position.z = z - 0.08
    group.set_pose_target(pose_target)
    group.go(wait=True)

    # Open the gripper after reaching the lower z position
    time.sleep(3)
    sluit_gripper()
    time.sleep(1)

    # Raise the gripper back to the initial z position + 0.10
    pose_target.position.z = z + 0.08
    group.set_pose_target(pose_target)
    group.go(wait=True)

def plaatsen(x, y, z):
    # Configure target position
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y 
    pose_target.position.z = z + 0.06
    pose_target.orientation.x = 0.999999999037
    pose_target.orientation.y = -1.31321999012e-06
    pose_target.orientation.z = -1.02368657662e-05
    pose_target.orientation.w = 4.26455450865e-05

    # Execute the movement
    group.set_pose_target(pose_target)
    group.go(wait=True)
    
    pose_target.position.y = y - 0.07
    group.set_pose_target(pose_target)
    group.go(wait=True)

    # Open the gripper after each movement
    open_gripper()
    time.sleep(1)

    pose_target.position.y = y + 0.07
    group.set_pose_target(pose_target)
    group.go(wait=True)

    sluit_gripper()

if __name__ == '__main__':
    try:
        rospy.init_node('Signal_Listener')  # Initialize the ROS node

        lamp_pub = rospy.Publisher('/Lampen', String, queue_size=5)
        
        time.sleep(3)  # Ensure that ROS communication is properly initialized
        lamp_pub.publish('WachtOpStart')  # Publish initial state

        rospy.Subscriber('/Signaal', String, signaal)
        rospy.loginfo('Ready to receive signals...')

        rospy.spin()  # Keep the node running and processing callbacks
    except rospy.ROSInterruptException:
        pass
