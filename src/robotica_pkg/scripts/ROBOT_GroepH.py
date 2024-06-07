#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
import math
from geometry_msgs.msg import Pose
from xarm_msgs.srv import SetInt16, SetInt16Request
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import time
 
# Initialize the ROS node once
rospy.init_node('robot_GroepH')
 
 
 
tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
 
 
 
 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
 
# Set speed limits
group.set_max_velocity_scaling_factor(0.2)  # Adjust this value as needed (0.2 means 20% of the maximum speed)
group.set_max_acceleration_scaling_factor(0.2)  # Adjust this value as needed (0.2 means 20% of the maximum acceleration)
 
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
 
def close_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 1  # Assuming 1 closes the gripper
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call gripper service: %s", e)
 
print("== ga naar home ==")
target_values = group.get_named_target_values("home")
group.go(target_values, wait=True)
 
# gripper open
open_gripper()
 
print("== ga naar rechts ==")
target_values = group.get_named_target_values("right")
group.go(target_values, wait=True)
 
# ga naar tandenborstel
print("== beweeg omlaag, 100 mm ==")
pose = group.get_current_pose()
posetarget = pose
posetarget.pose.position.z -= 0.1
group.set_pose_target(posetarget)
plan = group.plan()
group.go(wait=True)
 
 
# gripper sluiten
time.sleep(1)
close_gripper()
time.sleep(1)
 
 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~Keuze bakjes~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Replace the empty conditions with actual conditions based on your logic
if ():
    print("== ga naar bakje Links boven ==")
    target_values = group.get_named_target_values("BakjeLinksBoven")
    group.go(target_values, wait=True)
elif ():
    print("== ga naar bakje Links onder ==")
    target_values = group.get_named_target_values("BakjeLinksOnder")
    group.go(target_values, wait=True)
elif ():
    print("== ga naar bakje rechts boven ==")
    target_values = group.get_named_target_values("BakjeRechtsBoven")
    group.go(target_values, wait=True)
else:
    print("== ga naar bakje Rechts onder ==")
    target_values = group.get_named_target_values("BakjeRechtsOnder")
    group.go(target_values, wait=True)
 
# Gripper functie
print("== product leveren ==")
pose = group.get_current_pose()
posetarget = pose
posetarget.pose.position.y -= 0.03
group.set_pose_target(posetarget)
plan = group.plan()
group.go(wait=True)
 
# gripper open
time.sleep(1)
open_gripper()
time.sleep(1)
 
pose = group.get_current_pose()
posetarget = pose
posetarget.pose.position.y += 0.03
group.set_pose_target(posetarget)
plan = group.plan()
group.go(wait=True)
 
print("== ga naar home ==")
target_values = group.get_named_target_values("home")
group.go(target_values, wait=True)
 
 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
print("== go to test IK ==")
transform = tf_buffer.lookup_transform('world', 'ik_testpoint', rospy.Time())
 
destination_pose = Pose()
destination_pose.position = transform.transform.translation
destination_pose.orientation = transform.transform.rotation
 
q_orig = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
q_rot = quaternion_from_euler(math.pi, 0, 0)  # rotate along x-axis to meet z-face
q_new = quaternion_multiply(q_rot, q_orig)
transform.transform.rotation.x = q_new[0]
transform.transform.rotation.y = q_new[1]
transform.transform.rotation.z = q_new[2]
transform.transform.rotation.w = q_new[3]
 
group.set_pose_target(destination_pose)
plan = group.plan()
group.go(wait=True)

