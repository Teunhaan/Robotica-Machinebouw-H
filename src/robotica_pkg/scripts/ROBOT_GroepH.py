#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import moveit_commander
import threading
import sys
from xarm_msgs.srv import SetInt16, SetInt16Request
from geometry_msgs.msg import Pose
import time
from math import radians
from robotica_pkg.srv import lokalisatie, lokalisatieRequest, GetAngle, GetAngleResponse
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

# Global variables
operation_in_progress = False
repeat_count = 0
repeat_limit = 1
stop_requested = False  # Added variable to track stop button status

# Initialize MoveIt and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Signal_Listener', anonymous=True)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('arm')

# Set speed limits
group.set_max_velocity_scaling_factor(1)  # Adjust this value as needed
group.set_max_acceleration_scaling_factor(1)  # Adjust this value as needed

def open_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 0  
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        lamp_pub.publish('Fout')
        rospy.logerr("Failed to call gripper service: %s", e)

def sluit_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 1  
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        lamp_pub.publish('Fout')
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


        try:
            lokalisatie_service = rospy.ServiceProxy('lokalisatie', lokalisatie)
            request = lokalisatieRequest()  # Maak een verzoek object aan
            response = lokalisatie_service(request)  # Stuur het verzoek naar de service
            
            X = response.Xw 
            Y = response.Yw 
            z = response.Zw 
            Naam = response.Naam
            
            rospy.loginfo("Product locatie: x={}, y={}, z={}, Naam={}".format(X, Y, z, Naam))
            
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call lokalisatie service: %s", e)
            lamp_pub.publish('Storing')
            break  # Skip this iteration if service call fails

        
        try:
            hoek_service = rospy.ServiceProxy('get_angle', GetAngle)
            hoek_response = hoek_service()  # No need to pass an empty request object
            Hoek = hoek_response.angle
            rospy.loginfo("Received angle: %f", Hoek)
        except rospy.ServiceException as e:
            lamp_pub.publish('Fout')
            rospy.logerr("Angle service call failed: %s", e)

        


        print("== ga naar rechts ==")
        target_values = group.get_named_target_values("right")
        group.go(target_values, wait=True)

        # Lokalisatie van producten opvragen
        

        rospy.loginfo('tandenborstel pakken')
        oppakken(-0.3, 0.6, 0.15, Hoek)

        if Naam == "Colgate":
            rospy.loginfo('Moving to bottom right...')
            plaatsen(-0.31, 0.25, 0.06)
            rospy.loginfo('At position 4')

        elif Naam == "Elektrisch":
            rospy.loginfo('Moving to top right...')
            plaatsen(-0.31, 0.15, 0.13)
            rospy.loginfo('At position 2')

        elif Naam == "PepaPig":
            rospy.loginfo('Moving to top left...')
            plaatsen(-0.2, 0.15, 0.13)
            rospy.loginfo('At position 1')

        elif Naam == "TongBorstel":
            rospy.loginfo('Moving to bottom left...')
            plaatsen(-0.2, 0.25, 0.06)
            rospy.loginfo('At position 3')

        print("== ga naar home ==")
        target_values = group.get_named_target_values("home")
        group.go(target_values, wait=True)
        open_gripper()

        repeat_count += 1

    operation_in_progress = False  # Reset operation_in_progress after completion
    lamp_pub.publish('WachtOpStart')

    if not stop_requested:
        rospy.loginfo('Stopped due to stop button')

def oppakken(x, y, z, graden):
    # Configure target position
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z + 0.06
    
    q = [0.999999999037, -1.31321999012e-06, -1.02368657662e-05, 4.26455450865e-05]
    print (q)
    
    roll, pitch, yaw = euler_from_quaternion(q)
    print (roll)
    print (pitch)
    print (yaw)
    # Pas de quaternion aan als nodig is
    # Bijvoorbeeld, draai 90 graden om de Z-as
    print(graden)
    yaw += graden  # 90 graden in radialen
    print (yaw)
    # Bereken de nieuwe quaternion
    q_new = quaternion_from_euler(roll, pitch, yaw)

    # Update de pose met de nieuwe quaternion
    pose_target.orientation.x = q_new[0]
    pose_target.orientation.y = q_new[1]
    pose_target.orientation.z = q_new[2]
    pose_target.orientation.w = q_new[3]

    # Execute the movement with the updated orientation
    group.set_pose_target(pose_target)
    group.go(wait=True)

    # Adjust position slightly and ensure joint 6 orientation is maintained
    pose_target.position.z = z - 0.08

    # Execute the movement with the updated orientation
    group.set_pose_target(pose_target)
    group.go(wait=True)

    print (group)

    time.sleep(3)
    sluit_gripper()
    time.sleep(1)

    pose_target.position.z = z + 0.08
    group.set_pose_target(pose_target)
    group.go(wait=True)

def plaatsen(x, y, z):
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z + 0.06
    
    q = [0.999999999037, -1.31321999012e-06, -1.02368657662e-05, 4.26455450865e-05]
    print (q)
    
    roll, pitch, yaw = euler_from_quaternion(q)
    print (roll)
    print (pitch)
    print (yaw)
    # Pas de quaternion aan als nodig is
    # Bijvoorbeeld, draai 90 graden om de Z-as
    yaw += radians(90)  # 90 graden in radialen
    print (yaw)
    # Bereken de nieuwe quaternion
    q_new = quaternion_from_euler(roll, pitch, yaw)

    # Update de pose met de nieuwe quaternion
    pose_target.orientation.x = q_new[0]
    pose_target.orientation.y = q_new[1]
    pose_target.orientation.z = q_new[2]
    pose_target.orientation.w = q_new[3]

    # Execute the movement with the updated orientation
    group.set_pose_target(pose_target)
    group.go(wait=True)

    # Adjust position slightly and ensure joint 6 orientation is maintained
    pose_target.position.y = y - 0.07

    # Execute the movement with the updated orientation
    group.set_pose_target(pose_target)
    group.go(wait=True)

    print (group)

    # Open the gripper after each movement
    open_gripper()
    time.sleep(1)

    pose_target.position.y = y + 0.07
    group.set_pose_target(pose_target)
    group.go(wait=True)


if __name__ == '__main__':
    try:
        lamp_pub = rospy.Publisher('/Lampen', String, queue_size=5)
        
        time.sleep(3)  # Ensure that ROS communication is properly initialized
        lamp_pub.publish('WachtOpStart')  # Publish initial state

        rospy.Subscriber('/Signaal', String, signaal)
        rospy.loginfo('Ready to receive signals...')

        rospy.spin()  # Keep the node running and processing callbacks
    except rospy.ROSInterruptException:
        lamp_pub.publish('Fout')
        pass
