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
from robotica_pkg.srv import lokalisatie, lokalisatieRequest, GetAngle, GetAngleResponse, lampen, lampenRequest, lampenResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from depthai_ros_msgs.msg import SpatialDetectionArray

# Global variables
operation_in_progress = False
repeat_count = 0
repeat_limit = 1
stop_requested = False

# Initialize MoveIt and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Signal_Listener', anonymous=True)


# Global variable to store the last received state
last_state = None

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('arm')

# Set speed limits
group.set_max_velocity_scaling_factor(0.1)
group.set_max_acceleration_scaling_factor(0.1)

def open_gripper():
    time.sleep(1)
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 0  
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        set_lampen_status('Fout')
        rospy.logerr("Failed to call gripper service: %s", e)

def close_gripper():
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 1  
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
        time.sleep(1)
    except rospy.ServiceException as e:
        set_lampen_status('Fout')
        rospy.logerr("Failed to call gripper service: %s", e)

def signal_callback(data):
    global operation_in_progress, repeat_limit, repeat_count, stop_requested, detection_count

    if operation_in_progress == False:
      move_to_named_target("home")

    if data.data in ["Start", "StartCyclus"] and not operation_in_progress:
        
        repeat_limit = 1 if data.data == "Start" else detection_count
        repeat_count = 0
        stop_requested = False
        rospy.loginfo("Start signal received. Beginning the action...")

        
        if not operation_in_progress and not detection_count == 0:
            operation_in_progress = True
            threading.Thread(target=perform_movements).start()
            set_lampen_status('InBedrijf')
        else:
            set_lampen_status('Storing')


    elif data.data == "Stop":
        stop_requested = True
        rospy.loginfo("Stop signal received. Stopping operation...")


def perform_movements():
    global repeat_count, stop_requested, operation_in_progress

    while repeat_count < repeat_limit and not stop_requested:
        rospy.loginfo("Going to home position")
        move_to_named_target("home")
        open_gripper()

        X, Y, Z, Naam = get_localisation_data()

        angle = get_angle_data()
        if X is None or Y is None or angle is None:
            set_lampen_status('Fout')
            return
        
        if not pick_up_product(X, Y, 0.15, angle):
            break
    

        if Naam == "Colgate":
            rospy.loginfo('Moving to bottom right...')
            if not place_product(-0.307, 0.25, 0.045):
                break

        elif Naam == "Elektrisch":
            rospy.loginfo('Moving to top right...')
            if not place_product(-0.307, 0.15, 0.13):
                break

        elif Naam == "PeppaPig":
            rospy.loginfo('Moving to top left...')
            if not place_product(-0.2, 0.15, 0.13):
                break

        elif Naam == "TongBorstel":
            rospy.loginfo('Moving to bottom left...')
            if not place_product(-0.2, 0.25, 0.045):
                break
            

        rospy.loginfo("Going back to home position")
        move_to_named_target("home")
        open_gripper()

        repeat_count += 1

    operation_in_progress = False
    set_lampen_status('WachtOpStart')

    if stop_requested:
        rospy.loginfo('Stopped due to stop signal')

def move_to_named_target(target_name):
    target_values = group.get_named_target_values(target_name)
    group.go(target_values, wait=True)

def get_localisation_data():
    try:
        print ("lokatie")
        rospy.wait_for_service('lokalisatie')
        lokalisatie_service = rospy.ServiceProxy('lokalisatie', lokalisatie)
        response = lokalisatie_service(lokalisatieRequest())
        rospy.loginfo("Product location: x={}, y={}, z={}, Name={}".format(response.Xw, response.Yw, response.Zw, response.Naam))
        return response.Xw, response.Yw, response.Zw, response.Naam
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call localisation service: %s", e)
        print ("Storing")
        set_lampen_status('Fout')
        return None, None, None, None

def get_angle_data():
    try:
        hoek_service = rospy.ServiceProxy('get_angle', GetAngle)
        response = hoek_service()
        rospy.loginfo("Received angle: %f", response.angle)
        return response.angle
    except rospy.ServiceException as e:
        rospy.logerr("Angle service call failed: %s", e)
        set_lampen_status('Fout')
        return None

def pick_up_product(x, y, z, angle):
    global stop_requested

    pose_target = group.get_current_pose().pose
    pose_target.position.x = x
    pose_target.position.y = y + 0.015
    pose_target.position.z = z + 0.06

    graden = angle
    # Convert degrees to radians
    rad = graden * 3.14159265359 / 180.0

    # Get current orientation
    current_orientation = pose_target.orientation
    current_quat = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

    # Create a quaternion for the desired rotation
    desired_quat = tf.transformations.quaternion_from_euler(0, 0, rad)

    # Combine the current orientation with the desired rotation
    combined_quat = tf.transformations.quaternion_multiply(current_quat, desired_quat)

    pose_target.orientation.x = combined_quat[0]
    pose_target.orientation.y = combined_quat[1]
    pose_target.orientation.z = combined_quat[2]
    pose_target.orientation.w = combined_quat[3]

    group.set_pose_target(pose_target)
    
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion planning or execution failed while picking up the product.")
        if stop_requested == True:
            return False

    pose_target.position.z = z - 0.08
    group.set_pose_target(pose_target)

    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion planning or execution failed while picking up the product.")
        if stop_requested == True:
            return False

    time.sleep(1)
    close_gripper()

    pose_target.position.z = z + 0.08
    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion planning or execution failed while picking up the product.")
        if stop_requested == True:
            return False

    return True

def place_product(x, y, z):
    global stop_requested

    pose_target = group.get_current_pose().pose
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z + 0.06

    # Get current orientation
    new_quat = [-7.07099207e-01, 7.07114351e-01, -3.23752479e-05, 6.78676505e-05]

    pose_target.orientation.x = new_quat[0]
    pose_target.orientation.y = new_quat[1]
    pose_target.orientation.z = new_quat[2]
    pose_target.orientation.w = new_quat[3]

    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion planning or execution failed while placing the product.")
        if stop_requested == True:
            return False


    pose_target.position.y = y - 0.07

    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion planning or execution failed while placing the product.")
        if stop_requested == True:
            return False

    open_gripper()

    return True

def set_lampen_status(waarde):
        lampen_service = rospy.ServiceProxy('/Lampen', lampen)
        # Maak een verzoek aan de service
        request = lampenRequest(waarde)
        response = lampen_service(request)


def detection_callback(msg):
    global detection_count  # Voeg deze regel toe

    # Gebruik niet langer detection_count.unregister() hier

    detection_count = len(msg.detections)
    return


    


if __name__ == '__main__':
    try:
        
        set_lampen_status('WachtOpStart')
        
        try:
            rospy.Subscriber('/stereo_inertial_nn_publisher/color/detections', SpatialDetectionArray, detection_callback)
        except rospy.ROSException as e:
            set_lampen_status('Fout')
            rospy.logerr("Service is not available within the timeout: %s", e)
            
        rospy.Subscriber('/Signaal', String, signal_callback)
        rospy.loginfo('Ready to receive signals...')
        rospy.spin()

    except rospy.ROSInterruptException:
        set_lampen_status('Fout')


