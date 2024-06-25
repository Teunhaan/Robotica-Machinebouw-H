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
from xarm_msgs.msg import RobotMsg

# Global variables
operation_in_progress = False
repeat_count = 0
repeat_limit = 1
stop_requested = False

# Initialize MoveIt and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Signal_Listener', anonymous=True)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('arm')

# Set speed limits
group.set_max_velocity_scaling_factor(0.2)
group.set_max_acceleration_scaling_factor(0.2)


# Functie om de gripper te openen
def open_gripper():
    time.sleep(1)
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)               
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)            
        request = SetInt16Request()
        request.data = 0                                                                                   #Verrander de requested data naar 0 om de gripper te openen
        response = vacuum_gripper_set_proxy(request)
        rospy.loginfo("Gripper service response: %s", response)
    except rospy.ServiceException as e:
        set_lampen_status('Fout')
        rospy.logerr("Failed to call gripper service: %s", e)

def close_gripper():
    time.sleep(1)
    try:
        rospy.wait_for_service('/ufactory/vacuum_gripper_set', timeout=5)
        vacuum_gripper_set_proxy = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        request = SetInt16Request()
        request.data = 1                                                                                    #Verrander de requested data naar 0 om de gripper te sluiten
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
        
        repeat_limit = 1 if data.data == "Start" else detection_count                                       #Afhankelijk van start of cyclus 1 of aantal gedetecteerde objecten
        repeat_count = 0                                                                                    #Tellen van het aantal loops              
        stop_requested = False                                                                              
        rospy.loginfo("Start signal received. Beginning the action...")

        
        if not operation_in_progress and not detection_count == 0:                                          #Zorgen dat het systeem niet opnieuw aan gaat                                         
            operation_in_progress = True
            threading.Thread(target=perform_movements).start()                                              #Start perform_movement parallel aan signal_callback
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
        if not move_to_named_target("home"):                                                                #Controle of de robot naar home gaat en stuurt aan
            set_lampen_status('Fout')       
            operation_in_progress = False
            return
        open_gripper()

        X, Y, Z, Naam = get_localisation_data()                                                             #Pak de locatie data van get_location_data
        angle = get_angle_data()                                                                            #Pakt de data van get_angle_data


        if X is None or Y is None or angle is None:
            set_lampen_status('Fout')
            operation_in_progress = False
            return

        if not pick_up_product(X, Y, 0.15, angle):
            set_lampen_status('Fout')
            operation_in_progress = False
            return 

       
        if Naam == "Colgate":
            rospy.loginfo('Moving to bottom right...')
            if not place_product(-0.307, 0.25, 0.050):
                set_lampen_status('Fout')
                operation_in_progress = False
                return  

        elif Naam == "Elektrisch":
            rospy.loginfo('Moving to top right...')
            if not place_product(-0.307, 0.15, 0.13):
                set_lampen_status('Fout')
                operation_in_progress = False
                return 

        elif Naam == "PeppaPig":
            rospy.loginfo('Moving to top left...')
            if not place_product(-0.200, 0.15, 0.13):
                set_lampen_status('Fout')
                operation_in_progress = False
                return

        elif Naam == "TongBorstel":
            rospy.loginfo('Moving to bottom left...')
            if not place_product(-0.200, 0.25, 0.045):
                set_lampen_status('Fout')
                operation_in_progress = False
                return 

        rospy.loginfo("Going back to home position")
        if not move_to_named_target("home"):
            set_lampen_status('Fout')
            operation_in_progress = False
            return
        open_gripper()

        repeat_count += 1

    operation_in_progress = False
    set_lampen_status('WachtOpStart')

    if stop_requested:
        rospy.loginfo('Stopped due to stop signal')


def move_to_named_target(target_name):
    global stop_requested

    if stop_requested:
        return False

    target_values = group.get_named_target_values(target_name)
    success = group.go(target_values, wait=True)
    
    if not success or stop_requested:
        set_lampen_status('Fout')
        return False
    
    return True

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

    pose_target = group.get_current_pose().pose
    pose_target.position.x = x + 0.005
    pose_target.position.y = y + 0.023
    pose_target.position.z = z + 0.06

    graden = angle + 180
    
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
        rospy.logerr("Product buiten locatie (motion plan mislukt)")
        return False

    pose_target.position.z = z - 0.081
    group.set_pose_target(pose_target)

    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Product buiten locatie (motion plan mislukt)")
        return False

    close_gripper()

    pose_target.position.z = z + 0.081
    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion plan mislukt")
        return False

    return True

def place_product(x, y, z):
    global stop_requested

    pose_target = group.get_current_pose().pose
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z + 0.06

    # Get current orientation
    new_quat = [7.07088233e-01, 7.07125328e-01, 3.15780339e-05, 1.96012893e-05]                   #Waarde van 270graden in quaternial

    pose_target.orientation.x = new_quat[0]
    pose_target.orientation.y = new_quat[1]
    pose_target.orientation.z = new_quat[2]
    pose_target.orientation.w = new_quat[3]

    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion plan mislukt")
        return False


    pose_target.position.y = y - 0.07

    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    
    if not success:
        set_lampen_status('Fout')
        rospy.logerr("Motion plan mislukt")
        return False
    
    open_gripper()

    return True

def set_lampen_status(waarde):
    lampen_service = rospy.ServiceProxy('/Lampen', lampen)
    request = lampenRequest(waarde)
    response = lampen_service(request)

def detection_callback(msg):
    global detection_count  
    detection_count = len(msg.detections)
    return

def Noodstop_callback(msg):
    if msg.state == 4:
        rospy.logerr("Noodstop is ingedrukt")
        set_lampen_status('Noodstop')
        
    

if __name__ == '__main__':
    set_lampen_status('WachtOpStart')
    try:
        rospy.sleep(1)                                                                  # benodigd voor wachtopstart status

        rospy.Subscriber('/Signaal', String, signal_callback)
        try:
            rospy.Subscriber('/stereo_inertial_nn_publisher/color/detections', SpatialDetectionArray, detection_callback)
        except rospy.ROSException as e:
            set_lampen_status('Fout')
            rospy.logerr("Service is not available within the timeout: %s", e)

        rospy.Subscriber('/ufactory/robot_states', RobotMsg, Noodstop_callback)

        set_lampen_status('WachtOpStart')
        rospy.loginfo('Ready to receive signals...')
        rospy.spin()

    except rospy.ROSInterruptException:
        set_lampen_status('Fout')
