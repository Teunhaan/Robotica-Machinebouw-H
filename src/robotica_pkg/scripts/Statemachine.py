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
from robotica_pkg.srv import lokalisatie, lokalisatieRequest, lampen, lampenResponse, lampenRequest, GetAngle, GetAngleRequest, GetAngleResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core import Behavior, OperatableStateMachine
from depthai_ros_msgs.msg import SpatialDetectionArray

# Globale variabelen
operation_in_progress = False
stop_requested = False
detection_count = 0

class SetLampenStatusState(EventState):
    def __init__(self, status):
        super(SetLampenStatusState, self).__init__(outcomes=['done', 'failed'])
        self.status = status
        self.lampen_service = 'Lampen'
        self.proxy = ProxyServiceCaller({self.lampen_service: lampen})

    def execute(self, userdata):
        try:
            request = lampenRequest(self.status)
            response = self.proxy.call(self.lampen_service, request)
            Logger.loginfo("Antwoord van lampenservice: %s" % response)
            return 'done'
        except rospy.ServiceException as e:
            Logger.logerr("Kon lampenservice niet aanroepen: %s" % e)
            return 'failed'

class OpenGripperState(EventState):
    def __init__(self):
        super(OpenGripperState, self).__init__(outcomes=['done', 'failed'])
        self.gripper_service = '/ufactory/vacuum_gripper_set'
        self.proxy = ProxyServiceCaller({self.gripper_service: SetInt16})

    def execute(self, userdata):
        request = SetInt16Request(data=0)
        try:
            response = self.proxy.call(self.gripper_service, request)
            Logger.loginfo("Antwoord van gripperservice: %s" % response)
            return 'done'
        except rospy.ServiceException as e:
            Logger.logerr("Kon gripperservice niet aanroepen: %s" % e)
            return 'failed'

class CloseGripperState(EventState):
    def __init__(self):
        super(CloseGripperState, self).__init__(outcomes=['done', 'failed'])
        self.gripper_service = '/ufactory/vacuum_gripper_set'
        self.proxy = ProxyServiceCaller({self.gripper_service: SetInt16})

    def execute(self, userdata):
        request = SetInt16Request(data=1)
        try:
            response = self.proxy.call(self.gripper_service, request)
            Logger.loginfo("Antwoord van gripperservice: %s" % response)
            return 'done'
        except rospy.ServiceException as e:
            Logger.logerr("Kon gripperservice niet aanroepen: %s" % e)
            return 'failed'

class MoveToNamedTargetState(EventState):
    def __init__(self, target_name):
        super(MoveToNamedTargetState, self).__init__(outcomes=['done', 'failed'])
        self.target_name = target_name
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')

    def execute(self, userdata):
        try:
            self.group.set_named_target(self.target_name)
            self.group.go(wait=True)
            return 'done'
        except Exception as e:
            Logger.logerr("Kon niet naar doel bewegen: %s" % e)
            return 'failed'

class GetLocalizationDataState(EventState):
    def __init__(self):
        super(GetLocalizationDataState, self).__init__(outcomes=['done', 'failed'], output_keys=['x', 'y', 'z', 'name'])
        self.localization_service = 'lokalisatie'
        self.proxy = ProxyServiceCaller({self.localization_service: lokalisatie})

    def execute(self, userdata):
        try:
            response = self.proxy.call(self.localization_service, lokalisatieRequest())
            Logger.loginfo("Productlocatie: x={}, y={}, z={}, Naam={}".format(response.Xw, response.Yw, response.Zw, response.Naam))
            userdata.x = response.Xw
            userdata.y = response.Yw
            userdata.z = response.Zw
            userdata.name = response.Naam
            return 'done'
        except rospy.ServiceException as e:
            Logger.logerr("Kon lokalisatieservice niet aanroepen: %s" % e)
            return 'failed'

class GetAngleDataState(EventState):
    def __init__(self):
        super(GetAngleDataState, self).__init__(outcomes=['done', 'failed'], output_keys=['angle'])
        self.angle_service = 'GetAngle'
        self.proxy = ProxyServiceCaller({self.angle_service: GetAngle})

    def execute(self, userdata):
        try:
            response = self.proxy.call(self.angle_service, GetAngleRequest())
            Logger.loginfo("Ontvangen hoek: {}".format(response.angle))
            userdata.angle = response.angle
            return 'done'
        except rospy.ServiceException as e:
            Logger.logerr("Kon hoekservice niet aanroepen: %s" % e)
            return 'failed'

class PickUpProductState(EventState):
    def __init__(self):
        super(PickUpProductState, self).__init__(outcomes=['done', 'failed'], input_keys=['x', 'y', 'z', 'angle'])
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')

    def execute(self, userdata):
        try:
            pose_target = Pose()
            pose_target.position.x = userdata.x
            pose_target.position.y = userdata.y
            pose_target.position.z = userdata.z + 0.06

            roll, pitch, yaw = euler_from_quaternion([0.999999999037, -1.31321999012e-06, -1.02368657662e-05, 4.26455450865e-05])
            yaw += userdata.angle
            q_new = quaternion_from_euler(roll, pitch, yaw)

            pose_target.orientation.x = q_new[0]
            pose_target.orientation.y = q_new[1]
            pose_target.orientation.z = q_new[2]
            pose_target.orientation.w = q_new[3]

            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            pose_target.position.z = userdata.z - 0.08
            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            pose_target.position.z = userdata.z + 0.08
            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            return 'done'
        except Exception as e:
            Logger.logerr("Kon product niet oppakken: %s" % e)
            return 'failed'

class PlaceProductState(EventState):
    def __init__(self, x, y, z):
        super(PlaceProductState, self).__init__(outcomes=['done', 'failed'])
        self.x = x
        self.y = y
        self.z = z
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')

    def execute(self, userdata):
        # NOG EEN WERKEND PROGRAMMA MAKEN ;-;
        try:
            pose_target = Pose()
            pose_target.position.x = self.x
            pose_target.position.y = self.y
            pose_target.position.z = self.z

            #ORIENTATIE MAKEN ;-;
            roll, pitch, yaw = euler_from_quaternion([0.999999999037, -1.31321999012e-06, -1.02368657662e-05, 4.26455450865e-05])
            yaw += radians(90)
            q_new = quaternion_from_euler(roll, pitch, yaw)

            pose_target.orientation.x = q_new[0]
            pose_target.orientation.y = q_new[1]
            pose_target.orientation.z = q_new[2]
            pose_target.orientation.w = q_new[3]

            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            #Beweging de bak in
            pose_target.position.y = self.y - 0.07
            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            #Beweging de bak uit
            pose_target.position.y = self.y + 0.07
            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)

            return 'done'
        except Exception as e:
            Logger.logerr("Kon product niet plaatsen: %s" % e)
            return 'failed'

class DeterminePlacementState(EventState):
    def __init__(self):
        super(DeterminePlacementState, self).__init__(outcomes=['done', 'failed'], input_keys=['name'], output_keys=['x', 'y', 'z'])

    def execute(self, userdata):
        try:
            if userdata.name == "Colgate":
                rospy.loginfo('Moving to bottom right...')
                userdata.x = -0.31
                userdata.y = 0.25
                userdata.z = 0.06
            elif userdata.name == "Elektrisch":
                rospy.loginfo('Moving to top right...')
                userdata.x = -0.31
                userdata.y = 0.15
                userdata.z = 0.13
            elif userdata.name == "PeppaPig":
                rospy.loginfo('Moving to top left...')
                userdata.x = -0.2
                userdata.y = 0.15
                userdata.z = 0.13
            elif userdata.name == "TongBorstel":
                rospy.loginfo('Moving to bottom left...')
                userdata.x = -0.2
                userdata.y = 0.25
                userdata.z = 0.06
            else:
                rospy.logerr("Onbekend product: %s" % userdata.name)
                return 'failed'
            return 'done'
        except Exception as e:
            Logger.logerr("Fout bij het bepalen van de plaatsing: %s" % e)
            return 'failed'

class CheckDetectionsState(EventState):
    def __init__(self):
        super(CheckDetectionsState, self).__init__(outcomes=['done', 'failed'])
        self.detection_count = 0

    def execute(self, userdata):
        global detection_count
        self.detection_count = detection_count
        rospy.loginfo("Aantal detecties: %d" % self.detection_count)
        if self.detection_count > 0:
            #Herhaal functie inbouwen
            return 'done'
        else:
            return 'failed'

class WaitForStartState(EventState):
    def __init__(self):
        super(WaitForStartState, self).__init__(outcomes=['start', 'start_cycle', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("Wachten op startsignaal...")
        while not rospy.is_shutdown():
            if rospy.get_param('/start_signal_received', False):
                rospy.set_param('/start_signal_received', False)
                return 'start'
            elif rospy.get_param('/start_cycle_signal_received', False):
                rospy.set_param('/start_cycle_signal_received', False)
                return 'start_cycle'
            time.sleep(0.1)
        return 'failed'

class ErrorState(EventState):
    def __init__(self):
        super(ErrorState, self).__init__(outcomes=['failed'])

    def execute(self, userdata):
        rospy.logerr("Er is een fout opgetreden tijdens de operatie.")
        return 'failed'

class RobotBehavior(Behavior):

    def __init__(self):
        super(RobotBehavior, self).__init__()
        self.name = 'Robot Behavior'

    def create(self):
        # Aanmaken van state machine
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _state_machine:
            OperatableStateMachine.add('WaitForStart',
                                       WaitForStartState(),
                                       transitions={'start': 'SetInBedrijf', 'start_cycle': 'CheckDetections', 'failed': 'ErrorState'})
            
            OperatableStateMachine.add('CheckDetections',
                                       CheckDetectionsState(),
                                       transitions={'done': 'SetInBedrijf', 'failed': 'ErrorState'})


            OperatableStateMachine.add('SetInBedrijf',
                                       SetLampenStatusState('InBedrijf'),
                                       transitions={'done': 'OpenGripper', 'failed': 'ErrorState'})

            OperatableStateMachine.add('OpenGripper',
                                       OpenGripperState(),
                                       transitions={'done': 'GetLocalizationData', 'failed': 'ErrorState'})

            OperatableStateMachine.add('GetLocalizationData',
                                       GetLocalizationDataState(),
                                       transitions={'done': 'GetAngleData', 'failed': 'ErrorState'},
                                       remapping={'x': 'x', 'y': 'y', 'z': 'z', 'name': 'name'})

            OperatableStateMachine.add('GetAngleData',
                                       GetAngleDataState(),
                                       transitions={'done': 'MoveToPickUp', 'failed': 'ErrorState'},
                                       remapping={'angle': 'angle'})

            OperatableStateMachine.add('MoveToPickUp',
                                       MoveToNamedTargetState(target_name='pick_up'),
                                       transitions={'done': 'PickUpProduct', 'failed': 'ErrorState'})

            OperatableStateMachine.add('PickUpProduct',
                                       PickUpProductState(),
                                       transitions={'done': 'DeterminePlacement', 'failed': 'ErrorState'},
                                       remapping={'x': 'x', 'y': 'y', 'z': 'z', 'angle': 'angle'})

            OperatableStateMachine.add('DeterminePlacement',
                                       DeterminePlacementState(),
                                       transitions={'done': 'PlaceProduct', 'failed': 'ErrorState'},
                                       remapping={'x': 'x', 'y': 'y', 'z': 'z', 'name': 'name'})

            OperatableStateMachine.add('PlaceProduct',
                                       PlaceProductState(),
                                       transitions={'done': 'OpenGripper', 'failed': 'ErrorState'},
                                       remapping={'x': 'x', 'y': 'y', 'z': 'z'})

            OperatableStateMachine.add('ErrorState',
                                       SetLampenStatusState('Storing'),
                                       transitions={'failed': 'WaitForStart'})

        return _state_machine

def detection_callback(msg):
    global detection_count
    detection_count = len(msg.detections)
    rospy.loginfo("Aantal detecties: %d" % detection_count)

def signal_callback(data):
    if data.data == "Start":
        rospy.set_param('/start_signal_received', True)
    elif data.data == "StartCyclus":
        rospy.set_param('/start_cycle_signal_received', True)
    elif data.data == "Stop":
        rospy.loginfo("Stopsignaal ontvangen. Operatie stoppen...")

if __name__ == '__main__':
    rospy.init_node('robot_behavior')
    behavior = RobotBehavior()
    rospy.Subscriber('/Signaal', String, signal_callback)
    rospy.Subscriber('/stereo_inertial_nn_publisher/color/detections', SpatialDetectionArray, detection_callback)
    outcome = behavior.execute()
    rospy.loginfo('Uitvoering van gedrag voltooid met resultaat: %s' % outcome)
