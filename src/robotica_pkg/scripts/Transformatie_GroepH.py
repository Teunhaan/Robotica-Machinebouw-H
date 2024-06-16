#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from depthai_ros_msgs.msg import SpatialDetectionArray

def transform_pose(input_pose, from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        output_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
        return output_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr('Transform failed')
        return None

def callback(data):
    if not data.detections:
        rospy.loginfo("Geen detecties gevonden.")
        return

    eerste_detectie = data.detections[0]
    if not eerste_detectie.results:
        rospy.loginfo("Geen resultaten gevonden in de eerste detectie.")
        return

    eerste_resultaat_id = eerste_detectie.results[0].id
    x_waarde = eerste_detectie.position.x
    y_waarde = eerste_detectie.position.y
    z_waarde = eerste_detectie.position.z

    rospy.loginfo("ID van het eerste resultaat: %d", eerste_resultaat_id)
    rospy.loginfo("Positie x: %f", x_waarde)
    rospy.loginfo("Positie y: %f", y_waarde)
    rospy.loginfo("Positie z: %f", z_waarde)

    if eerste_resultaat_id == 0:
        rospy.loginfo("Colgate")
        item = 'Colgate_0'
    elif eerste_resultaat_id == 1:
        rospy.loginfo("Elektrisch")
        item = 'Elektrisch_0'
    elif eerste_resultaat_id == 2:
        rospy.loginfo("PepaPig")
        item = 'PepaPig_0'
    elif eerste_resultaat_id == 3:
        rospy.loginfo("TongBorstel")
        item = 'TongBorstel_0'
    else:
        rospy.logwarn("Received unexpected value: %i", eerste_resultaat_id)
        return

    input_pose = PoseStamped()
    input_pose.header.frame_id = item
    input_pose.pose.position.x = x_waarde
    input_pose.pose.position.y = y_waarde
    input_pose.pose.position.z = z_waarde

    output_pose = transform_pose(input_pose, 'camera_link', 'world')
    if output_pose:
        rospy.loginfo('Getransformeerde Pose: %s', output_pose)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/stereo_inertial_nn_publisher/color/detections", SpatialDetectionArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
