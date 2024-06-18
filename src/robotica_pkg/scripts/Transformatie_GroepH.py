#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from depthai_ros_msgs.msg import SpatialDetectionArray
from robotica_pkg.srv import lokalisatie, lokalisatieResponse

listener = None
service = None  # Define a global variable to hold the service object

def lokalisatie_detectie(req):
    global listener, service
    
    data = rospy.wait_for_message("/stereo_inertial_nn_publisher/color/detections", SpatialDetectionArray)

    if not data.detections:
        rospy.loginfo("Geen detecties gevonden.")
        return lokalisatieResponse(Xw=0, Yw=0, Zw=0)

    eerste_detectie = data.detections[0]
    eerste_resultaat_id = eerste_detectie.results[0].id

    if eerste_resultaat_id == 0:
        rospy.loginfo("Resultaat van de herkenning is: Colgate")
        item = "Colgate_0"
    elif eerste_resultaat_id == 1:
        rospy.loginfo("Resultaat van de herkenning is: Elektrisch")
        item = "Elektrisch_0"
    elif eerste_resultaat_id == 2:
        rospy.loginfo("Resultaat van de herkenning is: PepaPig")
        item = "PepaPig_0"
    elif eerste_resultaat_id == 3:
        rospy.loginfo("Resultaat van de herkenning is: TongBorstel")
        item = "TongBorstel_0"
    else:
        rospy.logwarn("Received unexpected value: %i", eerste_resultaat_id)
        return lokalisatieResponse(Xw=0, Yw=0, Zw=0)
        

    # Start waarde met een '/' als TF.
    item = "/" + item

    try:
        # Wacht maximaal 1 seconde op de transform
        listener.waitForTransform('/world', item, rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/world', item, rospy.Time(0))
        rospy.loginfo("Translation: %s", str(trans))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF fout: %s", str(e))
        return lokalisatieResponse(Xw=0, Yw=0, Zw=0)

    return lokalisatieResponse(Xw=trans[0],Yw=trans[1],Zw=0.03)

def listener_node():
    global listener, service
    rospy.init_node('lokalisatie_camera')
    listener = tf.TransformListener()
    
    service = rospy.Service('lokalisatie', lokalisatie, lokalisatie_detectie)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_node()
    except rospy.ROSInterruptException:
        pass
