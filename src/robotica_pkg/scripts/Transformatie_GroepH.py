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

    x_vals = []
    y_vals = []
    z_vals = []

    for _ in range(10):
        data = rospy.wait_for_message("/stereo_inertial_nn_publisher/color/detections", SpatialDetectionArray)

        if not data.detections:
            rospy.loginfo("Geen detecties gevonden.")
            return lokalisatieResponse(Xw=0, Yw=0, Zw=0, Naam="")

        eerste_detectie = data.detections[0]
        eerste_resultaat_id = eerste_detectie.results[0].id

        if eerste_resultaat_id == 0:
            rospy.loginfo("Resultaat van de herkenning is: Colgate")
            item = "Colgate_0"
            naam = "Colgate"
        elif eerste_resultaat_id == 1:
            rospy.loginfo("Resultaat van de herkenning is: Elektrisch")
            item = "Elektrisch_0"
            naam = "Elektrisch"
        elif eerste_resultaat_id == 2:
            rospy.loginfo("Resultaat van de herkenning is: PepaPig")
            item = "PepaPig_0"
            naam = "PepaPig"
        elif eerste_resultaat_id == 3:
            rospy.loginfo("Resultaat van de herkenning is: TongBorstel")
            item = "TongBorstel_0"
            naam = "TongBorstel"
        else:
            rospy.logwarn("Received unexpected value: %i", eerste_resultaat_id)
            return lokalisatieResponse(Xw=0, Yw=0, Zw=0, Naam="")

        x_vals.append(eerste_detectie.spatial_coordinates.x)
        y_vals.append(eerste_detectie.spatial_coordinates.y)
        z_vals.append(eerste_detectie.spatial_coordinates.z)

    x_mean = sum(x_vals) / len(x_vals)
    y_mean = sum(y_vals) / len(y_vals)
    z_mean = sum(z_vals) / len(z_vals)

    try:
        # Wacht maximaal 1 seconde op de transform
        listener.waitForTransform('/world', '/camera', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/world', '/camera', rospy.Time(0))

        # Voeg de gemiddelde waarden toe aan de transform
        trans_mean = (trans[0] + x_mean, trans[1] + y_mean, trans[2] + z_mean)
        rospy.loginfo("Gemiddelde translation: %s", str(trans_mean))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF fout: %s", str(e))
        return lokalisatieResponse(Xw=0, Yw=0, Zw=0, Naam=naam)

    return lokalisatieResponse(Xw=trans_mean[0], Yw=trans_mean[1], Zw=trans_mean[2], Naam=naam)

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
