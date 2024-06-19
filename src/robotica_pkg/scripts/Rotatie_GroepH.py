#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from depthai_ros_msgs.msg import SpatialDetectionArray

class ImageProcessor:

    def __init__(self):
        self.display_image = True
        self.bridge = CvBridge()
        self.roi = (0, 0, 100, 100)  # Default ROI values
        self.pcl_sub = rospy.Subscriber("/stereo_inertial_nn_publisher/color/image", Image, self.image_callback)
        self.detections_sub = rospy.Subscriber("/stereo_inertial_nn_publisher/color/detections", SpatialDetectionArray, self.detections_callback)

    def detections_callback(self, msg):
        if len(msg.detections) > 0:
            detection = msg.detections[0]
            bbox = detection.bbox
            x = int(bbox.center.x - bbox.size_x / 2)
            y = int(bbox.center.y - bbox.size_y / 2)
            w = int(bbox.size_x)
            h = int(bbox.size_y)
            self.roi = (x, y, w, h)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        self.process_image(cv_image)
    
    def process_image(self, img):
        x, y, w, h = self.roi
        img_roi = img[y:y+h, x:x+w]
        imgscale = cv2.resize(img_roi, (0, 0), fx=2, fy=2)
        imgBlur = cv2.GaussianBlur(imgscale, (15, 15), 5)
        imgCanny = cv2.Canny(imgBlur, 25, 75)
        kernel_Dil = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        kernel_Erode = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        imgDil = cv2.dilate(imgCanny, kernel_Dil)
        imgErode = cv2.erode(imgDil, kernel_Erode)

        # Fix for OpenCV 4.x: cv2.findContours returns only contours and hierarchy
        contours, hierarchy = cv2.findContours(imgErode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        numBlob = len(contours)
        if numBlob == 0:
            print("Error: No contours found.")
            return

        textm = "Er zijn " + str(numBlob)
        cv2.putText(imgscale, textm, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 50), 2)

        # Find the largest contour
        maxArea = 0
        maxAreaIndex = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > maxArea:
                maxArea = area
                maxAreaIndex = i

        warped = None
        if maxAreaIndex != -1:
            # Draw the largest contour
            cv2.drawContours(imgscale, contours, maxAreaIndex, (0, 0, 255), 2)

            # Create a rectangle around the largest contour
            rect = cv2.minAreaRect(contours[maxAreaIndex])

            angle = rect[2]
            if rect[1][0] < rect[1][1]:
                angle += 90

            angleText = "Angle: " + str(angle) + " degrees"
            cv2.putText(imgscale, angleText, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            print(angle)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            cv2.drawContours(imgscale, [box], 0, (255, 50, 200), 2)

            # Warp the image to get a straight view of the rectangle
            width, height = int(rect[1][0]), int(rect[1][1])
            src_pts = np.array(box, dtype="float32")
            dst_pts = np.array([[0, height - 1],
                                [0, 0],
                                [width - 1, 0],
                                [width - 1, height - 1]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped = cv2.warpPerspective(imgscale, M, (width, height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        else:
            print("Error: No valid contour found.")

        if self.display_image:
            cv2.imshow("Image window", img)
            cv2.imshow("ImgScale", imgscale)
            cv2.imshow("imgErode", imgErode)
            if warped is not None:
                cv2.imshow("imgWarped", warped)

            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('image_processor')
    # Create the image processor object
    ip = ImageProcessor()
    
    # Keep the program alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
