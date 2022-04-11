#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, ObjectDistance, ObjectDistances
import math
import numpy as np


class ObjectCenterDistance:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.bounding_boxes = None
        rospy.init_node('ObjectDepth', anonymous=True)
        # rospy.Subscriber('/camera/color/image_raw', Image, self.rawImageCallback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthColorCallback)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.depth_publisher = rospy.Publisher('/darknet_ros/distance', ObjectDistances, queue_size = 1)
        freq = 30
        rate = rospy.Rate(freq)
        rospy.spin()
        cv2.destroyAllWindows()

    '''
    depth 거리 계산
    '''
    def depthColorCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            depth_array = np.array(self.depth_image, dtype=np.float32)
            if self.bounding_boxes != None:
                objectDistances = []
                for bounding_box in self.bounding_boxes:
                    xmin = bounding_box.xmin
                    xmax = bounding_box.xmax
                    ymin = bounding_box.ymin
                    ymax = bounding_box.ymax
                    centerX = math.round((xmin + xmax) / 2)
                    centerY = math.round((ymin + ymax) / 2)
                    objectDistance = ObjectDistance()
                    objectDistance.probability = bounding_box.probability
                    objectDistance.Class = bounding_box.Class
                    objectDistance.distance = depth_array[centerX, centerY] 
                    objectDistances.append(objectDistance)
                self.depth_publisher.publish(objectDistances)
        except CvBridgeError as e:
            print(e)

    def bbox_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes

        
if __name__ == '__main__':
    ObjectCenterDistance()