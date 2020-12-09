#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from geometry_msgs.msg import PoseStamped, PointStamped
import math

SEARCH_RADIUS = 0.75
def callback(leg, people, area):
    rospy.loginfo("leg timestamp: %d ns" % leg.header.stamp.to_nsec())
    rospy.loginfo("people timestamp: %d ns" % people.header.stamp.to_nsec())
    rospy.loginfo("area timestamp: %d ns" % area.header.stamp.to_nsec())

    center_x = area.point.x
    center_y = area.point.y

    target_name = ""
    target_diff = SEARCH_RADIUS

    posearray_msg = PoseStamped()


    for p in people.people:
        diff = math.hypot(center_x - p.pos.x, center_y - p.pos.y)
        if diff < SEARCH_RADIUS and diff < target_diff:
            target_diff = diff
            target_name = p.object_id
            posearray_msg.pos.position.x = p.pos.x
            posearray_msg.pos.position.y = p.pos.y

    if target_name:

        leg = [[l.pos.x, l.pos.y] for l in leg.people if l.object_id == target_name]

        if len(leg) == 2:

            posearray_msg.header.frame_id = "/base_scan"
            posearray_msg.header.stamp = rospy.Time.now()

            pose_pub.publish(posearray_msg)


    else:
        pass

rospy.init_node('filetering_leg_detector_node')

pub = rospy.Publisher("/output", PoseStamped, queue_size=1)
pose_pub = rospy.Publisher("/filtering/scan/detect_leg_person", PoseStamped, queue_size=1)
sub1 = message_filters.Subscriber("leg_tracker_measurements", PositionMeasurementArray, queue_size=1)
sub2 = message_filters.Subscriber("people_tracker_measurements", PositionMeasurementArray, queue_size=1)
sub3 = message_filters.Subscriber("target_point", PointStamped, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], 10, 1)
ts.registerCallback(callback)

rospy.spin()
