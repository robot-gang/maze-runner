#!/usr/bin/env python

import rospy
import message_filters

from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, CameraInfo

def callback(img, cam, ar):
    print('very cool')

def callback2(a):
    print('ultra cool')

rospy.init_node('ar_test')
ar_sub = message_filters.Subscriber('/ar_pose_marker', AlvarMarkers)
cam_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
img_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
ts = message_filters.ApproximateTimeSynchronizer([img_sub,cam_sub, ar_sub], 10, 0.2, allow_headerless=True)
ts.registerCallback(callback)
#rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback2)
rospy.spin()