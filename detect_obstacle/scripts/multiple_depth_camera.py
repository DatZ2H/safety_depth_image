#!/usr/bin/env python
from __future__ import print_function

import roslib
import time
import sys
import rospy
import rospkg
# import pyautogui
import argparse
import message_filters
from detect_obstacle.msg import fields_safety

camera = "/depth_camera"
camera_left = "/depth_camera_left"
camera_right = "/depth_camera_right"
topic_fields_safety_pub = "/fields_safety"
count_dangerous_field = 0

class merge_multiple_depth_camera:
  def __init__(self):
    global camera, camera_left, camera_right, topic_fields_safety_pub
    node_name = rospy.get_name()
    self.getParam(node_name)

    self.fields_safety_pub = rospy.Publisher(camera + topic_fields_safety_pub , fields_safety, queue_size=1)	
    rospy.loginfo("Publish the topic: " + camera + topic_fields_safety_pub)

    self.camera_left_sub = message_filters.Subscriber(camera_left + topic_fields_safety_pub , fields_safety)
    rospy.loginfo("Subscriber the topic: " + camera_left + topic_fields_safety_pub)
    self.camera_right_sub = message_filters.Subscriber(camera_right + topic_fields_safety_pub , fields_safety)
    rospy.loginfo("Subscriber the topic: " + camera_right + topic_fields_safety_pub)
        
    self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_left_sub, self.camera_right_sub], queue_size=5, slop=1)
    self.ts.registerCallback(self.safetyCallback)

  def safetyCallback(self, camera_left, camera_right):
    global camera, count_dangerous_field
    fields_safety_ = fields_safety()
    fields_safety_.header.frame_id = camera
    if camera_left.system_good == True and camera_right.system_good == True:
      fields_safety_.system_good = True
    if camera_left.enable == True and camera_right.enable == True:
      fields_safety_.enable = True
    else:
      fields_safety_.enable = False
    fields_safety_.time_detect = (camera_left.time_detect+camera_right.time_detect)/2
    if camera_left.fields[2] == True or camera_right.fields[2] == True:
      count_dangerous_field = 10
      fields_safety_.fields.append(False)
      fields_safety_.fields.append(False)
      fields_safety_.fields.append(True)
    elif count_dangerous_field == 0:
      if camera_left.fields[1] == True or camera_right.fields[1] == True:
        fields_safety_.fields.append(False)
        fields_safety_.fields.append(True)
        fields_safety_.fields.append(False)
      elif camera_left.fields[0] == True or camera_right.fields[0] == True:
        fields_safety_.fields.append(True)
        fields_safety_.fields.append(False)
        fields_safety_.fields.append(False)
      else:
        fields_safety_.fields.append(False)
        fields_safety_.fields.append(False)
        fields_safety_.fields.append(False)
    elif count_dangerous_field != 0: 
      count_dangerous_field = count_dangerous_field - 1
      if count_dangerous_field < 0:
        count_dangerous_field = 0
      fields_safety_.fields.append(False)
      fields_safety_.fields.append(False)
      fields_safety_.fields.append(True)
    self.fields_safety_pub.publish(fields_safety_)

  def getParam(self, node_name):
    global camera, camera_left, camera_right
    camera = rospy.get_param(node_name + "/camera")
    camera_left = rospy.get_param(node_name + "/camera_left")
    camera_right = rospy.get_param(node_name + "/camera_right")

def main(args):
  rospy.init_node('multiple_depth_camera', anonymous=True)
  rospy.loginfo("create node multiple_depth_camera")
  dist_obj = merge_multiple_depth_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)