#!/usr/bin/env python
import rospy
import rospkg
import sva
from eigen import Vector3d

#import os
import sys

from helper_scripts.rtc_abstract_interactive_marker import AbstractInteractiveMarker
import helper_scripts.rtc_transform as rtc_transform
import tf

"""
Script that loads an object URDF and adds an interactive marker
to move the object in rviz
"""
if __name__ == '__main__':
  if len(sys.argv) < 8:
    print 'usage: movable_object.py x y z angle_x angle_y angle_z urdf_name \
           \nwhere: \
           \n arg 1-3 are the translation \
           \n arg 4-6 are the rotation angles \
           \n arg 7 is the name of the urdf in this package \
           \n arg 8 is the publishing rate in Hz'
  else:
    rospy.init_node('movable_object')
    # initial transformation of object in the world frame
    rot = [float(x) for x in sys.argv[4:7]]
    trans = [float(x) for x in sys.argv[1:4]]
    X_init = sva.PTransformd(sva.RotX(rot[0])*sva.RotY(rot[1])*sva.RotZ(rot[2]),
                             Vector3d(trans[0], trans[1], trans[2]))

    # load the object urdf
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('comanoid_rennes')
    urdf_name = sys.argv[7]
    with open (base_path+'/urdf/'+urdf_name, 'r') as myfile:
      object_urdf=myfile.read()
    ns = rospy.get_namespace()
    rospy.set_param(ns+'robot_description', object_urdf)

    # interactive marker scaling
    box_marker_scale = 0.01
    control_scale = 0.2
    imarker_obj = AbstractInteractiveMarker('object_pose', X_init, box_marker_scale, control_scale)
    rate = rospy.Rate(float(sys.argv[8]))
    print 'publishing object pose'
    while not rospy.is_shutdown():
      trans, quat = rtc_transform.toTf(imarker_obj.X)
      br = tf.TransformBroadcaster()
      br.sendTransform(trans, quat,
                       rospy.Time.now(),
                       ns+'base_link','/robot_map')
      rate.sleep()