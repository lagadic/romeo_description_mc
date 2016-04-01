#!/usr/bin/env python
import rospy
import rospkg
import spacevecalg as sva
from eigen3 import Vector3d

#import os
import sys

from helper_scripts.abstract_interactive_marker import AbstractInteractiveMarker
import tf
from mc_ros_utils import transform

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
    base_path = rospack.get_path('romeo_mc')
    urdf_name = sys.argv[7]
    with open (base_path+'/urdf/'+urdf_name, 'r') as myfile:
      object_urdf=myfile.read()
    ns = rospy.get_namespace()
    rospy.set_param(ns+'robot_description', object_urdf)

    imarker_obj = AbstractInteractiveMarker('object_pose', X_init, 0.001)
    rate = rospy.Rate(float(sys.argv[8]))
    print 'publishing object pose'
    while not rospy.is_shutdown():
      trans, quat = transform.toTf(imarker_obj.X)
      br = tf.TransformBroadcaster()
      br.sendTransform(trans, quat,
                       rospy.Time.now(),
                       ns+'base_link','/map')
      rate.sleep()