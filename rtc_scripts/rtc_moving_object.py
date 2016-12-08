#!/usr/bin/env python
import rospy
import rospkg
import sva
from eigen import Vector3d

#import os
import sys

import tf
import helper_scripts.rtc_transform as rtc_transform
import numpy as np
import copy
"""
Script that loads an object URDF and adds a pre-planned path to move the object
"""
if __name__ == '__main__':
  if len(sys.argv) < 10:
    print 'usage: moving_object.py x y z angle_x angle_y angle_z urdf_name \
           \nwhere: \
           \n arg 1-3 are the translation \
           \n arg 4-6 are the rotation angles \
           \n arg 7 is the name of the urdf in this package \
           \n arg 8 is the publishing rate in Hz\
           \n arg 9 is the motion_model (wave or two_target)'
  else:
    rospy.init_node('moving_object')
    # initial transformation of object in the world frame
    rot_base = [float(x) for x in sys.argv[4:7]]
    trans_base = [float(x) for x in sys.argv[1:4]]


    # load the object urdf
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('comanoid_rennes')
    urdf_name = sys.argv[7]
    with open (base_path+'/urdf/'+urdf_name, 'r') as myfile:
      object_urdf=myfile.read()
    rospy.set_param('/object/robot_description', object_urdf)

    motion_model = sys.argv[9]
    if motion_model not in ['wave', 'two_target']:
      rospy.logwarn('in moving_object.py: motion_model not found, defaulting to wave')
      motion_model = 'wave'

    #TODO: find somewhere better to place these motion params
    # sin wave
    amplitude = [0., 0., 0.2] # x,y,z meters
    freq = 0.2 #Hz

    # two target switching
    trans = copy.copy(trans_base)
    rot = copy.copy(rot_base)
    two_target_offset = [0., 0., 0.2] # x,y,z meters
    two_target_time = 5.0 #sec
    two_target_multiplier = 1.0

    publish_rate = float(sys.argv[8])
    rate = rospy.Rate(publish_rate)

    # Main loop
    while not rospy.is_shutdown():
      # sin wave motion
      if motion_model == 'wave':
        for i, (tb, a) in enumerate(zip(trans_base, amplitude)):
          trans[i] = tb + a*np.sin(2*np.pi*freq*rospy.get_time())
      elif motion_model == 'two_target':
        #TODO: there is still a bug from this where it is skipped
        if (rospy.get_time() % two_target_time) < (1./publish_rate):
          for i,_ in enumerate(trans):
            trans[i] = trans_base[i] + two_target_multiplier*two_target_offset[i]
          two_target_multiplier = -two_target_multiplier

      #TODO: inefficient but ok for now
      X_0_obj = sva.PTransformd(sva.RotX(rot[0])*sva.RotY(rot[1])*sva.RotZ(rot[2]),
                             Vector3d(trans[0], trans[1], trans[2]))
      trans_tf, quat_tf = rtc_transform.toTf(X_0_obj)

      br = tf.TransformBroadcaster()
      br.sendTransform(trans, quat_tf,
                       rospy.Time.now(),
                       '/object/base_link','/robot_map')
      rate.sleep()