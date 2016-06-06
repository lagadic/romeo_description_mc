#! /usr/bin/env python
import rospy
import numpy as np
from eigen3 import Vector3d, Matrix3d, toEigenX, Vector2d
import spacevecalg as sva
import rbdyn as rbd
import tasks
from mc_rbdyn import loadRobots, rbdList, MRContact
from mc_solver import MRQPSolver, DynamicsConstraint, ContactConstraint, \
  KinematicsConstraint, CollisionsConstraint, Collision
from joint_state_publisher import JointStatePublisher
from mc_robot_msgs.msg import MCRobotState
from ask_user import ask_user
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int8, Bool

import sys

import tf
from mc_ros_utils import transform

# private helper scripts within the package
from helper_scripts.tasks_helper import orientationTask, comTask
from helper_scripts.stop_experiment_helper import stopMotion, goHalfSitting
from helper_scripts.plot_task_error import TaskErrorPub
from helper_scripts.fake_vision import fakeVision

"""
This script demonstrates a gaze task in a whole body controller by using
visual servoing on a single image point
"""

# control parameters
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('test_gaze_task_tracker')
  tracker_type = sys.argv[1]
  print 'Tracker used is ', tracker_type
  if tracker_type not in ['visp_auto_tracker','whycon']:
    rospy.logwarn('Invalid tracker, defaulting to visp_auto_tracker\
      \nusage: test_gaze_task_tracker.py tracker_type\
      \nwhere tracker_type can be: \
      \n visp_auto_tracker \
      \n whycon ')


  # load the robot and the environment
  robots = loadRobots()
  for r in robots.robots:
    r.mbc.gravity = Vector3d(0., 0., 9.81)

  romeo_index = 0
  env_index = 1

  romeo = robots.robots[romeo_index]
  env = robots.robots[env_index]

  # compute foot position to be in contact with the ground
  rbd.forwardKinematics(romeo.mb, romeo.mbc)
  tz = -romeo.surfaces['Lfoot'].X_0_s(romeo).translation().z()
  tx = -romeo.surfaces['Lfoot'].X_0_s(romeo).translation().x() #zero the feet surface
  romeo_q = rbdList(romeo.mbc.q)

  romeo_q[0] = [1., 0., 0., 0., tx, 0., tz]
  romeo.mbc.q = romeo_q

  # compute init fk and fv
  for r in robots.robots:
    rbd.forwardKinematics(r.mb, r.mbc)
    rbd.forwardVelocity(r.mb, r.mbc)

  romeoJsp = JointStatePublisher(romeo)

  # create solver
  qpsolver = MRQPSolver(robots, timeStep)

  # add dynamics constraint to QPSolver
  # Use 50% of the velocity limits cf Sebastient Langagne.
  contactConstraint = ContactConstraint(timeStep, ContactConstraint.Position)
  dynamicsConstraint1 = DynamicsConstraint(robots, romeo_index, timeStep,
                                           damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  kinConstraint1 = KinematicsConstraint(robots, romeo_index, timeStep,
                                        damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  qpsolver.addConstraintSet(contactConstraint)
  qpsolver.addConstraintSet(dynamicsConstraint1)

  # Self-collision robot
  cols = []
  cols += [Collision('torso', 'HeadRoll_link', 0.06, 0.017, 0.),# 0.1, 0.02, 0.
           Collision('l_wrist', 'torso', 0.05, 0.01, 0.), 
           Collision('l_wrist', 'body', 0.05, 0.01, 0.),
           Collision('l_wrist', 'LThigh', 0.05, 0.01, 0.),
           Collision('r_wrist', 'torso', 0.05, 0.01, 0.),
           Collision('r_wrist', 'body', 0.05, 0.01, 0.),
           Collision('r_wrist', 'RThigh', 0.05, 0.01, 0.),
          ]

  r1SelfCollisionConstraint = CollisionsConstraint(robots, romeo_index,
                                                   romeo_index, timeStep)
  r1SelfCollisionConstraint.addCollisions(robots, cols)

  qpsolver.addConstraintSet(r1SelfCollisionConstraint)


  # Setting up tasks for balance and posture
  postureTask1 = tasks.qp.PostureTask(robots.mbs, romeo_index,
                                      romeo_q, 0.1, 10.)
  torsoOriTask, torsoOriTaskSp =\
    orientationTask(robots, romeo_index, 'torso', Matrix3d.Identity(), 10., 10.)

  comTask, comTaskSp = comTask(robots, romeo_index, rbd.computeCoM(romeo.mb, romeo.mbc),
                               5., 10000.)

  # disable the CoM height
  com_axis_weight = np.mat([1., 1., 0.]).T
  comTaskSp.dimWeight(toEigenX(com_axis_weight))

  # add tasks to the solver
  #qpsolver.solver.addTask(torsoOriTaskSp)
  qpsolver.solver.addTask(comTaskSp)
  qpsolver.solver.addTask(postureTask1)

  # setup all
  c1L = MRContact(romeo_index, env_index,
                  romeo.surfaces['Lfoot'], env.surfaces['AllGround'])
  c1R = MRContact(romeo_index, env_index,
                  romeo.surfaces['Rfoot'], env.surfaces['AllGround'])

  qpsolver.setContacts([c1L, c1R])
  qpsolver.update()

  tfListener = tf.TransformListener()
  tfWriter = tf.TransformBroadcaster()

  class Controller(object):
    def __init__(self):
      self.isRunning = True
      self.stopCB = ask_user.askUserNonBlock('stop_control', 'Stop')

      # fake vision to provide the pose using ROS TFs
      self.fake_vision = fakeVision('0/CameraLeftEye_optical_frame')
      # Target pose 
      self.target_pose = sva.PTransformd(Vector3d(0,0,1))
      # Status tracker
      self.status_tracker = 0;

      # gaze task to be created later
      self.gazeTask = []
      self.gazeTaskSp = []

      # sequence of states - each must correspond to a method of this object
      self.fsm_sequence  = ['wait_init_position',
                            'init_gaze_task',
                            'interactive_gaze']
      self.checkSequence()

    # check if there are still states in the FSM to be executed
    def checkSequence(self):
      if self.fsm_sequence:
        print 'Sequence left: ', self.fsm_sequence
        self.fsm = getattr(self, self.fsm_sequence.pop(0))
      else:
        self.fsm = self.idle
        print 'idling'

    def wait_init_position(self, rs):
      if comTask.eval().norm() < 0.05 and comTask.speed().norm() < 0.001 and \
         torsoOriTask.eval().norm() < 0.1 and torsoOriTask.speed().norm() < 0.001:
        self.checkSequence()

    def init_gaze_task(self, rs):
      try:
        (trans, quat) = tfListener.lookupTransform('/0/LEye', '0/CameraLeftEye_optical_frame', rospy.Time(0))
        X_b_gaze = transform.fromTf(trans, quat)

        #X_gaze_object = self.fake_vision('/object/base_link')
        self.gazeTask = tasks.qp.GazeTask(robots.mbs, romeo_index,
                                        robots.robots[romeo_index].bodyIdByName('LEye'),
                                        self.target_pose.translation(), X_b_gaze)
        self.gazeTaskSp = tasks.qp.SetPointTask(robots.mbs, romeo_index, self.gazeTask, 10., 50.)
        qpsolver.solver.addTask(self.gazeTaskSp)

        # for plotting task error
        self.task_error_pub = TaskErrorPub(self.gazeTask, 'gaze_IBVS_task')

        print 'gaze task added'
        self.checkSequence()
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'tf exception'

    def interactive_gaze(self, rs):
      if (self.status_tracker == 1):
        self.gazeTask.error(self.target_pose.translation(), Vector2d(0.0, 0.0)) #center the object in the image frame
      else:
        self.gazeTask.error(Vector2d(0.0, 0.0), Vector2d(0.0, 0.0)) #Stop the robot: the object is not detected
      #print 'eval: ', self.gazeTask.eval()

    # main control loop
    def run(self, rs):
      if self.stopCB is not None and self.stopCB.check():
        print 'stopping'
        self.stopCB = None
        self.isRunning = True
        self.hsCB = stopMotion(robots, qpsolver, postureTask1, None, rbdList(romeo.mbc.q))
        self.fsm = self.waitHS

      if self.isRunning:
        if not qpsolver.run():
          print 'FAIL !!!'
          self.isRunning = False
          return
        curTime = rs.header.stamp

        romeoJsp.publish(curTime)
        if self.fsm == self.interactive_gaze:
          self.task_error_pub.publish()
        qpsolver.send(curTime)

        self.fsm(rs)

    # FSM state: after stopped go back to half-sitting
    def waitHS(self, rs):
      if self.hsCB is not None and self.hsCB.check():
        self.hsCB = None
        goHalfSitting(qpsolver, postureTask1, romeo_q, \
                      [dynamicsConstraint1, contactConstraint], \
                      [kinConstraint1])
        self.fsm = self.idle
        print 'idling'

    def objectVispAutoTrackerPoseCB(self, pose):
      self.target_pose = transform.fromPose(pose.pose)
      #print self.target_pose.translation()
      [tf_tran,tf_quat] = transform.toTf(self.target_pose)
      tfWriter.sendTransform(tf_tran,
                        tf_quat,
                        rospy.Time.now(),
                        "/target",
                        "0/CameraLeftEye_optical_frame")

    def statusVispAutoTrackerCB(self, status):
      if (status.data == 3):
        self.status_tracker = 1
      else:
        self.status_tracker = 0

    def objectWhyconPoseCB(self, poseArray):
      pose = poseArray.poses[0]
      self.target_pose = transform.fromPose(pose)
      #print self.target_pose.translation()
      [tf_tran,tf_quat] = transform.toTf(self.target_pose)
      tfWriter.sendTransform(tf_tran,
                        tf_quat,
                        rospy.Time.now(),
                        "/target",
                        "0/CameraLeftEye_optical_frame")

    def statusWhyconCB(self, status):
      if (status.data == True):
        self.status_tracker = 1
      else:
        self.status_tracker = 0

    def idle(self, rs):
      pass

  ask_user.askUser('start', 'Start')
  controller = Controller()
  rospy.Subscriber('/robot/sensors/robot_state', MCRobotState,
                   controller.run, queue_size=10, tcp_nodelay=True)
  if (tracker_type == 'visp_auto_tracker'):
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped,
                     controller.objectVispAutoTrackerPoseCB, queue_size=10, tcp_nodelay=True)
    rospy.Subscriber('/visp_auto_tracker/status', Int8,
                     controller.statusVispAutoTrackerCB, queue_size=10, tcp_nodelay=True)
  elif (tracker_type == 'whycon'):
    rospy.Subscriber('/whycon/poses', PoseArray,
                     controller.objectWhyconPoseCB, queue_size=10, tcp_nodelay=True)
    rospy.Subscriber('/whycon/status', Bool,
                     controller.statusWhyconCB, queue_size=10, tcp_nodelay=True)

  rospy.spin()