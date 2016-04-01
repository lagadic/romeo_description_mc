#! /usr/bin/env python
import roslib; roslib.load_manifest('mc_control')
import rospy
import numpy as np
from eigen3 import Vector3d, Matrix3d, toEigenX
import spacevecalg as sva
import rbdyn as rbd
import tasks
from mc_rbdyn import loadRobots, rbdList, MRContact
from mc_solver import MRQPSolver, DynamicsConstraint, ContactConstraint, \
  KinematicsConstraint
from joint_state_publisher import JointStatePublisher
from mc_robot_msgs.msg import MCRobotState
from ask_user import ask_user

import sys
import copy

# private helper scripts within the package
from helper_scripts.tasks_helper import orientationTask, \
  comTask, positionTask, pbvsTask
from helper_scripts.stop_experiment_helper import stopMotion, goHalfSitting
from helper_scripts.plot_task_error import TaskErrorPub
from helper_scripts.fake_vision import fakeVision

"""
This script is used to test a position-based visual servoing task in the
multi_contact framework as a QP task. Two robots are used one to represent the
internal knowledge which is imprecise and another to represent reality. An
interactive object is used to define the goal.
"""

# control parameters
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('test_visservo_grasp')
  test_mode = sys.argv[1]
  print 'usage mode is ', test_mode
  if test_mode not in ['normal','posTask','surface']:
    rospy.logwarn('Invalid test_mode, defaulting to normal\
       \nusage: test_visservo_grasp.py test_mode\
       \nwhere test_mode can be: \
       \n normal - servo right wrist with PBVS \
       \n posTask - servo right_wrist with position Task equivalent to PBVS \
       \n surface - servo the right_hand surface with PBVS')

  # load the robot and the environment
  robots = loadRobots()
  for r in robots.robots:
    r.mbc.gravity = Vector3d(0., 0., 9.81)

  romeo_index = 0
  env_index = 1
  romeo_real_index = 2

  romeo = robots.robots[romeo_index]
  env = robots.robots[env_index]
  romeo_real = robots.robots[romeo_real_index]

  # compute foot position to be in contact with the ground
  for rob in [romeo, romeo_real]:
    rbd.forwardKinematics(rob.mb, rob.mbc)
    tz = -rob.surfaces['Lfoot'].X_0_s(rob).translation().z()
    tx = -rob.surfaces['Lfoot'].X_0_s(rob).translation().x() #zero the feet surface
    rob_q = rbdList(rob.mbc.q)

    rob_q[0] = [1., 0., 0., 0., tx, 0., tz]
    rob.mbc.q = rob_q
    if rob is romeo:
      romeo_q = rob_q

  # compute init fk and fv
  for r in robots.robots:
    rbd.forwardKinematics(r.mb, r.mbc)
    rbd.forwardVelocity(r.mb, r.mbc)

  romeoJsp = JointStatePublisher(romeo)
  romeo_real_Jsp = JointStatePublisher(romeo_real)

  # create solver
  qpsolver = MRQPSolver(robots, timeStep)

  # add dynamics constraint to QPSolver
  # Use 50% of the velocity limits cf Sebastien Langagne.
  contactConstraint = ContactConstraint(timeStep, ContactConstraint.Position)
  dynamicsConstraint1 = DynamicsConstraint(robots, romeo_index, timeStep,
                                           damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  kinConstraint1 = KinematicsConstraint(robots, romeo_index, timeStep,
                                        damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  qpsolver.addConstraintSet(contactConstraint)
  qpsolver.addConstraintSet(dynamicsConstraint1)

  # Useful robot surfaces
  rFoot = romeo.surfaces['Rfoot']
  lFoot = romeo.surfaces['Lfoot']
  rHand = romeo.surfaces['Rhand']
  lHand = romeo.surfaces['Lhand']

  # Initial target goals (arbitrary)
  rf_pos_goal = rFoot.X_0_s(romeo).translation()
  rf_ori_goal = rFoot.X_0_s(romeo).rotation()
  rh_pos_goal = rHand.X_0_s(romeo).translation()# + Vector3d(0.1,  0.1, 0.0) #+ Vector3d(0.1, -0.1, 0.3)
  lh_pos_goal = lHand.X_0_s(romeo).translation()# + Vector3d(0.1,  0.1, 0.3)
  hand_rotation_goal = rHand.X_0_s(romeo).rotation()
  #lHand.X_0_s(romeo).rotation() #sva.RotY(-np.pi/2.)

  # Setting up the tasks
  postureTask1 = tasks.qp.PostureTask(robots.mbs, romeo_index,
                                      romeo_q, 0.1, 10.)
  rfPosTask, rfPosTaskSp = positionTask(robots, romeo_index, 'r_ankle',
                                        rf_pos_goal,
                                        5., 1000., rFoot.X_b_s.translation())
  rfOriTask, rfOriTaskSp = orientationTask(robots, romeo_index, 'r_ankle', rf_ori_goal,
                                           5., 100.)
  rhOriTask, rhOriTaskSp = orientationTask(robots, romeo_index, 'r_wrist', hand_rotation_goal,
                                           5., 100.)
  torsoOriTask, torsoOriTaskSp = orientationTask(robots, romeo_index, 'torso',
                                                 Matrix3d.Identity(), 10., 10.)
  comTask, comTaskSp = comTask(robots, romeo_index, rbd.computeCoM(romeo.mb, romeo.mbc),
                               5., 100000.)

  # Set up tasks specific to test_mode
  if test_mode == 'posTask':
    posTask_X_b_s = Vector3d.Zero()
  else:
    posTask_X_b_s = rHand.X_b_s.translation()
    if test_mode == 'surface':
      pbvs_X_b_s = rHand.X_b_s
    else:
      pbvs_X_b_s = sva.PTransformd.Identity()
    rhPbvsTask, rhPbvsTaskSp = pbvsTask(robots, romeo_index, 'r_wrist',
                                        sva.PTransformd.Identity(),
                                        5., 1000., pbvs_X_b_s)

  rhPosTask, rhPosTaskSp = positionTask(robots, romeo_index, 'r_wrist',
                                        rh_pos_goal,
                                        5., 1000., posTask_X_b_s)
  if test_mode == 'posTask':
    task_error_to_pub = rhPosTask
  else:
    task_error_to_pub = rhPbvsTask

  # disable the CoM height
  com_axis_weight = np.mat([1., 1., 0.]).T
  comTaskSp.dimWeight(toEigenX(com_axis_weight))

  # add tasks to the solver
  qpsolver.solver.addTask(rhPosTaskSp)
  qpsolver.solver.addTask(rhOriTaskSp)

  qpsolver.solver.addTask(torsoOriTaskSp)
  qpsolver.solver.addTask(comTaskSp)
  qpsolver.solver.addTask(postureTask1)

  # setup all
  c1L = MRContact(romeo_index, env_index,
                  lFoot, env.surfaces['AllGround'])
  c1R = MRContact(romeo_index, env_index,
                  rFoot, env.surfaces['AllGround'])

  qpsolver.setContacts([c1L, c1R])
  qpsolver.update()

  class Controller(object):
    def __init__(self):
      self.isRunning = True
      self.stopCB = ask_user.askUserNonBlock('stop_control', 'Stop')

      # initialize fake Vision
      self.X_gaze_hand = sva.PTransformd(Vector3d.Zero()) #TODO: better init
      self.X_gaze_object = sva.PTransformd(Vector3d.Zero()) #TODO: better init
      self.hand_frame = '/'+str(romeo_real_index)+'/r_wrist'
      self.object_frame = '/object/base_link'
      if test_mode == 'posTask':
        self.tf_base_frame = '/map'
      else:
        self.tf_base_frame = '/'+str(romeo_index)+'/CameraLeftEye_optical_frame'
      self.fake_vision = fakeVision(self.tf_base_frame)

      # initialize joint names for adding uncertainty
      self.joint_names = map(lambda x: x.name(), list(romeo_real.mb.joints()))

      # for plotting task error
      self.task_error_pub = TaskErrorPub(task_error_to_pub, 'rh_PBVS_task')

      # sequence of states - each must correspond to a method of this object
      self.fsm_sequence  = ['wait_init_position',
                            'visual_servo_grasp']
      self.checkSequence()

    def fakeControl(self):
      """
      Copy the robot state / posture of the controlled internal model to the "real" one
      """
      romeo_real.mbc = copy.copy(romeo.mbc)

    def addUncertainties(self):
      """
      Adds bias and noise to the relevant portions
      TODO:
        - more modular bias
        - something more generic that can handle the free-flyer change properly
        - noise
      """
      new_posture = rbdList(romeo_real.mbc.q)
#      new_posture[self.joint_names.index('CHEST_P')] = [new_posture[self.joint_names.index('CHEST_P')][0] - 0.5]
      new_posture[self.joint_names.index('RShoulderPitch')] = [new_posture[self.joint_names.index('RShoulderPitch')][0] - 0.5]
#      new_posture[self.joint_names.index('R_SHOULDER_R')] = [new_posture[self.joint_names.index('R_SHOULDER_R')][0] + 0.3]

      romeo_real.mbc.q = new_posture

    # check if there are still states in the FSM to be executed
    def checkSequence(self):
      if self.fsm_sequence:
        print 'Sequence left: ', self.fsm_sequence
        self.fsm = getattr(self, self.fsm_sequence.pop(0))
      else:
        self.fsm = self.idle
        print 'idling'

    def wait_init_position(self, rs):
      self.fakeControl()
      self.addUncertainties()
      if rhPosTask.eval().norm() < 0.1 and rhPosTask.speed().norm() < 0.001:
        if test_mode != 'posTask':
          # remove the right hand positioning task and replace it with a position based visual servoing task
          qpsolver.solver.removeTask(rhPosTaskSp)
          qpsolver.solver.removeTask(rhOriTaskSp)
          qpsolver.solver.addTask(rhPbvsTaskSp)
          qpsolver.update()
        self.checkSequence()

    def visual_servo_grasp(self, rs):
      self.fakeControl()
      self.addUncertainties()

      if test_mode == 'posTask':
        # test: update the position task instead
        X_world_object = self.fake_vision(self.object_frame)
        X_world_hand_real = self.fake_vision(self.hand_frame)
        X_world_hand = self.fake_vision('/'+str(romeo_index)+'/r_wrist') # internal knowledge

        #note: compensate the internal state update by re-adding it
        rhPosTask.position(X_world_object.translation() - X_world_hand_real.translation() + X_world_hand.translation())

      else:
        self.X_gaze_hand = self.fake_vision(self.hand_frame)
        self.X_gaze_object = self.fake_vision(self.object_frame)
        # update PBVS scheme with the pose of the real hand in the target frame
        rhPbvsTask.error(pbvs_X_b_s * self.X_gaze_hand * self.X_gaze_object.inv())

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
        romeo_real_Jsp.publish(curTime)
        self.task_error_pub.publish()
        qpsolver.send(curTime)

        self.fsm(rs)
#        if not (self.fsm == self.wait_init_position):
#          raw_input('wait user input')

    # FSM state: after stopped go back to half-sitting
    def waitHS(self, rs):
      if self.hsCB is not None and self.hsCB.check():
        self.hsCB = None
        goHalfSitting(qpsolver, postureTask1, romeo_q, \
                      [dynamicsConstraint1, contactConstraint], \
                      [kinConstraint1])
        self.fsm = self.idle
        print 'idling'

    def idle(self, rs):
      pass

  ask_user.askUser('start', 'Start')
  controller = Controller()
  rospy.Subscriber('/robot/sensors/robot_state', MCRobotState,
                   controller.run, queue_size=10, tcp_nodelay=True)
  rospy.spin()
