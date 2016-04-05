#! /usr/bin/env python
import roslib; roslib.load_manifest('mc_control')
import rospy
import numpy as np
from eigen3 import Vector3d, Matrix3d
import spacevecalg as sva
import rbdyn as rbd
import tasks
from mc_rbdyn import loadRobots, rbdList, MRContact
from mc_solver import MRQPSolver, DynamicsConstraint, ContactConstraint, \
  KinematicsConstraint, CollisionsConstraint, Collision
from joint_state_publisher import JointStatePublisher
from mc_robot_msgs.msg import MCRobotState
from ask_user import ask_user

# private helper scripts within the package
from helper_scripts.tasks_helper import orientationTask, comTask
from helper_scripts.stop_experiment_helper import stopMotion, goHalfSitting

# for grasping
from mc_grippers import RomeoGripper

"""
This script is a simple demonstration of the multi_contact framework's
mc_gripper for handling the mimic joints of Romeo
"""

# control parameters
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('test_grippers_romeo')

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

  # setup gripper interface
  rGripper = RomeoGripper(RomeoGripper.RIGHT_SIDE, 0.)
  lGripper = RomeoGripper(RomeoGripper.LEFT_SIDE, 0.)

  romeoJsp = JointStatePublisher(romeo, {"rg":rGripper, "lg":lGripper})

  # create solver
  qpsolver = MRQPSolver(robots, timeStep)

  # add dynamics constraint to QPSolver
  # Use 50% of the velocity limits cf Sebastien Langagne.
  contactConstraint = ContactConstraint(timeStep, ContactConstraint.Position)
  dynamicsConstraint1 = DynamicsConstraint(robots, romeo_index, timeStep,
                                           damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  kinConstraint1 = KinematicsConstraint(robots, romeo_index, timeStep,
                                        damper=(0.1, 0.01, 0.5), velocityPercent=0.5)

  # Self-collision robot
  cols = []
  # Collision between arms
  armsBodies = ['{prefL}_wrist', '{prefU}Elbow']
  for lab in map(lambda x: x.format(prefL='l', prefU='L'), armsBodies):
    for rab in map(lambda x: x.format(prefL='r', prefU='R'), armsBodies):
      cols.append(Collision(lab, rab, 0.10, 0.05, 0.))


  cols += [Collision('l_wrist', 'torso', 0.05, 0.01, 0.), #TODO create mesh with l_wrist and fingers
           Collision('l_wrist', 'body', 0.05, 0.01, 0.),
           Collision('l_wrist', 'NeckPitch_link', 0.05, 0.01, 0.),
           Collision('l_wrist', 'HeadRoll_link', 0.05, 0.01, 0.),
           Collision('l_wrist', 'LThigh', 0.05, 0.01, 0.),
           Collision('l_wrist', 'LTibia', 0.05, 0.01, 0.),
           Collision('r_wrist', 'torso', 0.05, 0.01, 0.),
           Collision('r_wrist', 'body', 0.05, 0.01, 0.),
           Collision('r_wrist', 'NeckPitch_link', 0.05, 0.01, 0.),
           Collision('r_wrist', 'HeadRoll_link', 0.05, 0.01, 0.),
           Collision('r_wrist', 'RThigh', 0.05, 0.01, 0.),
           Collision('r_wrist', 'RTibia', 0.05, 0.01, 0.),
           Collision('LElbow', 'torso', 0.05, 0.01, 0.),
           Collision('LElbow', 'body', 0.05, 0.01, 0.),
           Collision('LElbow', 'NeckPitch_link', 0.05, 0.01, 0.),
           Collision('LElbow', 'HeadRoll_link', 0.05, 0.01, 0.),
           Collision('RElbow', 'torso', 0.05, 0.01, 0.),
           Collision('RElbow', 'body', 0.05, 0.01, 0.),
           Collision('RElbow', 'NeckPitch_link', 0.05, 0.01, 0.),
           Collision('RElbow', 'HeadRoll_link', 0.05, 0.01, 0.),
           Collision('RThigh', 'LThigh', 0.01, 0.001, 0.),
           Collision('RTibia', 'LTibia', 0.05, 0.01, 0.),
           Collision('r_ankle', 'l_ankle', 0.05, 0.01, 0.),
           Collision('RThigh', 'LTibia', 0.05, 0.01, 0.),
           Collision('LThigh', 'RTibia', 0.05, 0.01, 0.),
           Collision('r_ankle', 'LTibia', 0.05, 0.01, 0.),
           Collision('l_ankle', 'RTibia', 0.05, 0.01, 0.),
           Collision('r_ankle', 'LThigh', 0.05, 0.01, 0.),
           Collision('l_ankle', 'RThigh', 0.05, 0.01, 0.),
          ]

  r1SelfCollisionConstraint = CollisionsConstraint(robots, romeo_index,
                                                   romeo_index, timeStep)
  r1SelfCollisionConstraint.addCollisions(robots, cols)

  qpsolver.addConstraintSet(contactConstraint)
  qpsolver.addConstraintSet(dynamicsConstraint1)
  qpsolver.addConstraintSet(r1SelfCollisionConstraint)

  # Useful robot surfaces
  rFoot = romeo.surfaces['Rfoot']
  lFoot = romeo.surfaces['Lfoot']

  # Setting up the tasks
  postureTask1 = tasks.qp.PostureTask(robots.mbs, romeo_index,
                                      romeo_q, 0.1, 10.)
  torsoOriTask, torsoOriTaskSp = orientationTask(robots, romeo_index, 'torso',
                                                 Matrix3d.Identity(), 10., 10.)
  comTask, comTaskSp = comTask(robots, romeo_index, rbd.computeCoM(romeo.mb, romeo.mbc),
                               5., 100000.)

  # add tasks to the solver
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

      # sequence of states - each must correspond to a method of this object
      self.fsm_sequence  = ['wait_init_position',
                            'grippers_close_open']
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
      if comTask.eval().norm() < 0.1 and comTask.speed().norm() < 0.001:
        self.checkSequence()

    def grippers_close_open(self, rs):
      # servo the grippers with wave functions oscillating
      # between 0 and 1 (min and max values)
      rGripper.targetHand = 0.5 + 0.5*np.sin(2*np.pi*0.1*rospy.get_time())
      lGripper.targetHand = 0.5 - 0.5*np.sin(2*np.pi*0.1*rospy.get_time())

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

    def idle(self, rs):
      pass

  ask_user.askUser('start', 'Start')
  controller = Controller()
  rospy.Subscriber('/robot/sensors/robot_state', MCRobotState,
                   controller.run, queue_size=10, tcp_nodelay=True)
  rospy.spin()
