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
from helper_scripts.tasks_helper import orientationTask, \
  comTask, positionTask
from helper_scripts.stop_experiment_helper import stopMotion, goHalfSitting
from helper_scripts.abstract_interactive_marker import AbstractInteractiveMarker

"""
This script is a simple demonstration of the multi_contact framework's QP tasks
4 interactive markers are used defining goals for:
  Right Hand
  Left Hand
  Right Foot
  Center of Mass
Additionally, a posture and torso orientation task are added to make the robot
posture 'look good'
"""

# control parameters
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('template_demo_romeo')

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
  rHand = romeo.surfaces['Rhand']
  lHand = romeo.surfaces['Lhand']

  # Target goals
  rf_pos_goal = rFoot.X_0_s(romeo).translation()
  rf_ori_goal = rFoot.X_0_s(romeo).rotation()
  rh_pos_goal = rHand.X_0_s(romeo).translation() # + Vector3d(0.4, -0.2, 0.3) 
  lh_pos_goal = lHand.X_0_s(romeo).translation()  + Vector3d(0.1,  0.1, 0.0)
  lhand_rotation_goal = lHand.X_0_s(romeo).rotation() # sva.RotY(-np.pi/2.)
  rhand_rotation_goal = rHand.X_0_s(romeo).rotation()

  # Setting up the tasks
  postureTask1 = tasks.qp.PostureTask(robots.mbs, romeo_index,
                                      romeo_q, 0.1, 10.)
  rfPosTask, rfPosTaskSp = positionTask(robots, romeo_index, 'r_ankle',
                                        rf_pos_goal,
                                        5., 1000., rFoot.X_b_s.translation())
  rfOriTask, rfOriTaskSp = orientationTask(robots, romeo_index, 'r_ankle', rf_ori_goal,
                                           5., 100.)
  rhPosTask, rhPosTaskSp = positionTask(robots, romeo_index, 'r_wrist',
                                        rh_pos_goal,
                                        5., 1000., rHand.X_b_s.translation())
  rhOriTask, rhOriTaskSp = orientationTask(robots, romeo_index, 'r_wrist', rhand_rotation_goal,
                                         5., 100.)
  lhPosTask, lhPosTaskSp = positionTask(robots, romeo_index, 'l_wrist',
                                        lh_pos_goal,
                                        5., 1000., lHand.X_b_s.translation())
  lhOriTask, lhOriTaskSp = orientationTask(robots, romeo_index, 'l_wrist', lhand_rotation_goal,
                                           5., 100.)
  torsoOriTask, torsoOriTaskSp = orientationTask(robots, romeo_index, 'torso',
                                                 Matrix3d.Identity(), 10., 10.)
  headOriTask, headOriTaskSp = orientationTask(robots, romeo_index, 'HeadRoll_link',
                                                 Matrix3d.Identity(), 10., 10.)

  # move the CoM to the center of the support, TODO: remove this if a better half-sitting is available
  com_init = rbd.computeCoM(romeo.mb, romeo.mbc)
  com_init[0] = 0.
  com_init[1] = 0.
  comTask, comTaskSp = comTask(robots, romeo_index, com_init,
                               5., 100000.)

  # add tasks to the solver
  qpsolver.solver.addTask(rhPosTaskSp)
  qpsolver.solver.addTask(rhOriTaskSp)
  qpsolver.solver.addTask(lhPosTaskSp)
  qpsolver.solver.addTask(lhOriTaskSp)

  qpsolver.solver.addTask(torsoOriTaskSp)
  qpsolver.solver.addTask(headOriTaskSp)
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
                            'wait_com_transfer',
                            'wait_right_foot_lift',
                            'interactive_goals']
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
      if rhPosTask.eval().norm() < 0.1 and rhPosTask.speed().norm() < 0.001 and lhPosTask.eval().norm() < 0.1 and lhPosTask.speed().norm() < 0.001:
        # move the x and y location of the CoM target to the left foot center
        desired_com = rbd.computeCoM(romeo.mb, romeo.mbc)
        lFoot_translation = lFoot.X_0_s(romeo).translation()
        desired_com[0] = lFoot_translation[0]
        desired_com[1] = lFoot_translation[1]
        comTask.com(desired_com)
        self.checkSequence()
        print 'transferring CoM'

    def wait_com_transfer(self, rs):
      if comTask.eval().norm() < 0.01 and comTask.speed().norm() < 0.001:
        # task to lift the right foot
        rfPosTask.position(rfPosTask.position() + Vector3d(0., 0., 0.15))
        qpsolver.solver.addTask(rfPosTaskSp)
        qpsolver.solver.addTask(rfOriTaskSp)
        qpsolver.setContacts([c1L]) #remove right foot contact, keep left foot in contact
        qpsolver.update()
        self.checkSequence()
        print 'lifting right foot'

    def wait_right_foot_lift(self, rs):
      if rfPosTask.eval().norm() < 0.1 and rfPosTask.speed().norm() < 0.001:
        self.init_goal_rh = sva.PTransform(rhand_rotation_goal, rh_pos_goal)
        self.init_goal_lh = sva.PTransform(lhand_rotation_goal, lh_pos_goal)
        self.init_goal_rf = sva.PTransform(rf_ori_goal, rfPosTask.position())
        self.init_goal_com = sva.PTransform(comTask.com())
        self.imarker_rh = AbstractInteractiveMarker('rhand_goal', self.init_goal_rh)
        self.imarker_lh = AbstractInteractiveMarker('lhand_goal', self.init_goal_lh)
        self.imarker_rf = AbstractInteractiveMarker('rfoot_goal', self.init_goal_rf)
        self.imarker_com = AbstractInteractiveMarker('com_goal', self.init_goal_com)
        self.checkSequence()
        print 'interactive mode ON'

    def interactive_goals(self, rs):
      rhPosTask.position(self.imarker_rh.X.translation())
      rhOriTask.orientation(self.imarker_rh.X.rotation())
      lhPosTask.position(self.imarker_lh.X.translation())
      lhOriTask.orientation(self.imarker_lh.X.rotation())
      rfPosTask.position(self.imarker_rf.X.translation())
      rfOriTask.orientation(self.imarker_rf.X.rotation())
      comTask.com(self.imarker_com.X.translation())

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
