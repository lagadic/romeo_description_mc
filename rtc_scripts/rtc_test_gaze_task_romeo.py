import mc_rbdyn
import mc_control
import mc_rtc
import time
from eigen import Vector2d, Vector3d, Matrix3d, VectorXd
import sva
import rbdyn as rbd
import numpy as np
import tasks

import rospy
import tf

# private helper scripts within the package
from rtc_tasks_helper import orientationTask, comTask
from rtc_fake_vision import fakeVision
import rtc_transform

"""
This script demonstrates a gaze task in a whole body controller by using
visual servoing on a single image point
"""
class TestGazeTaskRomeo(mc_control.MCPythonController):
  def __init__(self, rm, dt):
    print '--- rtc_test_gaze_task'
    rospy.init_node('rtc_test_gaze_task')

    self.romeo_index = 0
    self.self.env_index = 1

    # Useful surfaces
    self.rFoot = self.robot().surface('Rfoot')
    self.lFoot = self.robot().surface('Lfoot')
    self.rHand = self.robot().surface('Rhand')
    self.lHand = self.robot().surface('Lhand')

    # compute foot position to be in contact with the ground
    rbd.forwardKinematics(self.robot().mb, self.robot().mbc)
    tz = -self.lFoot.X_0_s(self.robot()).translation().z()
    tx = -self.lFoot.X_0_s(self.robot()).translation().x() #zero the feet surface
    romeo_q = self.robot().mbc.q
    romeo_q[0] = [1., 0., 0., 0., tx, 0., tz]
    self.robot().mbc.q = romeo_q
    rospy.set_param('mc_rtc_ticker/init_pos', [1., 0., 0., 0., tx, 0., tz])

    # compute init fk and fv
    rbd.forwardKinematics(self.robot().mb, self.robot().mbc)
    rbd.forwardVelocity(self.robot().mb, self.robot().mbc)

    # Target goals
    self.rf_pos_goal = self.rFoot.X_0_s(self.robot()).translation()
    self.rf_ori_goal = self.rFoot.X_0_s(self.robot()).rotation()
    self.rh_pos_goal = self.rHand.X_0_s(self.robot()).translation() + \
                               Vector3d(0.1, -0.1, 0.3)
    self.lh_pos_goal = self.lHand.X_0_s(self.robot()).translation() + \
                               Vector3d(0.1,  0.1, 0.3)
    self.hand_rotation_goal = sva.RotY(-np.pi/2.)

    # Setting up usual constraints
    self.qpsolver.addConstraintSet(self.kinematicsConstraint)
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.addConstraintSet(self.contactConstraint)

    # Setting up tasks for balance and posture
    self.torsoOriTask, self.torsoOriTaskSp = orientationTask(self.robots(),
      self.romeo_index, 'torso', Matrix3d.Identity(), 10., 10.)
    self.roboComTask, self.roboComTaskSp = comTask(self.robots(), self.romeo_index,
      rbd.computeCoM(self.robot().mb, self.robot().mbc), 5., 10000.)

    # disable the CoM height
    self.roboComTaskSp.dimWeight(VectorXd(1.,1.,0.))

    # add tasks to the solver
    self.postureTask.stiffness(0.1)
    self.postureTask.weight(10.)
    self.qpsolver.addTask(self.postureTask)
    self.qpsolver.addTask(self.torsoOriTaskSp)
    self.qpsolver.addTask(self.roboComTaskSp)

    # set the foot contact
    self.c1L = mc_rbdyn.Contact(self.robots(), self.romeo_index, self.self.env_index,
      'Lfoot', 'AllGround')
    self.c1R = mc_rbdyn.Contact(self.robots(), self.romeo_index, self.self.env_index,
      'Rfoot', 'AllGround')
    self.qpsolver.setContacts([self.c1L, self.c1R])

    # ROS TF listener
    self.tfListener = tf.TransformListener()

    # fake vision to provide the pose using ROS TFs
    self.fake_vision = fakeVision('/control/CameraLeftEye_optical_frame')

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

  def idle(self):
    pass

  def wait_init_position(self):
    if self.roboComTask.eval().norm() < 0.05 and self.roboComTask.speed().norm() < 0.001 and \
       self.torsoOriTask.eval().norm() < 0.1 and self.torsoOriTask.speed().norm() < 0.001:
      self.checkSequence()

  def init_gaze_task(self):
    try:
      (trans, quat) = self.tfListener.lookupTransform('/control/LEye',
        '/control/CameraLeftEye_optical_frame', rospy.Time(0))
      X_b_gaze = rtc_transform.fromTf(trans, quat)

      X_gaze_object = self.fake_vision('/object/base_link')
      self.gazeTask = tasks.qp.GazeTask(self.robots().mbs(), self.romeo_index,
        'LEye', X_gaze_object.translation(), X_b_gaze)
      self.gazeTaskSp = tasks.qp.SetPointTask(self.robots().mbs(), self.romeo_index,
        self.gazeTask, 10., 50.)
      self.qpsolver.addTask(self.gazeTaskSp)

      # for plotting task error
      # TODO: use the rtc logging interface. but also need to change the plotting scripts
#      self.task_error_pub = TaskErrorPub(self.gazeTask, 'gaze_IBVS_task')

      print 'gaze task added'
      self.checkSequence()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'tf exception'

  def interactive_gaze(self):
    X_gaze_object =self.fake_vision('/object/base_link')
    #center the object in the image frame
    self.gazeTask.error(X_gaze_object.translation(), Vector2d(0.0, 0.0))

  def run_callback(self):
    self.fsm()
    return True

  def reset_callback(self, reset_data):
    print 'in reset callback of py script'

  def read_msg_callback(self, msg):
    if msg == "are_you_python":
      return True
    else:
      return False

  def read_write_msg_callback(self, msg):
    return True,msg

  def log_header_callback(self):
    return ';PythonCustomLogValue;ASecondValue'

  def log_data_callback(self):
    return ';NoDataToLog'

  @staticmethod
  def create(robot, dt):
    env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
    return TestGazeTaskRomeo([robot,env], dt)
