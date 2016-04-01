import rospy
from rospy.numpy_msg import numpy_msg
from comanoid_rennes.msg import NumpyFloat64

from eigen3 import toNumpy

class TaskErrorPub(object):
  '''
  This class allows the publishing of the multi_contact Task errors to ROS
  Args:
    task (QP Task): The low-level task definitions (e.g. PositionTask, OrientationTask,
      CoMTask, etc.)
    topic_name (str): ROS topic name where the errors are published
  '''
  def __init__(self, task, topic_name):
    self.task = task
    self.dim = self.task.dim()
    self.ros_publisher = rospy.Publisher(topic_name, numpy_msg(NumpyFloat64),queue_size=10)

  def publish(self):
    self.ros_publisher.publish(toNumpy(self.task.eval()))