import rospy
import spacevecalg as sva
import tf
from mc_ros_utils import transform

class fakeVision(object):
  '''
  This class simulates the 3D pose estimation by just acquiring the pose from the
  ROS TF messages
  Args:
    gaze_frame (str): Gaze frame defined in ROS TF
  '''
  def __init__(self, gaze_frame):
    self.gaze_frame = gaze_frame
    self.tfListener = tf.TransformListener()

  def __call__(self, target_frame):
    '''
    Take the pose of target_frame using the gaze frame as a reference
      target_frame (str): Target frame defined in ROS TF
    '''
    try:
      (trans, quat) = self.tfListener.lookupTransform(self.gaze_frame, target_frame, rospy.Time(0))
      X_gaze_target = transform.fromTf(trans, quat)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'tf exception'
      X_gaze_target = sva.PTransformd.Identity() #TODO: better fallback
    return X_gaze_target