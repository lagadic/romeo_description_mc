'''
This is an RTC compatible copy of transform from mc_ros/mc_ros_utils
'''
from geometry_msgs.msg import Pose, Transform, Quaternion, Vector3, Wrench, Twist
from eigen import Quaterniond, Vector3d
import sva

def fromPose(pose):
  trans = pose.position
  quat = pose.orientation
  return sva.PTransformd(Quaterniond(quat.w, quat.x, quat.y, quat.z).inverse(),
                        Vector3d(trans.x, trans.y, trans.z))

def toPose(X):
  tran = tuple(X.translation())
  quat = tuple(Quaterniond(X.rotation()).inverse().coeffs())
  return Pose(position=Vector3(*tran), orientation=Quaternion(*quat))

def fromTransform(trans):
  tran = trans.translation
  quat = trans.rotation
  return sva.PTransformd(Quaterniond(quat.w, quat.x, quat.y, quat.z).inverse(),
                        Vector3d(tran.x, tran.y, tran.z))

def toTransform(X):
  tran = tuple(X.translation())
  quat = tuple(Quaterniond(X.rotation()).inverse().coeffs())
  return Transform(translation=Vector3(*tran), rotation=Quaternion(*quat))

def fromTf(trans, quat):
  return sva.PTransformd(Quaterniond(quat[3], quat[0], quat[1], quat[2]).inverse(),
                        Vector3d(trans[0], trans[1], trans[2]))

def toTf(pose):
  tran = tuple(pose.translation())
  quat = tuple(Quaterniond(pose.rotation()).inverse().coeffs())
  return tran, quat

def fromWrench(wrench):
  torq = wrench.torque
  forc = wrench.force
  return sva.ForceVec(Vector3d(torq.x, torq.y, torq.z),
                      Vector3d(forc.x, forc.y, forc.z))

def toWrench(X):
  forc = tuple(X.force())
  torq = tuple(X.couple())
  return Wrench(force=Vector3(*forc), torque=Vector3(*torq))

def fromTwist(twist):
  line = twist.linear
  angu = twist.angular
  return sva.MotionVec(Vector3d(angu.x, angu.y, angu.z),
                       Vector3d(line.x, line.y, line.z))

def toTwist(X):
  line = tuple(X.linear())
  angu = tuple(X.angular())
  return Twist(linear=Vector3(*line), angular=Vector3(*angu))