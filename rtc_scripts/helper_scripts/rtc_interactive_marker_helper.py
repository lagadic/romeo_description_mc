'''
This is an RTC compatible copy of the interactive marker from mc_ros/mc_control
'''

from geometry_msgs.msg import Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
    #InteractiveMarkerControl, InteractiveMarker, Marker
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker

from eigen import Vector3d, Matrix3d
import rtc_transform


def createBoxMarker(scale=0.1, rgb=(1., 0., 0.)):
  marker = Marker()
  marker.type = Marker.CUBE
  marker.scale.x = scale
  marker.scale.y = scale
  marker.scale.z = scale
  marker.color.r = rgb[0]
  marker.color.g = rgb[1]
  marker.color.b = rgb[2]
  marker.color.a = 1.

  return marker


def createMeshMarker(meshpath):
  marker = Marker()
  marker.type = Marker.MESH_RESOURCE
  marker.mesh_resource = meshpath
  marker.color.a = 1.

  return marker


def createTranslationControlMarker(name, axis):
  transControl = InteractiveMarkerControl()
  transControl.name = name
  transControl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  transControl.orientation = Quaternion(axis[0], axis[1], axis[2], 1.)
  return transControl


def createRotationControlMarker(name, axis):
  transControl = InteractiveMarkerControl()
  transControl.name = name
  transControl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  transControl.orientation = Quaternion(axis[0], axis[1], axis[2], 1.)
  return transControl


def createVisInteractive(frame_id, name, desc, X_init, marker):
  # interactive marker
  intMarker = InteractiveMarker()
  intMarker.header.frame_id = frame_id
  intMarker.name = name
  intMarker.description = desc
  intMarker.pose = rtc_transform.toPose(X_init)

  # control marker (box display)
  boxControl = InteractiveMarkerControl()
  boxControl.always_visible = True
  boxControl.markers.append(marker)

  intMarker.controls.append(boxControl)

  return intMarker


def create1DofTransInteractive(frame_id, name, desc, X_init, marker, direction=Vector3d.UnitX()):
  intMarker = createVisInteractive(frame_id, name, desc, X_init, marker)
  tControl = createTranslationControlMarker('trans', direction.T[0])

  intMarker.controls.append(tControl)

  return intMarker


def create3DofInteractive(frame_id, name, desc, X_init, marker, frame=Matrix3d.Identity()):
  intMarker = createVisInteractive(frame_id, name, desc, X_init, marker)

  xControl = createTranslationControlMarker('trans_x', frame[0,:][0])
  yControl = createTranslationControlMarker('trans_y', frame[1,:][0])
  zControl = createTranslationControlMarker('trans_z', frame[2,:][0])

  intMarker.controls.append(xControl)
  intMarker.controls.append(yControl)
  intMarker.controls.append(zControl)

  return intMarker


def create3DofRotInteractive(frame_id, name, desc, X_init, marker, frame=Matrix3d.Identity()):
  intMarker = createVisInteractive(frame_id, name, desc, X_init, marker)

  xrControl = createRotationControlMarker('rot_x', frame[0,:][0])
  yrControl = createRotationControlMarker('rot_y', frame[1,:][0])
  zrControl = createRotationControlMarker('rot_z', frame[2,:][0])

  intMarker.controls.append(xrControl)
  intMarker.controls.append(yrControl)
  intMarker.controls.append(zrControl)

  return intMarker


def create6DofInteractive(frame_id, name, desc, X_init, marker, frame=Matrix3d.Identity()):
  intMarker = create3DofInteractive(frame_id, name, desc, X_init, marker, frame)

  xrControl = createRotationControlMarker('rot_x', frame[0,:][0])
  yrControl = createRotationControlMarker('rot_y', frame[1,:][0])
  zrControl = createRotationControlMarker('rot_z', frame[2,:][0])

  intMarker.controls.append(xrControl)
  intMarker.controls.append(yrControl)
  intMarker.controls.append(zrControl)

  return intMarker
