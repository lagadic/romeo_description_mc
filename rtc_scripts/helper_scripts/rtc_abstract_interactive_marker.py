import rospy

from rtc_interactive_marker_helper import createBoxMarker, create6DofInteractive, \
  create1DofTransInteractive, createVisInteractive, createTranslationControlMarker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker

import rtc_transform
from eigen import Vector3d, Matrix3d

import sva

# interactive markers to define goals
class AbstractInteractiveMarker(object):
  def __init__(self, name, X_init, box_scale=0.1, control_scale=1.):
    self.name = name
    self.X = X_init
    self.marker = createBoxMarker(scale=box_scale)
    self.interactive_marker = create6DofInteractive('/robot_map', name, name,
                                                    X_init, self.marker)
    self.interactive_marker.scale = control_scale
    self.marker_server = InteractiveMarkerServer(name + 'server')
    self.marker_server.insert(self.interactive_marker, self.markerMoved)
    self.marker_server.applyChanges()

  def markerMoved(self, feedback):
    self.X = rtc_transform.fromPose(feedback.pose)

# text display
class TextMarker(object):
  def __init__(self, name, frame_id, pos_offset):
    # text display of mass
    self.text_marker = Marker()
    self.text_marker.header.frame_id = frame_id
    self.text_marker.id = 10 # TODO: make sure unique
    self.text_marker.type = Marker.TEXT_VIEW_FACING
    self.text_marker.action = Marker.ADD

    self.text_marker.pose.position.x = pos_offset[0]
    self.text_marker.pose.position.y = pos_offset[1]
    self.text_marker.pose.position.z = pos_offset[2]

    self.text_marker.pose.orientation.x = 0.0
    self.text_marker.pose.orientation.y = 0.0
    self.text_marker.pose.orientation.z = 0.0
    self.text_marker.pose.orientation.w = 1.0

    self.text_marker.scale.x = .1
    self.text_marker.scale.y = .1
    self.text_marker.scale.z = .1

    self.text_marker.color.r = 1.
    self.text_marker.color.g = 0.
    self.text_marker.color.b = 0.
    self.text_marker.color.a = 1.0

    self.text_marker.text = ''
    self.text_marker_ros_publisher = rospy.Publisher(name+'_text', Marker)

  def update(self, text):
    self.text_marker.text = text

  def publish(self):
    self.text_marker_ros_publisher.publish(self.text_marker)

# interactive marker to control mass
class MassInteractiveMarker(object):
  def __init__(self, name, frame_id, X_offset, text_offset, mass, scale):
    self.name = name
    self.mass = mass
    self.scale = scale

    # interactive marker
    self.marker = createBoxMarker(scale=0.001)
    self.interactive_marker = create1DofTransInteractive(frame_id, name, name+' control',
                                X_offset, self.marker, direction=-Vector3d.UnitY())

    self.marker_server = InteractiveMarkerServer(name + 'server')
    self.marker_server.insert(self.interactive_marker, self.markerMoved)
    self.marker_server.applyChanges()

    self.text_marker = TextMarker(name, frame_id, text_offset)

  def markerMoved(self, feedback):
    self.mass = abs(feedback.pose.position.z) * self.scale
    self.text_marker.update(str(self.mass) + ' kg')
    self.text_marker.publish()

# interactive marker for 2D plane
# TODO: for now axes list must be 2 items with 0,1,2 representing x,y,z
class PlanarInteractiveMarker(object):
  def __init__(self, name, frame_id, X_offset, text_offset, scale, axes_list=[0,1]):
    self.name = name
    self.scale = scale
    self.force_list = [0.] * len(axes_list)
    self.axes_list = axes_list

    # interactive marker
    self.marker = createBoxMarker(scale=0.001)
    intMarker = createVisInteractive(frame_id, name, name+' 2dcontrol', X_offset, self.marker)
    frame = Matrix3d.Identity()

    for i in axes_list:
      control = createTranslationControlMarker('control'+str(i), frame[i,:].tolist()[0])
      intMarker.controls.append(control)

    self.marker_server = InteractiveMarkerServer(name + 'server')
    self.marker_server.insert(intMarker, self.markerMoved)
    self.marker_server.applyChanges()

    # text display of forces
    self.text_marker = TextMarker(name, frame_id, text_offset)
    self.updateText()

  def updateText(self):
    self.text_marker.update(str(self.force_list[0]) + ' N \n' \
      + str(self.force_list[1]) + ' N')

  def markerMoved(self, feedback):
    #TODO: better coding, check the ordering of x,y,z
    for i in self.axes_list:
      if i == 0:
        self.force_list[i] = feedback.pose.position.x * self.scale
      elif i == 1:
        self.force_list[i] = feedback.pose.position.z * self.scale
      elif i == 2:
        self.force_list[i] = feedback.pose.position.y * self.scale
      else:
        print 'error, expected 0-2'

    self.updateText()
    self.text_marker.publish()

# interactive marker for a 6-dof wrench
# Note: there is a "bug" because of the rotation singularities where +/- sign instantaneously switches
#       a workaround is to increase the scale to have higher inputs far from singularities
class WrenchInteractiveMarker(object):
  def __init__(self, name, frame_id, X_init, text_offset, scale):
    self.name = name
    self.marker = createBoxMarker()
    self.interactive_marker = create6DofInteractive(frame_id, name, name,
                                                    X_init, self.marker)
    self.marker_server = InteractiveMarkerServer(name + 'server')
    self.marker_server.insert(self.interactive_marker, self.markerMoved)
    self.marker_server.applyChanges()
    self.text_marker = TextMarker(name, frame_id, text_offset)

    self.X_init_inv = X_init.inv()
    self.scale = scale
    self.forces = Vector3d.Zero()
    self.torques = Vector3d.Zero()

  def markerMoved(self, feedback):
    X = rtc_transform.fromPose(feedback.pose) * self.X_init_inv
    self.forces = self.scale[0] * X.translation()
    self.torques = self.scale[1] * sva.rotationError(Matrix3d.Identity(), X.rotation())
    self.text_marker.update(str(self.forces) + '\n' + str(self.torques))
    self.text_marker.publish()