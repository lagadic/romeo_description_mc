#!/usr/bin/env python
# Fake Camera Publisher
import rospy
from sensor_msgs.msg import CameraInfo, Image

class fakeCamera(object):
  def __init__(self):
    self.camInfo_Pub = rospy.Publisher('/fake_cam/camera_info', CameraInfo, queue_size=10)
    self.image_Pub = rospy.Publisher('/fake_cam/image', Image, queue_size=10)
    self.h = 480
    self.w = 640
    self.frame = '0/CameraLeftEye_optical_frame'

    # camera params copied from xtion
    self.cam_info_msg = CameraInfo()
    self.cam_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    self.cam_info_msg.K = [570.3422241210938, 0.0, 319.5, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 1.0]
    self.cam_info_msg.P = [570.3422241210938, 0.0, 319.5, 0.0, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    self.cam_info_msg.header.frame_id = self.frame
    self.cam_info_msg.height = self.h
    self.cam_info_msg.width = self.w
    self.cam_info_msg.distortion_model = 'plumb_bob'

    self.image_msg = Image()
    self.image_msg.height = self.h
    self.image_msg.width = self.w
    self.image_msg.header.frame_id = self.frame
    self.image_msg.encoding = 'rgb8'

    # blank all black image
    self.image_msg.data = [0]*(self.h*self.w*3)

  def publish(self, curTime):
    self.cam_info_msg.header.stamp = curTime
    self.image_msg.header.stamp = curTime

    self.camInfo_Pub.publish(self.cam_info_msg)
    self.image_Pub.publish(self.image_msg)

if __name__ == '__main__':
  rospy.init_node('fake_camera_publisher')
  fc = fakeCamera()
  rate = rospy.Rate(200)
  print 'publishing fake camera images'
  while not rospy.is_shutdown():
    fc.publish(rospy.Time.now())
    rate.sleep()