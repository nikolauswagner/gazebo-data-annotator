import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

class GazeboObject():
  def __init__(self):
    self.pose = Pose()
    self.id = 0

  def pos(self):
    return np.array((self.pose.position.x, self.pose.position.y, 
                     self.pose.position.z))

  def rot(self):
    return np.array((self.pose.orientation.x, self.pose.orientation.y,
                     self.pose.orientation.z, self.pose.orientation.w))

# Simulated Gazebo camera
class RealsenseCamera(GazeboObject):
  def __init__(self, camera_name='realsense_plugin'):
    # Metainformation
    self.info_rgb   = CameraInfo()
    self.info_depth = CameraInfo()
    self.offs_rgb   = np.array((0, -0.046, 0.004))
    self.offs_depth = np.array((0, -0.030, 0.004))

    # Callbacks
    self.info_rgb_sub   = rospy.Subscriber("/" + camera_name + "/camera/color/camera_info", 
                                           CameraInfo, self.infoRGBCB)
    self.info_depth_sub = rospy.Subscriber("/" + camera_name + "/camera/depth_registered/camera_info", 
                                           CameraInfo, self.infoDepthCB)
    self.img_rgb_sub    = rospy.Subscriber("/" + camera_name + "/camera/color/image_raw", 
                                           Image, self.imgRGBCB)
    self.img_depth_sub  = rospy.Subscriber("/" + camera_name + "/camera/depth_registered/image_raw", 
                                           Image, self.imgDepthCB)

    # Image storage
    self.bridge = CvBridge()
    self.img_rgb   = None
    self.img_depth = None
    self.img_ready_rgb   = False
    self.img_ready_depth = False

  def infoRGBCB(self, info):
    self.info_rgb = info

  def infoDepthCB(self, info):
    self.info_depth = info

  def imgRGBCB(self, img):
    self.img_rgb       = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    self.img_ready_rgb = True

  def imgDepthCB(self, img):
    self.img_depth       = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    self.img_ready_depth = True

class SegmentationCamera(RealsenseCamera):
  def __init__(self, camera_name="segmentation_plugin"):
    # Metainformation
    RealsenseCamera.__init__(self, camera_name)
    self.info_seg   = CameraInfo()
    self.offs_rgb   = np.array((0, 0, 0))
    self.offs_depth = np.array((0, 0, 0))

    # Callbacks
    self.info_depth_sub = rospy.Subscriber("/" + camera_name + "/camera/depth/camera_info",
                                         CameraInfo, self.infoDepthCB)
    self.img_depth_sub  = rospy.Subscriber("/" + camera_name + "/camera/depth/image_raw",
                                         Image, self.imgDepthCB)
    self.info_seg_sub   = rospy.Subscriber("/" + camera_name + "/camera/segmentation/camera_info",
                                           CameraInfo, self.infoSegCB)
    self.img_seg_sub    = rospy.Subscriber("/" + camera_name + "/camera/segmentation/image_raw",
                                           Image, self.imgSegCB)

    # Image storage
    self.img_seg       = None
    self.img_ready_seg = False

  def infoSegCB(self, info):
    self.info_seg = info

  def imgSegCB(self, img):
    self.img_seg       = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    self.img_ready_seg = True