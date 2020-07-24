import rospy
import numpy as np
import os
import sys
from gazebo_msgs.msg import LinkStates
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_inverse
import tf2_ros
import tf2_geometry_msgs
import cv2
import itertools
from coco_dataset import CocoDataset
from gazebo_tools import *


def rotVecByQuat(v, q):
  return quaternion_multiply(quaternion_multiply(q, np.append(v, 0)), 
                             quaternion_conjugate(q))[:-1]

def projectToImg(pos, cam_info):
  fx = cam_info.K[0]
  fy = cam_info.K[4]
  cx = cam_info.K[2]
  cy = cam_info.K[5]

  x = np.floor(fx * (pos[1] / pos[0]) + cx)
  y = np.floor(fy * (pos[2] / pos[0]) + cy)

  return int(cam_info.width - x), int(cam_info.height - y)


class TrainingDataGenerator():
  def __init__(self, keyword, target_dir, ref_frame='camera1d435e_camera'):
    self.keyword    = keyword
    self.target_dir = target_dir
    self.objects    = []
    self.dataset    = CocoDataset(target_dir + "/dataset.json")
    self.camera     = SegmentationCamera()
    self.states_sub = rospy.Subscriber("/gazebo/link_states", 
                                       LinkStates, self.statesCB)
    self.tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    try:
      os.makedirs(target_dir + "/rgb")
      os.makedirs(target_dir + "/depth")
      os.makedirs(target_dir + "/segmentation")
    except:
      pass

    self.rate = rospy.Rate(1000)
    self.run()

  def statesCB(self, links):
    # Get objects of interest
    self.objects = []
    for idx, name in enumerate(links.name):
      if self.keyword in name:
        obj = GazeboObject()
        obj.pose = links.pose[idx]
        self.objects.append(obj)

    # Get camera pose
    self.camera.pose = links.pose[links.name.index("segmentation_camera::link")]

  def run(self):
    while not rospy.is_shutdown():
      if self.camera.img_ready_rgb and \
         self.camera.img_ready_depth and \
         self.camera.img_ready_seg:

        print("Caught image #" + str(self.dataset.num_imgs))

        # Store images
        filename_rgb = (self.target_dir + "/rgb/" + 
                        str(self.dataset.num_imgs).zfill(4) + ".png")
        cv2.imwrite(filename_rgb, cv2.cvtColor(self.camera.img_rgb, cv2.COLOR_BGR2RGB))
        filename_depth = (self.target_dir + "/depth/" + 
                          str(self.dataset.num_imgs).zfill(4) + ".png")
        cv2.imwrite(filename_depth, self.camera.img_depth.astype(np.uint16))
        filename_seg = (self.target_dir + "/segmentation/" + 
                        str(self.dataset.num_imgs).zfill(4) + ".png")
        cv2.imwrite(filename_seg, self.camera.img_seg.astype(np.uint16))

        # Add image to dataset
        depth = {}
        depth["file_name"] = "depth/" + str(self.dataset.num_imgs).zfill(4) + ".png"
        depth["fx"] = self.camera.info_depth.K[0]
        depth["fy"] = self.camera.info_depth.K[4]
        depth["cx"] = self.camera.info_depth.K[2]
        depth["cy"] = self.camera.info_depth.K[5]
        self.dataset.addImage("rgb/" + str(self.dataset.num_imgs).zfill(4) + ".png", 
                              self.dataset.num_imgs, 
                              self.camera.info_rgb.width, 
                              self.camera.info_rgb.height,
                              depth)

        # Create object annotations
        for obj in self.objects:
          pos = rotVecByQuat(obj.pos() - self.camera.pos(), 
                             quaternion_inverse(self.camera.rot())) - self.camera.offs_rgb
          coords = projectToImg(pos, self.camera.info_rgb)
          if coords[0] >= 0 and coords[0] < self.camera.info_rgb.width and \
             coords[1] >= 0 and coords[1] < self.camera.info_rgb.height:

            # Obtain bounding box
            x1 = coords[0]
            x2 = coords[0]
            y1 = coords[1]
            y2 = coords[1]
            while x1 > 0 and \
                  self.camera.img_seg[coords[1], x1] == self.camera.img_seg[coords[1], coords[0]]:
              x1 -= 1
            while x2 < self.camera.info_seg.width - 1 and \
                  self.camera.img_seg[coords[1], x2] == self.camera.img_seg[coords[1], coords[0]]:
              x2 += 1
            while y1 > 0 and \
                  self.camera.img_seg[y1, coords[0]] == self.camera.img_seg[coords[1], coords[0]]:
              y1 -= 1
            while y2 < self.camera.info_seg.height - 1 and \
                  self.camera.img_seg[y2, coords[0]] == self.camera.img_seg[coords[1], coords[0]]:
              y2 += 1

            # Obtain direction
            rot = rotVecByQuat(np.array((0,0,1)), obj.rot())
            rot = rotVecByQuat(rot, quaternion_inverse(self.camera.rot()))

            # Obtain object ID
            obj.id = self.camera.img_seg[coords[1], coords[0]]

            # Obtain segmentation
            img_subseg = cv2.inRange(self.camera.img_seg, int(obj.id), int(obj.id))
            img_subseg, contours, hierarchy = cv2.findContours(img_subseg, 
                                                               cv2.RETR_TREE, 
                                                               cv2.CHAIN_APPROX_SIMPLE)
            segmentation = list(itertools.chain.from_iterable([c.flatten() for c in contours]))
            segmentation = [[int(s) for s in segmentation]]

            # Calculate segmented area:
            area = 0
            for c in contours:
              area += cv2.contourArea(c)

            # Add full annotation
            self.dataset.addAnnotation(x1, y1, x2 - x1, y2 - y1,
                                       rot[0], rot[1], rot[2], 
                                       3, self.dataset.num_imgs, 
                                       obj.id, segmentation, area)

        # Reset flags
        self.camera.img_ready_rgb   = False
        self.camera.img_ready_depth = False
        self.camera.img_ready_seg   = False
        self.dataset.num_imgs = self.dataset.num_imgs + 1


      self.rate.sleep()
      raw_input("Enter to record next image...")

if __name__ == '__main__':
  if len(sys.argv) < 1:
    print("No target dir passed")
    sys.exit()
  else:
    target_dir = sys.argv[1]
    print('Target directory: ' + target_dir)
  try:
    rospy.init_node('training_data_generator', anonymous=True)
    TrainingDataGenerator("strawberry", target_dir)

  except rospy.ROSInterruptException:
    pass