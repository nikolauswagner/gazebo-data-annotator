import rospy
import numpy as np
import os
from gazebo_msgs.msg import LinkStates
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_inverse
import cv2
from coco_dataset import CocoDataset
from gazebo_tools import GazeboObject, RealsenseCamera


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

  return cam_info.width - x, cam_info.height - y


class TrainingDataGenerator():
  def __init__(self, keyword, target_dir):
    self.keyword    = keyword
    self.target_dir = target_dir
    self.objects    = []
    self.dataset    = CocoDataset(target_dir + "dataset.json")
    self.camera     = RealsenseCamera()
    self.states_sub = rospy.Subscriber("/gazebo/link_states", 
                                       LinkStates, self.statesCB)

    try:
      os.makedirs(target_dir + "/rgb")
      os.makedirs(target_dir + "/depth")
    except:
      pass

    self.rate = rospy.Rate(100)
    self.run()

  def statesCB(self, links):
    # Get objects of interest
    self.objects = []
    for idx, name in enumerate(links.name):
      if self.keyword in name:
        obj = GazeboObject()
        obj.id = idx
        obj.pose = links.pose[idx]
        self.objects.append(obj)

    # Get camera pose
    self.camera.pose = links.pose[links.name.index("realsense_camera::link")]

  def run(self):
    while not rospy.is_shutdown():
      if self.camera.img_ready_rgb and self.camera.img_ready_depth:
        # Store image
        filename_rgb = (self.target_dir + "/rgb/" + 
                        str(self.camera.img_id_rgb).zfill(4) + ".png")
        cv2.imwrite(filename_rgb, cv2.cvtColor(self.camera.img_rgb, cv2.COLOR_BGR2RGB))
        filename_depth = (self.target_dir + "/depth/" + 
                          str(self.camera.img_id_depth).zfill(4) + ".png")
        cv2.imwrite(filename_depth, self.camera.img_depth.astype(np.uint16))

        # Add image to dataset
        self.dataset.addImage("rgb/" + str(self.camera.img_id_rgb).zfill(4) + ".png", 
                              self.camera.img_id_rgb, 
                              self.camera.info_rgb.width, 
                              self.camera.info_rgb.height)

        # Create object annotations
        for obj in self.objects:
          pos = rotVecByQuat(obj.pos() - self.camera.pos(), 
                             quaternion_inverse(self.camera.rot())) - self.camera.offs_rgb
          coords = projectToImg(pos, self.camera.info_rgb)
          if coords[0] >= 0 and coords[0] < self.camera.info_rgb.width and \
             coords[1] >= 0 and coords[1] < self.camera.info_rgb.height:
            # Obtain bounding box
            box = projectToImg((pos[0], 0.05, 0.05), self.camera.info_rgb)
            w = int(self.camera.info_rgb.width/2  - box[0])
            h = int(self.camera.info_rgb.height/2 - box[1])
            x_pos = int(coords[0] - (w / 2))
            y_pos = int(coords[1] - (h / 2))
            # Obtain direction
            rot = rotVecByQuat(np.array((0,0,1)), obj.rot())
            rot = rotVecByQuat(rot, quaternion_inverse(self.camera.rot()))
            self.dataset.addAnnotation(x_pos, y_pos, w, h,
                                       rot[0], rot[1], rot[2], 
                                       3, self.camera.img_id_rgb, obj.id)

        # Reset flags
        self.camera.img_ready_rgb   = False
        self.camera.img_ready_depth = False


      self.rate.sleep()

if __name__ == '__main__':
  try:
    rospy.init_node('training_data_generator', anonymous=True)
    target_dir = "./annotated_data/strawberries_01/"
    TrainingDataGenerator("strawberry", target_dir)

  except rospy.ROSInterruptException:
    pass