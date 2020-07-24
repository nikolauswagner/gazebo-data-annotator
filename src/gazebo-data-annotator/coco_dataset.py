import numpy as np
import rospy
import time
import json
import cv2

class CocoDataset():
  def __init__(self, filename, info="Gazebo generated dataset"):
    self.filename = filename
    self.dataset = {}
    self.dataset["annotations"] = []
    self.dataset["images"]      = []
    self.genInformation("Nikolaus Wagner", info)
    self.genLicenses()
    self.genCategories(["Unripe Strawberry", "Strawberry Flower",
                        "Bad Strawberry", "Ripe Strawberry"])
    self.num_imgs = 0
    rospy.on_shutdown(self.writeJson)

  def writeJson(self):
    print("Caught " + str(self.num_imgs) + " images!")
    file = open(self.filename, "w")
    json.dump(self.dataset, file, indent=2)
    file.close()
    print("Written dataset to" + self.filename)

  def genInformation(self, contributor, description):
    self.dataset["info"] = {}
    self.dataset["info"]["contributor"]  = contributor
    self.dataset["info"]["date_created"] = time.asctime()
    self.dataset["info"]["description"]  = description
    self.dataset["info"]["url"]          = "github.com/lcas"
    self.dataset["info"]["version"]      = "1.0"
    self.dataset["info"]["year"]         = time.localtime()[0]

  def genLicenses(self):
    self.dataset["licenses"] = []
    license = {}
    license["id"]   = 0
    license["name"] = "GNU Lesser General Public License v3 (LGPL-3.0)"
    license["url"]  = "https://opensource.org/licenses/lgpl-3.0.html"
    self.dataset["licenses"].append(license)

  def genCategories(self, cat_names):
    self.dataset["categories"] = []
    for i, name in enumerate(cat_names):
      category = {}
      category["id"]            = i
      category["name"]          = name
      category["supercategory"] = name
      self.dataset["categories"].append(category)

  def addImage(self, filename, img_id, w, h, depth=None):
    img = {}
    img["coco_url"]      = filename
    img["date_captured"] = time.asctime()
    if depth:
      img["depth"]     = depth
    img["file_name"]     = filename
    img["flickr_url"]    = filename
    img["height"]        = h
    img["id"]            = img_id
    img["license"]       = 0
    img["width"]         = w
    self.dataset["images"].append(img)

  def addAnnotation(self, x_pos, y_pos, w, h, x_rot, y_rot, z_rot, cat, img_id, 
                    obj_id, segmentation=None, area=None):
    annotation = {}
    if area:
      annotation["area"]         = area
    else:
      annotation["area"]         = w * h
    annotation["bbox"]           = [x_pos, y_pos, w, h]
    annotation["orientation"]    = [x_rot, y_rot, z_rot]
    annotation["category_id"]    = cat
    annotation["id"]             = int(obj_id)
    annotation["image_id"]       = img_id
    annotation["iscrowd"]        = 0
    if segmentation:
      annotation["segmentation"] = segmentation
    else:
      annotation["segmentation"] = "polygon"
    self.dataset["annotations"].append(annotation)