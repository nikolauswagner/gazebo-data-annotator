import json
import cv2
import os
import sys

def getAnnotation(json_data, img_id):
	bboxes = []
	for i in range(len(json_data["annotations"])):
		if json_data["annotations"][i]["image_id"] == img_id:
			(x1, y1, w, h) = json_data["annotations"][i]["bbox"]
			x2 = x1 + w
			y2 = y1 + h
			obj_id = json_data["annotations"][i]["id"]
			rot = json_data["annotations"][i]["orientation"]
			bboxes.append([obj_id, x1, y1, x2, y2, rot])

	return bboxes


if __name__ == '__main__':

	if len(sys.argv) < 1:
		print("No target dir passed")
		sys.exit()
	else:
		data_dir = sys.argv[1]
		print('Target directory: ' + data_dir)
	
	fp = open(data_dir + "dataset.json")
	json_data = json.load(fp)

	for i_img in range(len(json_data["images"])):
		rgb_filename   = json_data["images"][i_img]["file_name"]
		depth_filename = "depth" + rgb_filename[3:]
		seg_filename   = "segmentation" + rgb_filename[3:]
		img_id         = json_data["images"][i_img]["id"]

		img_rgb   = cv2.imread(data_dir + rgb_filename)
		#img_depth = cv2.imread(data_dir + depth_filename, cv2.IMREAD_ANYDEPTH)
		#img_depth = cv2.normalize(img_depth, None, alpha=0, beta=65536, norm_type=cv2.NORM_MINMAX)
		img_seg = cv2.imread(data_dir + seg_filename, cv2.IMREAD_ANYDEPTH)
		img_seg = cv2.normalize(img_seg, None, alpha=0, beta=65536, norm_type=cv2.NORM_MINMAX)

		annotations = getAnnotation(json_data, img_id)
		print(img_id)
		for (obj_id, x1, y1, x2, y2, rot) in annotations:
			cv2.rectangle(img_rgb, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 1)
			#cv2.rectangle(img_depth, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,65000), 1)
			cv2.putText(img_rgb, "obj:" + str(obj_id), (x1, y1), 
			            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200,200,0), 1)
			cv2.putText(img_rgb, "x:{:.1f}/y:{:.1f}/z:{:.1f}".format(rot[0], rot[1], rot[2]), 
                  (x1-30, y2+5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (50,50,0), 1)

		cv2.imshow("img_rgb", img_rgb)
		#cv2.imshow("img_seg", img_seg)
		#cv2.imshow("img_depth", img_depth)
		cv2.waitKey()