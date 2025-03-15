import cv2
import numpy as np
import matplotlib.pyplot as plt 
import os
#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template, line):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	if line:
		mask = np.zeros(img.shape[:2], np.uint8)
		mask[170:240, :] = 255
		img = cv2.bitwise_and(img, img, mask=mask)
    
	bounding_box = ((0,0),(0,0))
	hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	if template is not None:
		print(template.shape)
		template_hsv = cv2.cvtColor(template, cv2.COLOR_BGR2HSV)
		hue_values = template_hsv[:, :, 0].flatten()  # get hue values
		lower_hue = max(0, np.percentile(hue_values, 46))   # (avoid outliers)
		upper_hue = min(179, np.percentile(hue_values, 100))  #  (avoid outliers)
		print(lower_hue, upper_hue)
	else:
		lower_hue = 8  # Lower HSV threshold
		upper_hue = 27 # Upper HSV threshold
	lower_bound = np.array([lower_hue, 200, 190],  dtype=np.uint8)  # Lower HSV threshold 
	upper_bound = np.array([upper_hue, 255, 255],  dtype=np.uint8)  # Upper HSV threshold
	
	
	mask = cv2.inRange(hsv_image, lower_bound, upper_bound) 
	

	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	for contour in contours:
		if cv2.contourArea(contour) > 100:  # Minimum contour area to avoid noise
			x, y, w, h = cv2.boundingRect(contour)
			cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding box
			bounding_box = ( (x, y), (x + w, y + h))
	#cv2.imshow("Detected Cone", img)
	#image_print(img)
	return (bounding_box)



	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	# return bounding_box

if __name__ == "__main__":
	cone_template = "test_images_cone/cone_template.png"
	template = cv2.imread(cone_template)
	print(template.shape)
	for i in range(1,20):
		image_path = "test_images_cone/test" +str(i) + ".jpg"

		image = cv2.imread(image_path)
		cd_color_segmentation(np.array(image), template)
