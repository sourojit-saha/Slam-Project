# import the necessary packages
from test1 import align_images
import numpy as np
import argparse
import imutils
import cv2
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=True,
# 	help="path to input image that we'll align to template")
# ap.add_argument("-t", "--template", required=True,
# 	help="path to input template image")
args = vars(ap.parse_args())

# load the input image and template from disk
print("[INFO] loading images...")
image = cv2.imread('/home/p43s/test_ws/src/kmz_test/scripts/nb-slam_run_1-110.png')
image = imutils.resize(image, width = 700)
template = cv2.imread('/home/p43s/test_ws/src/kmz_test/scripts/nb-slam_run_2-110.png')
template = imutils.resize(template, width = 700)

# image = cv2.imread('/home/p43s/test_ws/src/kmz_test/scripts/190.png')
# image = imutils.resize(image, width = 700)
# template = cv2.imread('/home/p43s/test_ws/src/kmz_test/scripts/520.png')
# template = imutils.resize(template, width = 700)

image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)


# align the images
print("[INFO] aligning images...")
aligned = align_images(image, template, debug=True)

# resize both the aligned and template images so we can easily
# visualize them on our screen
aligned = imutils.resize(aligned, width=700)
template = imutils.resize(template, width=700)
# our first output visualization of the image alignment will be a
# side-by-side comparison of the output aligned image and the
# template
stacked = np.hstack([aligned, template])

# our second image alignment visualization will be *overlaying* the
# aligned image on the template, that way we can obtain an idea of
# how good our image alignment is
overlay = template.copy()
output = aligned.copy()
cv2.addWeighted(overlay, 0.5, output, 0.5, 0, output)
# show the two output image alignment visualizations
cv2.imshow("image", image)
cv2.imshow("template", template)
cv2.imshow("Image Alignment Stacked", stacked)
cv2.imshow("Image Alignment Overlay", output)
cv2.waitKey(0)