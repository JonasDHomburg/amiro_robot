#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from PIL import Image
import rospy
import os
#import create_image

parent_dir = os.path.dirname(os.path.dirname(__file__))
filepath = os.path.join(parent_dir, 'maps')

image = cv.imread(filepath+"/img_map_route.png")

#Bild sollte 115x115 sein
image_size = tuple(map(int, (rospy.get_param('/navigation_route/image_size')).split(',')))
resized_image = cv.resize(image, image_size, interpolation = cv.INTER_AREA)
resized_image[np.where(resized_image<128)] = 0
resized_image[np.where(resized_image>127)] = 255
cv.imwrite(filepath+"/img_map_route_resized.png", resized_image)

nav_image = Image.open(filepath+"/img_map_route_resized.png")
nav_image.save(filepath+"/map_route.pgm")