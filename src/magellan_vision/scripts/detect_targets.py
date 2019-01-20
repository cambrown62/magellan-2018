import cv2 as cv
import numpy as np

# Temporary for testing - hues from the images in rules document
hueArch = 39
huePole = 255
hueRamp = 156

# These should probably be ROSparams
hueMinArch = hueArch - 20
hueMaxArch = hueArch + 20
hueMinRamp = hueRamp - 20
hueMaxRamp = hueRamp + 20
hueMinPole = huePole - 20
hueMaxPole = huePole + 20
cannyThresh = 100
# I think these should be the same for all targets
# Sat should be wide for various outdoors conditions
satMin = 0
satMax = 255
# Low val = less color - raise min to reduce risk of picking up non-target objects
valMin = 0
valMax = 255

# Get the current frame from ROS
#frame = None
frame = cv.imread('Hoop.png', cv.IMREAD_COLOR)

frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
archThreshold = cv.inRange(frameHSV, (hueMinArch, satMin, valMin), (hueMaxArch, satMax, valMax))
cv.imshow('image', archThreshold)
key = cv.waitKey(0)


# Arch detection should look for two vertical lines and find the midpoint

# rampThreshold = cv.inRange(frameHSV, (hueMinRamp, satMin, valMin), (hueMaxRamp, satMax, valMax))
# rampBlur = cv.blur(rampThreshold, (3, 3))
# rampEdges = cv.canny(rampBlur, cannyThresh, cannyThresh*2)
# 
# # Ramp detection should find the center of the blob
# 
# poleThreshold = cv.inRange(frameHSV, (hueMinPole, satMin, valMin), (hueMaxPole, satMax, valMax))

# Pole detection should find the center of the blob
