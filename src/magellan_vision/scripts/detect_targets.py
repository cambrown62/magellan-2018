import cv2 as cv

# These should probably be ROSparams
hueMinArch = 0
hueMaxArch = 100
hueMinRamp = 0
hueMaxRamp = 100
hueMinPole = 0
hueMaxPole = 100 
cannyThresh = 100
# I think these should be the same for all targets
satMin = 0
satMax = 0
valMin = 0
valMax = 0

# Get the current frame from ROS
frame = None

frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
archThreshold = cv.inRange(frameHSV, (hueMinArch, satMin, valMin), (hueMaxArch, satMax, valMax))

# Arch detection should look for two vertical lines and find the midpoint

rampThreshold = cv.inRange(frameHSV, (hueMinRamp, satMin, valMin), (hueMaxRamp, satMax, valMax))
rampBlur = cv.blur(rampThreshold, (3, 3))
rampEdges = cv.canny(rampBlur, cannyThresh, cannyThresh*2)

# Ramp detection should find the center of the blob

poleThreshold = cv.inRange(frameHSV, (hueMinPole, satMin, valMin), (hueMaxPole, satMax, valMax))

# Pole detection should find the center of the blob
