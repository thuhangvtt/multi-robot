
import time
import os
import numpy as np


'''
Computer Vision Constants
'''
CV_DEBUG = False
CV_USE_CAMERA = True



CV_LOCALIZE_ROBOTS_FIDUCIALS = True

# The fiducial marker indexes for the sandbox perimeter and goals. 
# Ordering is not important.
CORNER_FIDUCIALS = [1,4,7,10]
PALLET_FIDUCIALS = [2, 6, 8, 12]
ROBOT_FIDUCIALS = [0, 3, 5]
GOAL_FIDUCIALS = [9,11,15,13,14]

# Which physical robot corresponds to which ficucial
# Robot 3 uses fiducial 0, Robot 1 uses fiducial 3
# Uses same index as ROBOT_FIDUCIALS
ROBOT_HARDWARE_NUMBERS = [3, 1, 2]

WEBCAM_ID = 1 

CV_VIZ_SCALE = 0.35

BACKUP_TIME = 15 # seconds

path = os.getcwd()
CV_DEBUG_IMAGE_PATH = path + "/computerVisionFragments/field1080p.jpg"

# lookup table for visualizer colors
CV_COLOR_LOOKUP = [(165, 165, 110),
                    (82, 107, 253),
                    (103, 191, 150)]

# The physical real world width of the fiducial markers in mm.
FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

CV_SANDBOX_IMAGE_BUFFER_PERCENT = 0.1 # How much extra to scale the image by when cropping the sandbox
CV_SANDBOX_HEIGHT = 650 # mm
CV_SANDBOX_WIDTH = 1000 # mm

# Keep the last object pose briefly when ArUco misses a frame, preventing
# one-frame camera/detection glitches from making the visualizer blink.
CV_MARKER_HOLD_FRAMES = 8

CAMERA_MATRIX = np.array([ \
    [1.61385605e+03, 0.00000000e+00, 6.35322605e+02], \
    [0.00000000e+00, 1.61656336e+03, 3.68157714e+02], \
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00], \
])

DISTORTION_COEFFICIENTS = np.array([ \
    [2.15255284e-01, -1.01672832e+00, -1.02937616e-03, 2.35515928e-04, 5.65913680e-01] \
])

# Enable only after recalibrating the same Camo resolution/lens used at runtime.
CV_USE_CAMERA_CALIBRATION = True

# Body-coordinate offsets from each marker center to the object's physical center (mm).
# Tuples are (forward, right) in the marker/object frame and rotate with theta.
ROBOT_CENTER_OFFSETS_MM = {0: (0.0, 0.0), 3: (0.0, 0.0), 5: (0.0, 0.0)}
PALLET_CENTER_OFFSETS_MM = {2: (0.0, 0.0), 6: (0.0, 0.0), 8: (0.0, 0.0), 12: (0.0, 0.0)}

CV_PALLET_CENTER_OFFSET = 60 # mm or pixels
CV_GOAL_CENTER_OFFSET = 50

CV_PALLET_OFFSET = 30 # offset to give the planner for pickup controller



def debugPrint(debugMsg):
    if CV_DEBUG:
        print(debugMsg)

def blockingError(errorMsg):
    while(True):
        print(errorMsg)
        time.sleep(1)


'''
Controls Constants
'''
wheel_base = 100 # mm # wheelbase is actually 100mm
half_wheel_base = 50 # mm # wheelbase is actually 100mm
wheel_radius = 30 # mm
tangent_curviness = 1.25 # multiplier for curviness of the spline interpolation
maxRobotSpeed = 100 #mm/s
CONTROLS_DEBUG = False
CONTROLS_STATE_DRIVING_TO_GOAL = 0
CONTROLS_STATE_DRIVING_TO_PALLET = 1
CONTROLS_MAX_PWM_OFFSET = 40
CONTROLS_MAX_PWM = 90 + CONTROLS_MAX_PWM_OFFSET
CONTROLS_MIN_PWM = 90 - CONTROLS_MAX_PWM_OFFSET


CONTROLS_ROBOT_PID_KPx = 1
CONTROLS_ROBOT_PID_KIx = 0.0
CONTROLS_ROBOT_PID_KDx = 0.8
CONTROLS_ROBOT_PID_KPy = 0.04
CONTROLS_ROBOT_PID_KIy = 0.0
CONTROLS_ROBOT_PID_KDy = 0.01
CONTROLS_ROBOT_PID_KPtheta = 0.5
CONTROLS_ROBOT_PID_KItheta = 0.0
CONTROLS_ROBOT_PID_KDtheta = 0.2

CONTROLS_PICKUP_ROBOT_PID_KPx = 0.8
CONTROLS_PICKUP_ROBOT_PID_KIx = 0.0
CONTROLS_PICKUP_ROBOT_PID_KDx = 0.6
CONTROLS_PICKUP_ROBOT_PID_KPy = 0.04
CONTROLS_PICKUP_ROBOT_PID_KIy = 0.0
CONTROLS_PICKUP_ROBOT_PID_KDy = 0.018
CONTROLS_PICKUP_ROBOT_PID_KPtheta = 0.5
CONTROLS_PICKUP_ROBOT_PID_KItheta = 0.0
CONTROLS_PICKUP_ROBOT_PID_KDtheta = 0.22

CONTROLS_ELECTROMAGNET_TIME_THRESHOLD = 1.5

CONTROLS_STATE_DRIVING_TO_PALLET = 0
CONTROLS_STATE_DRIVING_TO_GOAL = 1

ELECTROMAGNET_DONT_SEND = 0
ELECTROMAGNET_ENABLE = 1
ELECTROMAGNET_DISABLE = 2
