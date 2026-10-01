'''
ArUco Fiducial Marker Detection & Sandbox Perspective Transform.

Handles detection of ArUco markers for sandbox corners, robots, pallets,
and goals. Provides perspective-warped (top-down) view of the sandbox.

Compatible with OpenCV 5.x ArUco API (uses ArucoDetector + solvePnP
instead of deprecated Dictionary_get + estimatePoseSingleMarkers).
'''

import math
import cv2 as cv
import numpy as np
import time
import constants
from constants import debugPrint


# =====================================================================
#  ArUco detector (shared across all calls, created once at import time)
# =====================================================================
_aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
_aruco_params = cv.aruco.DetectorParameters()
_aruco_detector = cv.aruco.ArucoDetector(_aruco_dict, _aruco_params)


def _estimate_pose_single_marker(marker_corner, marker_size, camera_matrix, dist_coeffs):
    '''
    Replacement for the removed cv.aruco.estimatePoseSingleMarkers().
    Uses cv.solvePnP with 4 coplanar corner points.

    Returns: (rvec, tvec) — both as flat 1D arrays.
    '''
    # 3D coordinates of marker corners in marker frame (Z=0 plane)
    half = marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    img_points = marker_corner.reshape((4, 2)).astype(np.float64)

    success, rvec, tvec = cv.solvePnP(
        obj_points, img_points, camera_matrix, dist_coeffs
    )
    if not success:
        return None, None

    return rvec.flatten(), tvec.flatten()


class CV_Fiducial:
    def __init__(self):
        self.cv_fiducial_markerDict = {}
        self.cv_fiducial_cornerMarkerDict = {}
        self.cv_fiducial_warpedCornerMarkerDict = {}
        self.mm_per_pixel = None
        self.sandbox_height_mm = None
        self.sandbox_width_mm = None

    def cv_fiducial_setupSandbox(self, image_frame):
        # Find the sandbox corner fiducials's pose
        while(self._cv_fiducial_detectSandboxCorners(image_frame) == False):
            print("No sandbox corners detected.")
            time.sleep(1)

        sandboxImage = self.cv_fiducial_flattenSandboxImage(image_frame)

        if constants.CV_DEBUG:
            cv.imshow("Sandbox Init Image", sandboxImage)
            cv.waitKey(0)

        return sandboxImage



    def _cv_fiducial_detectSandboxCorners(self, image_frame):
        if constants.CV_DEBUG:
            image_frame_annotated = image_frame.copy()

        corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(image_frame)

        if fiducial_ids is not None and len(corner_list) > 0:
            fiducial_ids = fiducial_ids.flatten()
            for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = int((topLeft[1] + bottomRight[1]) / 2.0)

                rvec = None
                tvec = None

                # reserve the extra processing for the corner fiducials
                if fiducial_id in constants.CORNER_FIDUCIALS:
                    # estimate the pose of the marker
                    rvec, tvec = _estimate_pose_single_marker(
                        marker_corner,
                        constants.FIDUCIAL_WIDTH_MM,
                        constants.CAMERA_MATRIX,
                        constants.DISTORTION_COEFFICIENTS)
                
                self.cv_fiducial_cornerMarkerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, rvec, tvec)

        else:
            return False

        if constants.CV_DEBUG:
            for fiducial_id in self.cv_fiducial_cornerMarkerDict.keys():
                cv.circle(image_frame_annotated, self.cv_fiducial_cornerMarkerDict[fiducial_id][0:2], 4, (0, 0, 255), -1)
            cv.imshow("Corner Pose", image_frame_annotated)
            cv.waitKey(0)

        return True
        

    ''' 
    This function returns the fiducial locations in the image.
    '''
    def cv_fiducial_generatePalletLocations(self, sandbox_image):

        corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(sandbox_image)

        # debugPrint("Fiducial IDs detected in field: " + str(fiducial_ids))

        if fiducial_ids is not None and len(corner_list) > 0:
            fiducial_ids = fiducial_ids.flatten()
            for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Use the fiducial corners to determine the orientation
                orientation = math.atan2(topLeft[1] - bottomLeft[1], topLeft[0] - bottomLeft[0])
                orientation += math.pi/2
                orientation = math.atan2(math.sin(orientation), math.cos(orientation))
                orientation = orientation * -1 # negate the angle to match the robot's coordinate system

                # shift the pallets back by a constant amount to account for the robot's offset from the center of the pallet
                if fiducial_id in constants.PALLET_FIDUCIALS:
                    centerX = centerX - (constants.CV_PALLET_CENTER_OFFSET * math.cos(orientation))
                    centerY = centerY + (constants.CV_PALLET_CENTER_OFFSET * math.sin(orientation))
                    
                if fiducial_id in constants.GOAL_FIDUCIALS:
                    centerX = centerX - (constants.CV_GOAL_CENTER_OFFSET * math.cos(orientation))
                    centerY = centerY + (constants.CV_GOAL_CENTER_OFFSET * math.sin(orientation))
                
                self.cv_fiducial_markerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, orientation)

    '''
    This function finds the correct fiducial locations and returns them in the correct order.

    2 of the corners maximize and minimize their x,y values respectively.
    The other 2 corners only have 1 value that is either the max or min.
    This simple logic is used to sort the fiducials into the correct order.
    '''
    def _cv_fiducial_findCornerFiducialOrdering(self):
        cornerFiducialIDs = constants.CORNER_FIDUCIALS
        unsortedCornerFiducialCenters = []
        unsortedCornerFiducialCenterIDs = []
        
        # Get information about the corner fiducials.
        for fiducialID in cornerFiducialIDs:
            if fiducialID in self.cv_fiducial_cornerMarkerDict.keys():
                unsortedCornerFiducialCenters.append(self.cv_fiducial_cornerMarkerDict[fiducialID][0:2])
                unsortedCornerFiducialCenterIDs.append(fiducialID)
            else:
                print(self.cv_fiducial_cornerMarkerDict)
                constants.blockingError("Error, Sandbox corner fiducial not found." + str(fiducialID))
        
        # Get the min and max fiducials since those are easy
        top_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
        top_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])
        bottom_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
        bottom_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])

        # get the appropriate fiducial ID for each corner
        top_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_left)]
        top_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_right)]
        bottom_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_right)]
        bottom_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_left)]

        return top_left_id, top_right_id, bottom_left_id, bottom_right_id

    def cv_fiducial_flattenSandboxImage(self, image_frame, height = constants.CV_SANDBOX_WIDTH, width = constants.CV_SANDBOX_HEIGHT):
        
        # override after initial sandbox size is found
        if (self.sandbox_width_mm != None and self.sandbox_height_mm != None):
            height = int(self.sandbox_height_mm)
            width = int(self.sandbox_width_mm)

        # add a bit of a buffer to the sandbox size
        buffer_pixels_height = int(height * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)
        buffer_pixels_width = int(width * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)

        destination_corners = np.array([
            [buffer_pixels_height, buffer_pixels_width],
            [height - 1 + buffer_pixels_height, buffer_pixels_width],
            [height - 1 + buffer_pixels_height, width - 1 + buffer_pixels_width],
            [buffer_pixels_height, width - 1 + buffer_pixels_width]], dtype = "float32")
        
        # get fiducials in the right order
        top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducialOrdering()

        fiducial_corners = np.array([
            self.cv_fiducial_cornerMarkerDict[top_left_id][0:2],
            self.cv_fiducial_cornerMarkerDict[top_right_id][0:2],
            self.cv_fiducial_cornerMarkerDict[bottom_right_id][0:2],
            self.cv_fiducial_cornerMarkerDict[bottom_left_id][0:2]], dtype = "float32")

        M = cv.getPerspectiveTransform(fiducial_corners, destination_corners)
        sandbox_image = cv.warpPerspective(image_frame, M, (height + (buffer_pixels_height * 2), width + (buffer_pixels_width * 2)))

        return sandbox_image

    def cv_fiducial_getCornerPositions(self):
        '''Returns list of [x, y, orientation] for corner markers detected in the warped sandbox image.'''
        cornerFiducialIDs = constants.CORNER_FIDUCIALS
        foundCornerPositions = []
        for fiducialID in cornerFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                foundCornerPositions.append(pose)
        return foundCornerPositions

    def cv_fiducial_getGoalPositions(self):
        goalFiducialIDs = constants.GOAL_FIDUCIALS
        foundGoalFiducialIDs = []

        # Find and pack the found goals from the possible goal fiducials
        for fiducialID in goalFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                foundGoalFiducialIDs.append(pose)
        
        return foundGoalFiducialIDs


    def cv_fiducial_getPalletPositions(self):
        palletFiducialIDs = constants.PALLET_FIDUCIALS
        foundPalletFiducialIDs = []

        # Find and pack the found pallets from the possible pallet fiducials
        for fiducialID in palletFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                foundPalletFiducialIDs.append(pose)
        
        return foundPalletFiducialIDs

    def cv_fiducial_getRobotPositions(self):
        robotFiducialIDs = constants.ROBOT_FIDUCIALS
        foundRobotFiducialPoses = []
        foundRobotFiducialIds = []

        for fiducialID in robotFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                foundRobotFiducialPoses.append(pose)
                foundRobotFiducialIds.append(fiducialID)

        return foundRobotFiducialPoses, foundRobotFiducialIds