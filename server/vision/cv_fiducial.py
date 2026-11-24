import math
import traceback
import cv2 as cv
import numpy as np
import time
import constants
from constants import debugPrint

_aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
_aruco_params = cv.aruco.DetectorParameters()
_aruco_params.adaptiveThreshWinSizeMin = 3
_aruco_params.adaptiveThreshWinSizeMax = 23
_aruco_params.adaptiveThreshWinSizeStep = 10
_aruco_params.minMarkerPerimeterRate = 0.01
_aruco_params.maxMarkerPerimeterRate = 4.0
_aruco_params.polygonalApproxAccuracyRate = 0.08
_aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
_aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
_aruco_detector = cv.aruco.ArucoDetector(_aruco_dict, _aruco_params)

def _estimate_pose_single_marker(marker_corner, marker_size, camera_matrix, dist_coeffs):

    try:
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
    except Exception as e:
        print(f"[CV_Fiducial] LỖI: _estimate_pose_single_marker: {e}")
        print(traceback.format_exc())
        return None, None

def _marker_center_from_corners(corners):

    return (corners[0] + corners[2]) / 2.0

def _projected_square_center(corners):

    try:
        first_start, first_end = corners[0], corners[2]
        second_start, second_end = corners[1], corners[3]
        first_direction = first_end - first_start
        second_direction = second_end - second_start
        denominator = (
            first_direction[0] * second_direction[1]
            - first_direction[1] * second_direction[0]
        )
        if abs(float(denominator)) < 1e-8:
            return _marker_center_from_corners(corners)

        delta = second_start - first_start
        parameter = (
            delta[0] * second_direction[1]
            - delta[1] * second_direction[0]
        ) / denominator
        return first_start + parameter * first_direction
    except Exception as e:
        print(f"[CV_Fiducial] LỖI: _projected_square_center: {e}")
        print(traceback.format_exc())
        return _marker_center_from_corners(corners)

class CV_Fiducial:

    def __init__(self):
        self.cv_fiducial_markerDict = {}
        self.cv_fiducial_cornerMarkerDict = {}
        self.cv_fiducial_warpedCornerMarkerDict = {}
        self.mm_per_pixel = None
        self.sandbox_height_mm = constants.CV_SANDBOX_HEIGHT
        self.sandbox_width_mm = constants.CV_SANDBOX_WIDTH
        self._last_robot_ids_logged = None
        self._marker_missed_frames = {}
        self._marker_seen_at = {}
        self._observed_ids = set()
        self._homography_M = None
        self._sandbox_pixel_size = None
        self._viz_homography_M = None
        self._viz_scale = None
        self._viz_size = None

    def _apply_center_offset(self, fiducial_id, center_x, center_y, orientation):

        try:
            if fiducial_id in constants.ROBOT_CENTER_OFFSETS_MM:
                forward, right = constants.ROBOT_CENTER_OFFSETS_MM[fiducial_id]
            elif fiducial_id in constants.PALLET_CENTER_OFFSETS_MM:
                forward, right = constants.PALLET_CENTER_OFFSETS_MM[fiducial_id]
            else:
                return center_x, center_y

            offset_x = forward * math.cos(orientation) + right * math.sin(orientation)
            offset_y = -forward * math.sin(orientation) + right * math.cos(orientation)
            return center_x + offset_x, center_y + offset_y
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: _apply_center_offset fid={fiducial_id}: {e}")
            print(traceback.format_exc())
            return center_x, center_y

    @staticmethod
    def _warp_pixel_to_field_mm(pose):

        try:
            buffer_x = constants.CV_SANDBOX_WIDTH * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
            buffer_y = constants.CV_SANDBOX_HEIGHT * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
            return [pose[0] - buffer_x, pose[1] - buffer_y, pose[2]]
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: _warp_pixel_to_field_mm: {e}")
            print(traceback.format_exc())
            return list(pose)

    def cv_fiducial_setupSandbox(self, image_frame, frame_supplier=None):

        try:
            last_missing = None
            while True:
                self.cv_fiducial_cornerMarkerDict.clear()
                corners_ready = self._cv_fiducial_detectSandboxCorners(image_frame)
                if corners_ready:
                    break

                detected = sorted(self.cv_fiducial_cornerMarkerDict.keys())
                missing = [
                    fiducial_id
                    for fiducial_id in constants.CORNER_FIDUCIALS
                    if fiducial_id not in self.cv_fiducial_cornerMarkerDict
                ]
                if missing != last_missing:
                    print(f"[CV] Detected corner IDs: {detected or 'none'}")
                    print(f"[CV] Missing corner IDs: {missing}")
                    last_missing = missing
                print("  Đang chờ nhận đủ 4 góc sa bàn [1, 4, 7, 10]...", end='\r')
                time.sleep(0.2)
                if frame_supplier is not None:
                    image_frame = frame_supplier()

            sandboxImage = self.cv_fiducial_flattenSandboxImage(image_frame)

            if constants.CV_DEBUG:
                cv.imshow("Sandbox Init Image", sandboxImage)
                cv.waitKey(0)

            return sandboxImage
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_setupSandbox: {e}")
            print(traceback.format_exc())
            raise

    def _cv_fiducial_detectSandboxCorners(self, image_frame):

        try:
            if constants.CV_DEBUG:
                image_frame_annotated = image_frame.copy()

            corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(image_frame)

            if fiducial_ids is not None and len(corner_list) > 0:
                fiducial_ids = fiducial_ids.flatten()
                for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                    corners = marker_corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    marker_center = _projected_square_center(corners)
                    centerX = int(round(float(marker_center[0])))
                    centerY = int(round(float(marker_center[1])))

                    rvec = None
                    tvec = None

                    if fiducial_id in constants.CORNER_FIDUCIALS:
                        rvec, tvec = _estimate_pose_single_marker(
                            marker_corner,
                            constants.FIDUCIAL_WIDTH_MM,
                            constants.CAMERA_MATRIX,
                            (np.zeros_like(constants.DISTORTION_COEFFICIENTS)
                             if constants.CV_USE_CAMERA_CALIBRATION else constants.DISTORTION_COEFFICIENTS))

                    self.cv_fiducial_cornerMarkerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, rvec, tvec)

            else:
                return False

            if constants.CV_DEBUG:
                for fiducial_id in self.cv_fiducial_cornerMarkerDict.keys():
                    cv.circle(image_frame_annotated, self.cv_fiducial_cornerMarkerDict[fiducial_id][0:2], 4, (0, 0, 255), -1)
                cv.imshow("Corner Pose", image_frame_annotated)
                cv.waitKey(0)

            return all(
                fiducial_id in self.cv_fiducial_cornerMarkerDict
                for fiducial_id in constants.CORNER_FIDUCIALS
            )
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: _cv_fiducial_detectSandboxCorners: {e}")
            print(traceback.format_exc())
            return False

    def _update_markers_from_sandbox_corners(self, sandbox_corner_list, fiducial_ids, captured_at=None):
        """Update marker dictionary and missed-frame counters from sandbox-space corners."""
        detected_ids = [] if fiducial_ids is None else [int(fid) for fid in np.asarray(fiducial_ids).flatten()]
        detected_id_set = set(detected_ids)
        self._observed_ids = detected_id_set
        captured_at = time.monotonic() if captured_at is None else captured_at

        for fid in detected_id_set:
            self._marker_seen_at[fid] = captured_at
            self._marker_missed_frames[fid] = 0
        for fid in list(self.cv_fiducial_markerDict.keys()):
            fid_int = int(fid)
            if fid_int not in detected_id_set:
                self._marker_missed_frames[fid_int] = self._marker_missed_frames.get(fid_int, 0) + 1

        detected_robot_ids = sorted(
            fid for fid in detected_ids if fid in constants.ROBOT_FIDUCIALS
        )
        if detected_robot_ids != self._last_robot_ids_logged:
            missing_robot_ids = [
                fid for fid in constants.ROBOT_FIDUCIALS if fid not in detected_robot_ids
            ]
            print(f"[CV] Detected robot IDs: {detected_robot_ids or 'none'}")
            print(f"[CV] Missing robot IDs: {missing_robot_ids}")
            self._last_robot_ids_logged = detected_robot_ids

        if not detected_ids or not sandbox_corner_list:
            return

        for (corners_raw, fiducial_id) in zip(sandbox_corner_list, detected_ids):
            corners = np.asarray(corners_raw, dtype=np.float64).reshape((4, 2))
            (topLeft_f, topRight_f, bottomRight_f, bottomLeft_f) = corners
            topRight = (int(round(topRight_f[0])), int(round(topRight_f[1])))
            bottomRight = (int(round(bottomRight_f[0])), int(round(bottomRight_f[1])))
            bottomLeft = (int(round(bottomLeft_f[0])), int(round(bottomLeft_f[1])))
            topLeft = (int(round(topLeft_f[0])), int(round(topLeft_f[1])))

            marker_center = _marker_center_from_corners(corners)
            centerX = float(marker_center[0])
            centerY = float(marker_center[1])

            orientation = math.atan2(
                float(topLeft_f[1] - bottomLeft_f[1]),
                float(topLeft_f[0] - bottomLeft_f[0]),
            )
            orientation += math.pi / 2
            orientation = math.atan2(math.sin(orientation), math.cos(orientation))
            orientation = orientation * -1

            if fiducial_id in constants.PALLET_FIDUCIALS:
                centerX = centerX - (constants.CV_PALLET_CENTER_OFFSET * math.cos(orientation))
                centerY = centerY + (constants.CV_PALLET_CENTER_OFFSET * math.sin(orientation))

            if fiducial_id in constants.GOAL_FIDUCIALS:
                centerX = centerX - (constants.CV_GOAL_CENTER_OFFSET * math.cos(orientation))
                centerY = centerY + (constants.CV_GOAL_CENTER_OFFSET * math.sin(orientation))

            centerX, centerY = self._apply_center_offset(
                fiducial_id, centerX, centerY, orientation
            )

            self.cv_fiducial_markerDict[fiducial_id] = (
                centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, orientation
            )

    def cv_fiducial_generatePalletLocations(self, sandbox_image, captured_at=None):

        try:
            corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(sandbox_image)
            sandbox_corners = (
                [np.asarray(c).reshape((4, 2)) for c in corner_list]
                if corner_list is not None else []
            )
            self._update_markers_from_sandbox_corners(sandbox_corners, fiducial_ids, captured_at)
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_generatePalletLocations: {e}")
            print(traceback.format_exc())
            self._observed_ids.clear()
            raise

    def cv_fiducial_localizeFromRaw(self, raw_undistorted_frame, captured_at=None):
        """Detect markers directly in camera resolution and project corners with cached homography."""
        if self._homography_M is None:
            raise RuntimeError("Sandbox homography has not been initialized")
        try:
            detect_img = (
                cv.cvtColor(raw_undistorted_frame, cv.COLOR_BGR2GRAY)
                if raw_undistorted_frame.ndim == 3 else raw_undistorted_frame
            )
            corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(detect_img)
            sandbox_corners = []
            if corner_list is not None and len(corner_list) > 0:
                for marker_corner in corner_list:
                    pts = marker_corner.reshape((-1, 1, 2)).astype(np.float32)
                    warped = cv.perspectiveTransform(pts, self._homography_M).reshape((4, 2))
                    sandbox_corners.append(warped)
            self._update_markers_from_sandbox_corners(sandbox_corners, fiducial_ids, captured_at)
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_localizeFromRaw: {e}")
            print(traceback.format_exc())
            self._observed_ids.clear()
            raise

    def marker_missed_frames(self, fid):
        fid_int = int(fid)
        if fid_int not in self.cv_fiducial_markerDict:
            return 999999
        return int(self._marker_missed_frames.get(fid_int, 999999))

    def marker_age(self, fid, now=None):
        seen_at = self._marker_seen_at.get(int(fid))
        if seen_at is None:
            return float('inf')
        now = time.monotonic() if now is None else now
        return max(0.0, float(now) - float(seen_at))

    def is_marker_fresh(self, fid, max_missed_frames=2, max_age_s=None, now=None):
        fid_int = int(fid)
        if fid_int not in self.cv_fiducial_markerDict:
            return False
        if self.marker_missed_frames(fid_int) >= max_missed_frames:
            return False
        if max_age_s is not None and self.marker_age(fid_int, now=now) > max_age_s:
            return False
        return True

    def fresh_marker_positions_mm(self):

        return {
            int(fid): self._warp_pixel_to_field_mm([value[0], value[1], value[6]])
            for fid, value in self.cv_fiducial_markerDict.items()
        }

    def _cv_fiducial_findCornerFiducialOrdering(self):

        try:
            cornerFiducialIDs = constants.CORNER_FIDUCIALS
            unsortedCornerFiducialCenters = []
            unsortedCornerFiducialCenterIDs = []

            for fiducialID in cornerFiducialIDs:
                if fiducialID in self.cv_fiducial_cornerMarkerDict.keys():
                    unsortedCornerFiducialCenters.append(self.cv_fiducial_cornerMarkerDict[fiducialID][0:2])
                    unsortedCornerFiducialCenterIDs.append(fiducialID)
                else:
                    print(self.cv_fiducial_cornerMarkerDict)
                    constants.blockingError("Error, Sandbox corner fiducial not found." + str(fiducialID))

            top_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
            top_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])
            bottom_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
            bottom_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])

            top_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_left)]
            top_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_right)]
            bottom_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_right)]
            bottom_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_left)]

            return top_left_id, top_right_id, bottom_left_id, bottom_right_id
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: _cv_fiducial_findCornerFiducialOrdering: {e}")
            print(traceback.format_exc())
            raise

    def cv_fiducial_flattenSandboxImage(self, image_frame, width=constants.CV_SANDBOX_WIDTH, height=constants.CV_SANDBOX_HEIGHT):

        try:
            field_width = int(self.sandbox_width_mm or width)
            field_height = int(self.sandbox_height_mm or height)

            buffer_x = int(field_width * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)
            buffer_y = int(field_height * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)

            destination_corners = np.array([
                [buffer_x, buffer_y],
                [field_width - 1 + buffer_x, buffer_y],
                [field_width - 1 + buffer_x, field_height - 1 + buffer_y],
                [buffer_x, field_height - 1 + buffer_y]
            ], dtype="float32")

            top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducialOrdering()

            fiducial_corners = np.array([
                self.cv_fiducial_cornerMarkerDict[top_left_id][0:2],
                self.cv_fiducial_cornerMarkerDict[top_right_id][0:2],
                self.cv_fiducial_cornerMarkerDict[bottom_right_id][0:2],
                self.cv_fiducial_cornerMarkerDict[bottom_left_id][0:2]
            ], dtype="float32")

            M = cv.getPerspectiveTransform(fiducial_corners, destination_corners)
            self._homography_M = np.asarray(M, dtype=np.float64)
            self._sandbox_pixel_size = (int(field_width + buffer_x * 2), int(field_height + buffer_y * 2))
            self._viz_homography_M = None
            self._viz_scale = None
            self._viz_size = None

            corner_ids = (top_left_id, top_right_id, bottom_right_id, bottom_left_id)
            self.cv_fiducial_warpedCornerMarkerDict = {}
            for fiducial_id in corner_ids:
                source_marker = np.array(
                    self.cv_fiducial_cornerMarkerDict[fiducial_id][2:6],
                    dtype="float32",
                ).reshape((-1, 1, 2))
                warped_marker = cv.perspectiveTransform(source_marker, M).reshape((-1, 2))
                warped_center = _projected_square_center(warped_marker)
                self.cv_fiducial_warpedCornerMarkerDict[fiducial_id] = (
                    float(warped_center[0]),
                    float(warped_center[1]),
                    *warped_marker.tolist(),
                    None,
                )

            sandbox_image = cv.warpPerspective(image_frame, M, self._sandbox_pixel_size)
            return sandbox_image
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_flattenSandboxImage: {e}")
            print(traceback.format_exc())
            raise

    def cv_fiducial_buildDisplayImage(self, image_frame, scale=None):
        """Warp camera frame directly into scaled display resolution in one pass."""
        if self._homography_M is None or self._sandbox_pixel_size is None:
            return None
        scale = float(constants.CV_VIZ_SCALE if scale is None else scale)
        if self._viz_homography_M is None or self._viz_scale != scale:
            scale_mat = np.array(
                [[scale, 0.0, 0.0], [0.0, scale, 0.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            self._viz_homography_M = scale_mat @ self._homography_M
            full_w, full_h = self._sandbox_pixel_size
            self._viz_size = (max(1, int(full_w * scale)), max(1, int(full_h * scale)))
            self._viz_scale = scale
        return cv.warpPerspective(image_frame, self._viz_homography_M, self._viz_size)

    def cv_fiducial_getCornerPositions(self):

        try:
            cornerFiducialIDs = constants.CORNER_FIDUCIALS
            foundCornerPositions = []
            for fiducialID in cornerFiducialIDs:
                if fiducialID in self.cv_fiducial_warpedCornerMarkerDict:
                    pose = list(self.cv_fiducial_warpedCornerMarkerDict[fiducialID][0:2]) + [None]
                    foundCornerPositions.append(pose)
            return foundCornerPositions
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getCornerPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_fiducial_getGoalPositions(self):

        try:
            goalFiducialIDs = constants.GOAL_FIDUCIALS
            foundGoalFiducialIDs = []
            for fiducialID in goalFiducialIDs:
                if fiducialID in self.cv_fiducial_markerDict.keys():
                    pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                    foundGoalFiducialIDs.append(pose)
            return foundGoalFiducialIDs
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getGoalPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_fiducial_getPalletPositions(self):

        try:
            palletFiducialIDs = constants.PALLET_FIDUCIALS
            foundPalletFiducialIDs = []
            for fiducialID in palletFiducialIDs:
                if fiducialID in self.cv_fiducial_markerDict.keys():
                    pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                    foundPalletFiducialIDs.append(pose)
            return foundPalletFiducialIDs
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getPalletPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_fiducial_getRobotPositions(self):

        try:
            robotFiducialIDs = constants.ROBOT_FIDUCIALS
            foundRobotFiducialPoses = []
            foundRobotFiducialIds = []
            for fiducialID in robotFiducialIDs:
                if fiducialID in self.cv_fiducial_markerDict.keys():
                    pose = list(self.cv_fiducial_markerDict[fiducialID][0:2]) + [self.cv_fiducial_markerDict[fiducialID][6]]
                    foundRobotFiducialPoses.append(pose)
                    foundRobotFiducialIds.append(fiducialID)
            return foundRobotFiducialPoses, foundRobotFiducialIds
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getRobotPositions: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_fiducial_getRobotPositionsMM(self):

        try:
            positions, marker_ids = self.cv_fiducial_getRobotPositions()
            return [self._warp_pixel_to_field_mm(pose) for pose in positions], marker_ids
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getRobotPositionsMM: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_fiducial_getPalletPositionsMM(self):

        try:
            return [
                self._warp_pixel_to_field_mm(pose)
                for pose in self.cv_fiducial_getPalletPositions()
            ]
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getPalletPositionsMM: {e}")
            print(traceback.format_exc())
            return []

    def cv_fiducial_getGoalPositionsMM(self):

        try:
            return [
                self._warp_pixel_to_field_mm(pose)
                for pose in self.cv_fiducial_getGoalPositions()
            ]
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getGoalPositionsMM: {e}")
            print(traceback.format_exc())
            return []
