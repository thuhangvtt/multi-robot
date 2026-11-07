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
_aruco_params.adaptiveThreshWinSizeMax = 53
_aruco_params.adaptiveThreshWinSizeStep = 4
_aruco_params.minMarkerPerimeterRate = 0.01   # nới lỏng hơn (mặc định 0.02) → detect marker nhỏ/xa hơn
_aruco_params.maxMarkerPerimeterRate = 4.0
_aruco_params.polygonalApproxAccuracyRate = 0.12  # chấp nhận hình dạng méo hơn (mặc định 0.05)
_aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
_aruco_detector = cv.aruco.ArucoDetector(_aruco_dict, _aruco_params)


def _estimate_pose_single_marker(marker_corner, marker_size, camera_matrix, dist_coeffs):
    """Ước lượng tư thế 3D (rvec, tvec) của một ArUco marker."""
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
    """Tính tâm marker từ giao điểm 2 góc đối diện."""
    return (corners[0] + corners[2]) / 2.0


def _projected_square_center(corners):
    """Tính tâm phối cảnh chính xác từ giao điểm 2 đường chéo."""
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
    """Xử lý phát hiện ArUco marker và biến đổi phối cảnh sandbox."""

    def __init__(self):
        self.cv_fiducial_markerDict = {}
        self.cv_fiducial_cornerMarkerDict = {}
        self.cv_fiducial_warpedCornerMarkerDict = {}
        self.mm_per_pixel = None
        self.sandbox_height_mm = constants.CV_SANDBOX_HEIGHT
        self.sandbox_width_mm = constants.CV_SANDBOX_WIDTH
        self._last_robot_ids_logged = None
        self._marker_missed_frames = {}

    def _apply_center_offset(self, fiducial_id, center_x, center_y, orientation):
        """Bù sai lệch tâm giữa marker và tâm thực tế của vật thể."""
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
        """Chuyển tọa độ pixel trong ảnh đã làm phẳng sang mm thực tế trên sân."""
        try:
            buffer_x = constants.CV_SANDBOX_WIDTH * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
            buffer_y = constants.CV_SANDBOX_HEIGHT * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
            return [pose[0] - buffer_x, pose[1] - buffer_y, pose[2]]
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: _warp_pixel_to_field_mm: {e}")
            print(traceback.format_exc())
            return list(pose)

    def cv_fiducial_setupSandbox(self, image_frame, frame_supplier=None):
        """Chờ và phát hiện đủ 4 marker góc sandbox, sau đó tính ảnh phẳng ban đầu."""
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
        """Nhận diện các marker góc sandbox trên ảnh gốc."""
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
                            constants.DISTORTION_COEFFICIENTS)

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

    def cv_fiducial_generatePalletLocations(self, sandbox_image):
        """Nhận diện vị trí và hướng của tất cả marker trên ảnh sandbox đã làm phẳng."""
        try:
            corner_list, fiducial_ids, _ = _aruco_detector.detectMarkers(sandbox_image)

            detected_ids = [] if fiducial_ids is None else fiducial_ids.flatten()
            detected_id_set = {int(fiducial_id) for fiducial_id in detected_ids}
            tracked_ids = set(
                constants.ROBOT_FIDUCIALS
                + constants.PALLET_FIDUCIALS
                + constants.GOAL_FIDUCIALS
            )
            for tracked_id in tracked_ids:
                if tracked_id in detected_id_set:
                    self._marker_missed_frames[tracked_id] = 0
                elif tracked_id in self.cv_fiducial_markerDict:
                    missed_frames = self._marker_missed_frames.get(tracked_id, 0) + 1
                    self._marker_missed_frames[tracked_id] = missed_frames
                    if missed_frames > constants.CV_MARKER_HOLD_FRAMES:
                        self.cv_fiducial_markerDict.pop(tracked_id, None)

            detected_robot_ids = sorted(
                int(fiducial_id)
                for fiducial_id in detected_ids
                if int(fiducial_id) in constants.ROBOT_FIDUCIALS
            )
            if detected_robot_ids != self._last_robot_ids_logged:
                missing_robot_ids = [
                    fiducial_id
                    for fiducial_id in constants.ROBOT_FIDUCIALS
                    if fiducial_id not in detected_robot_ids
                ]
                print(f"[CV] Detected robot IDs: {detected_robot_ids or 'none'}")
                print(f"[CV] Missing robot IDs: {missing_robot_ids}")
                self._last_robot_ids_logged = detected_robot_ids

            if fiducial_ids is not None and len(corner_list) > 0:
                fiducial_ids = fiducial_ids.flatten()
                for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                    corners = marker_corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    marker_center = _marker_center_from_corners(corners)
                    centerX = int(round(float(marker_center[0])))
                    centerY = int(round(float(marker_center[1])))

                    orientation = math.atan2(topLeft[1] - bottomLeft[1], topLeft[0] - bottomLeft[0])
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

                    self.cv_fiducial_markerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, orientation)
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_generatePalletLocations: {e}")
            print(traceback.format_exc())

    def _cv_fiducial_findCornerFiducialOrdering(self):
        """Sắp xếp 4 marker góc theo thứ tự: top-left, top-right, bottom-left, bottom-right."""
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
        """Biến đổi phối cảnh ảnh sandbox từ góc nhìn camera thành góc nhìn từ trên xuống (nằm ngang chuẩn)."""
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

            sandbox_image = cv.warpPerspective(image_frame, M, (field_width + buffer_x * 2, field_height + buffer_y * 2))
            return sandbox_image
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_flattenSandboxImage: {e}")
            print(traceback.format_exc())
            raise

    def cv_fiducial_getCornerPositions(self):
        """Lấy danh sách vị trí 4 góc sân."""
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
        """Lấy danh sách vị trí các điểm đích."""
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
        """Lấy danh sách vị trí các pallet theo pixel."""
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
        """Lấy vị trí và ID các robot nhận diện được theo pixel."""
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
        """Lấy vị trí và ID các robot theo mm thực tế."""
        try:
            positions, marker_ids = self.cv_fiducial_getRobotPositions()
            return [self._warp_pixel_to_field_mm(pose) for pose in positions], marker_ids
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getRobotPositionsMM: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_fiducial_getPalletPositionsMM(self):
        """Lấy vị trí các pallet theo mm thực tế."""
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
        """Lấy vị trí các điểm đích theo mm thực tế."""
        try:
            return [
                self._warp_pixel_to_field_mm(pose)
                for pose in self.cv_fiducial_getGoalPositions()
            ]
        except Exception as e:
            print(f"[CV_Fiducial] LỖI: cv_fiducial_getGoalPositionsMM: {e}")
            print(traceback.format_exc())
            return []