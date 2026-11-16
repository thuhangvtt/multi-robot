import cv2 as cv
import numpy as np
import time
import threading
import traceback
import constants
from constants import debugPrint
from cv_fiducial import CV_Fiducial

class CV:

    def __init__(self):
        self.cv_fiducial = CV_Fiducial()
        self.latestImageRaw = None
        self.latestSandboxImage = None
        self._cvImage = None
        self._camera_thread = None
        self._camera_running = False
        self._cap = None
        self._frame_lock = threading.Lock()
        self._frame_id = 0
        self._captured_at = None
        self._localized_frame_id = -1
        self._localized_at = None

    def _open_camera_stream(self, candidates, max_retries=4):

        for attempt in range(1, max_retries + 1):
            for dev_id in candidates:
                for backend in [cv.CAP_DSHOW, cv.CAP_MSMF, cv.CAP_ANY]:
                    try:
                        cap = cv.VideoCapture(dev_id, backend)
                    except Exception:
                        continue
                    if cap.isOpened():
                        if constants.CV_USE_CAMERA_CALIBRATION and constants.CV_CAMERA_CALIBRATION_SIZE:
                            width, height = constants.CV_CAMERA_CALIBRATION_SIZE
                            cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
                            cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)

                        ret, test_frame = cap.read()
                        if ret and test_frame is not None:
                            print(f"[CV] Camera mở và nhận diện frame thành công trên index {dev_id} (backend={backend}, lần thử {attempt}).")
                            return cap
                        cap.release()
            print(f"[CV] Chưa nhận được frame ở lần thử {attempt}/{max_retries}. Đang giải phóng và reset lại camera...")
            time.sleep(1.0)
        return None

    def start_camera(self, webcam_id=None):

        try:
            if webcam_id is None:
                webcam_id = constants.WEBCAM_ID

            self._camera_running = True
            self._camera_thread = threading.Thread(
                target=self._updateImage,
                args=(webcam_id,),
                daemon=True
            )
            self._camera_thread.start()
            print(f"[CV] Camera thread started (webcam_id={webcam_id}), warming up...")

            start_t = time.time()
            while self._cvImage is None and self._camera_running and (time.time() - start_t < 8.0):
                time.sleep(0.2)

            if self._cvImage is not None:
                print("[CV] Camera ready.")
            else:
                print("[CV] Cảnh báo: Chưa nhận được frame đầu tiên sau thời gian khởi động.")
        except Exception as e:
            print(f"[CV] LỖI: start_camera: {e}")
            print(traceback.format_exc())
            raise

    def stop_camera(self):

        try:
            self._camera_running = False
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
            if self._camera_thread is not None:
                self._camera_thread.join(timeout=2)
                self._camera_thread = None
            self._cvImage = None
            print("[CV] Camera stopped.")
        except Exception as e:
            print(f"[CV] LỖI: stop_camera: {e}")
            print(traceback.format_exc())

    def _updateImage(self, webcam_id):

        self._cap = None
        consecutive_fails = 0
        try:
            candidates = [webcam_id]
            self._cap = self._open_camera_stream(candidates, max_retries=4)

            if self._cap is None or not self._cap.isOpened():
                print(f"[CV] LỖI: Không thể mở bất kỳ camera nào trong danh sách {candidates} sau nhiều lần thử!")
                self._camera_running = False
                return

            print("[CV] Flushing initial frames for exposure stabilization...")
            for _ in range(15):
                self._cap.read()

            while self._camera_running:
                if self._cap is None:
                    break
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    with self._frame_lock:

                        if self._cvImage is None or not np.array_equal(frame, self._cvImage):
                            self._cvImage = frame
                            self._frame_id += 1
                            self._captured_at = time.monotonic()
                    consecutive_fails = 0
                else:
                    consecutive_fails += 1

                    if consecutive_fails >= 50:
                        print("[CV] Camera mất tín hiệu quá lâu, đang tự động reset kết nối...")
                        try:
                            self._cap.release()
                        except Exception:
                            pass
                        self._cap = None
                        time.sleep(0.5)
                        self._cap = self._open_camera_stream(candidates, max_retries=3)
                        consecutive_fails = 0
                        if self._cap is None:
                            print("[CV] Không thể khôi phục kết nối camera sau khi reset!")
                            break
                    time.sleep(0.01)
        except Exception as e:
            print(f"[CV] LỖI: _updateImage: {e}")
            print(traceback.format_exc())
            self._camera_running = False
        finally:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

    def _cv_CaptureImage(self):

        try:
            deadline = time.time() + 8.0
            while self._cvImage is None:
                if not self._camera_running or time.time() > deadline:
                    raise RuntimeError("Không thể nhận khung hình từ camera (camera bị ngắt hoặc không mở được)!")
                time.sleep(0.1)
            with self._frame_lock:
                if self._captured_at is None or time.monotonic() - self._captured_at > constants.CV_MAX_FRAME_AGE_S:
                    raise RuntimeError("Camera frame is stale; motion must stop")
                return self._cvImage
        except Exception as e:
            print(f"[CV] LỖI: _cv_CaptureImage: {e}")
            print(traceback.format_exc())
            raise

    def cv_InitComputerVision(self):

        try:
            frame = self._prepare_frame(self._cv_CaptureImage())
            self.latestSandboxImage = self.cv_fiducial.cv_fiducial_setupSandbox(
                frame,
                frame_supplier=lambda: self._prepare_frame(self._cv_CaptureImage()),
            )
            print("[CV] Computer Vision Field Ready")
        except Exception as e:
            print(f"[CV] LỖI: cv_InitComputerVision: {e}")
            print(traceback.format_exc())
            raise

    def cv_runLocalizer(self):

        try:
            self._cv_CaptureImage()
            with self._frame_lock:
                frame, frame_id, captured_at = self._cvImage, self._frame_id, self._captured_at
            if frame_id == self._localized_frame_id:
                return False
            self.latestImageRaw = self._prepare_frame(frame)

            if self.latestImageRaw is None:
                debugPrint("[CV] No image captured")
                return

            self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
                self.latestImageRaw
            )
            self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage, captured_at)
            self._localized_frame_id = frame_id
            self._localized_at = captured_at
            return True
        except Exception as e:
            print(f"[CV] LỖI: cv_runLocalizer: {e}")
            print(traceback.format_exc())

            raise

    def _prepare_frame(self, frame):

        try:
            if not constants.CV_USE_CAMERA_CALIBRATION:
                return frame

            expected = constants.CV_CAMERA_CALIBRATION_SIZE
            if expected is None or tuple(expected) != (frame.shape[1], frame.shape[0]):
                raise RuntimeError("Set CV_CAMERA_CALIBRATION_SIZE to the original calibration resolution; stream must match")

            return cv.undistort(
                frame,
                constants.CAMERA_MATRIX,
                constants.DISTORTION_COEFFICIENTS,
            )
        except Exception as e:
            print(f"[CV] LỖI: _prepare_frame: {e}")
            print(traceback.format_exc())
            raise

    def assert_fresh(self, fid_ids=()):
        now = time.monotonic()
        if self._localized_at is None or now - self._localized_at > constants.CV_MAX_FRAME_AGE_S:
            raise RuntimeError("No fresh localized frame")

        for fid in fid_ids:
            if fid not in self.cv_fiducial.cv_fiducial_markerDict:
                raise RuntimeError(f"Marker {fid} has not been observed yet")

    def cv_GetMarkerPositionsMM(self):
        self.assert_fresh()
        return self.cv_fiducial.fresh_marker_positions_mm()

    def cv_getLatestSandboxImage(self):

        return self.latestSandboxImage

    def cv_GetRobotPositions(self):

        try:
            return self.cv_fiducial.cv_fiducial_getRobotPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetRobotPositions: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_GetRobotPositionsMM(self):
        markers = self.cv_GetMarkerPositionsMM()
        ids = [fid for fid in constants.ROBOT_FIDUCIALS if fid in markers]
        return [markers[fid] for fid in ids], ids

    def cv_GetPalletPositions(self):

        try:
            return self.cv_fiducial.cv_fiducial_getPalletPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetPalletPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetPalletPositionsMM(self):

        try:
            return self.cv_fiducial.cv_fiducial_getPalletPositionsMM()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetPalletPositionsMM: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetPalletPositionsOffset(self):

        pallet_positions = self.cv_GetPalletPositionsMM()
        modified = []
        for pose in pallet_positions:
            px, py, p_theta = pose
            approach_theta = (p_theta + getattr(constants, 'CV_PALLET_APPROACH_ANGLE_OFFSET', 0.0)) % (2 * math.pi)
            dock_x = px - getattr(constants, 'CV_PALLET_OFFSET', 0.0) * math.cos(approach_theta)
            dock_y = py + getattr(constants, 'CV_PALLET_OFFSET', 0.0) * math.sin(approach_theta)
            modified.append([dock_x, dock_y, approach_theta])
        return modified

    def cv_GetGoalPositions(self):

        try:
            return self.cv_fiducial.cv_fiducial_getGoalPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetGoalPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetGoalPositionsMM(self):

        try:
            return self.cv_fiducial.cv_fiducial_getGoalPositionsMM()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetGoalPositionsMM: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetCornerPositions(self):

        try:
            return self.cv_fiducial.cv_fiducial_getCornerPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetCornerPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetSandboxSize(self):

        w = self.cv_fiducial.sandbox_width_mm or constants.CV_SANDBOX_WIDTH
        h = self.cv_fiducial.sandbox_height_mm or constants.CV_SANDBOX_HEIGHT
        return w, h

    def cv_GetAllMarkerData(self):

        return self.cv_fiducial.cv_fiducial_markerDict
