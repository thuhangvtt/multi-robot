import cv2 as cv
import numpy as np
import time
import threading
import traceback
import constants
from constants import debugPrint
from cv_fiducial import CV_Fiducial


class CV:
    """Quản lý luồng camera và điều phối định vị ArUco marker."""

    def __init__(self):
        self.cv_fiducial = CV_Fiducial()
        self.latestImageRaw = None
        self.latestSandboxImage = None
        self._cvImage = None
        self._camera_thread = None
        self._camera_running = False
        self._cap = None

    def _open_camera_stream(self, candidates, max_retries=4):
        """Mở camera và xác nhận đọc được frame thật.
        Nếu driver còn bị lock/stale do lần ngắt kết nối trước, tự động giải phóng và thử lại."""
        for attempt in range(1, max_retries + 1):
            for dev_id in candidates:
                for backend in [cv.CAP_DSHOW, cv.CAP_MSMF, cv.CAP_ANY]:
                    try:
                        cap = cv.VideoCapture(dev_id, backend)
                    except Exception:
                        continue
                    if cap.isOpened():
                        # Đọc thử 1 frame để kiểm tra luồng stream thực sự có data
                        ret, test_frame = cap.read()
                        if ret and test_frame is not None:
                            print(f"[CV] Camera mở và nhận diện frame thành công trên index {dev_id} (backend={backend}, lần thử {attempt}).")
                            return cap
                        cap.release()
            print(f"[CV] Chưa nhận được frame ở lần thử {attempt}/{max_retries}. Đang giải phóng và reset lại camera...")
            time.sleep(1.0)
        return None

    def start_camera(self, webcam_id=None):
        """Khởi động luồng đọc camera ngầm."""
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

            # Chờ frame đầu tiên xuất hiện thay vì ngủ cứng 5s
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
        """Dừng luồng đọc camera và giải phóng tài nguyên."""
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
        """Đọc liên tục các khung hình từ camera trong luồng ngầm."""
        self._cap = None
        consecutive_fails = 0
        try:
            candidates = [webcam_id, 0, 1] if webcam_id not in (0, 1) else ([webcam_id, 1 - webcam_id])
            self._cap = self._open_camera_stream(candidates, max_retries=4)

            if self._cap is None or not self._cap.isOpened():
                print(f"[CV] LỖI: Không thể mở bất kỳ camera nào trong danh sách {candidates} sau nhiều lần thử!")
                self._camera_running = False
                return

            # Flush các frame đầu không ổn định
            print("[CV] Flushing initial frames for exposure stabilization...")
            for _ in range(15):
                self._cap.read()

            while self._camera_running:
                if self._cap is None:
                    break
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    self._cvImage = frame
                    consecutive_fails = 0
                else:
                    consecutive_fails += 1
                    # Nếu mất frame liên tục (> 50 frame ~ 1.5s), tự động reconnect/reset camera
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
        """Lấy khung hình mới nhất từ luồng camera."""
        try:
            deadline = time.time() + 8.0
            while self._cvImage is None:
                if not self._camera_running or time.time() > deadline:
                    raise RuntimeError("Không thể nhận khung hình từ camera (camera bị ngắt hoặc không mở được)!")
                time.sleep(0.1)
            return self._cvImage
        except Exception as e:
            print(f"[CV] LỖI: _cv_CaptureImage: {e}")
            print(traceback.format_exc())
            raise

    def cv_InitComputerVision(self):
        """Khởi tạo sandbox: nhận diện 4 góc và tính ma trận biến đổi phối cảnh."""
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
        """Chụp khung hình mới, làm phẳng phối cảnh và nhận diện toàn bộ marker."""
        try:
            self.latestImageRaw = self._prepare_frame(self._cv_CaptureImage())

            if self.latestImageRaw is None:
                debugPrint("[CV] No image captured")
                return

            self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
                self.latestImageRaw
            )
            self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)
        except Exception as e:
            print(f"[CV] LỖI: cv_runLocalizer: {e}")
            print(traceback.format_exc())

    def _prepare_frame(self, frame):
        """Khử méo ống kính camera dựa trên ma trận hiệu chuẩn."""
        try:
            if not constants.CV_USE_CAMERA_CALIBRATION:
                return frame

            return cv.undistort(
                frame,
                constants.CAMERA_MATRIX,
                constants.DISTORTION_COEFFICIENTS,
            )
        except Exception as e:
            print(f"[CV] LỖI: _prepare_frame: {e}")
            print(traceback.format_exc())
            return frame

    def cv_getLatestSandboxImage(self):
        """Lấy ảnh sandbox đã biến đổi góc nhìn thẳng từ trên xuống."""
        return self.latestSandboxImage

    def cv_GetRobotPositions(self):
        """Lấy danh sách vị trí robot theo pixel ảnh sandbox."""
        try:
            return self.cv_fiducial.cv_fiducial_getRobotPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetRobotPositions: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_GetRobotPositionsMM(self):
        """Lấy danh sách vị trí robot theo mm thực tế."""
        try:
            return self.cv_fiducial.cv_fiducial_getRobotPositionsMM()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetRobotPositionsMM: {e}")
            print(traceback.format_exc())
            return [], []

    def cv_GetPalletPositions(self):
        """Lấy danh sách vị trí pallet theo pixel."""
        try:
            return self.cv_fiducial.cv_fiducial_getPalletPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetPalletPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetPalletPositionsMM(self):
        """Lấy danh sách vị trí pallet theo mm thực tế."""
        try:
            return self.cv_fiducial.cv_fiducial_getPalletPositionsMM()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetPalletPositionsMM: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetGoalPositions(self):
        """Lấy danh sách vị trí điểm đích theo pixel."""
        try:
            return self.cv_fiducial.cv_fiducial_getGoalPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetGoalPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetGoalPositionsMM(self):
        """Lấy danh sách vị trí điểm đích theo mm thực tế."""
        try:
            return self.cv_fiducial.cv_fiducial_getGoalPositionsMM()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetGoalPositionsMM: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetCornerPositions(self):
        """Lấy danh sách 4 góc sandbox theo pixel."""
        try:
            return self.cv_fiducial.cv_fiducial_getCornerPositions()
        except Exception as e:
            print(f"[CV] LỖI: cv_GetCornerPositions: {e}")
            print(traceback.format_exc())
            return []

    def cv_GetSandboxSize(self):
        """Lấy kích thước chiều rộng và chiều cao thực tế của sandbox (mm)."""
        w = self.cv_fiducial.sandbox_width_mm or constants.CV_SANDBOX_WIDTH
        h = self.cv_fiducial.sandbox_height_mm or constants.CV_SANDBOX_HEIGHT
        return w, h

    def cv_GetAllMarkerData(self):
        """Lấy toàn bộ từ điển dữ liệu marker đã nhận diện."""
        return self.cv_fiducial.cv_fiducial_markerDict
