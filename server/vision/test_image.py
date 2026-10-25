import sys
import os
import glob
import traceback
import cv2 as cv
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'vision'))

import constants
from cv_fiducial import CV_Fiducial
from cv_visualize import CVVisualizer


class StaticImageCV:
    """Mô phỏng lớp CV nhưng sử dụng ảnh tĩnh để kiểm thử ngoại tuyến."""

    def __init__(self, image_path: str):
        self.image_path = image_path
        self.cv_fiducial = CV_Fiducial()
        self.latestSandboxImage = None
        self._raw_image = None
        self.sandbox_warped = False

    def load(self):
        """Đọc ảnh từ tệp tin."""
        img = cv.imread(self.image_path)
        if img is None:
            raise FileNotFoundError(f"Không thể đọc ảnh: {self.image_path}")
        self._raw_image = img
        print(f"[Test] Loaded image: {self.image_path}  ({img.shape[1]}x{img.shape[0]})")
        return self

    def run_pipeline(self):
        """Chạy pipeline nhận diện các điểm đánh dấu trên ảnh."""
        try:
            detected = self.cv_fiducial._cv_fiducial_detectSandboxCorners(self._raw_image)
        except Exception as e:
            print(f"[Test] LỖI: Phát hiện sandbox corners thất bại: {e}")
            print(traceback.format_exc())
            detected = False
        if not detected:
            print("[Test] CảNH BÁO: Không tìm thấy sandbox corner markers!")
            print("       Dùng ảnh gốc thay cho sandbox.")
            self.latestSandboxImage = self._raw_image.copy()
        else:
            try:
                self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
                    self._raw_image
                )
            except Exception as e:
                print(f"[Test] LỖI: Làm phẳng sandbox thất bại: {e}")
                print(traceback.format_exc())
                self.latestSandboxImage = self._raw_image.copy()
            else:
                self.sandbox_warped = True
                print(f"[Test] Sandbox đã biến đổi: {self.latestSandboxImage.shape[1]}x{self.latestSandboxImage.shape[0]}")

        try:
            self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)
        except Exception as e:
            print(f"[Test] LỖI: generatePalletLocations thất bại: {e}")
            print(traceback.format_exc())

        robot_poses, robot_ids = self.cv_fiducial.cv_fiducial_getRobotPositions()
        pallet_poses = self.cv_fiducial.cv_fiducial_getPalletPositions()
        goal_poses = self.cv_fiducial.cv_fiducial_getGoalPositions()
        corner_poses = self.cv_fiducial.cv_fiducial_getCornerPositions()
        print(f"[Test] Tìm thấy: {len(robot_ids)} robot  {len(pallet_poses)} pallet"
              f"  {len(goal_poses)} goal  {len(corner_poses)} corner")
        if robot_ids:
            print(f"       Robot IDs: {robot_ids}")
        return self

    def cv_getLatestSandboxImage(self):
        return self.latestSandboxImage

    def cv_GetRobotPositions(self):
        return self.cv_fiducial.cv_fiducial_getRobotPositions()

    def cv_GetPalletPositions(self):
        return self.cv_fiducial.cv_fiducial_getPalletPositions()

    def cv_GetGoalPositions(self):
        return self.cv_fiducial.cv_fiducial_getGoalPositions()

    def cv_GetCornerPositions(self):
        return self.cv_fiducial.cv_fiducial_getCornerPositions()


def render_to_image(cv_obj: StaticImageCV) -> np.ndarray:
    """Vẽ kết quả nhận diện lên ma trận ảnh tĩnh để lưu ra file."""
    sandbox_image = cv_obj.cv_getLatestSandboxImage()
    if sandbox_image is None:
        raise RuntimeError("No sandbox image to render.")

    frame = sandbox_image.copy()

    if not cv_obj.sandbox_warped:
        cv.putText(
            frame,
            "WARP SKIPPED: need all 4 corner markers",
            (10, 50),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv.LINE_AA,
        )
        for fiducial_id, data in cv_obj.cv_fiducial.cv_fiducial_cornerMarkerDict.items():
            center = (int(data[0]), int(data[1]))
            cv.circle(frame, center, 8, (0, 0, 255), 2)
            cv.putText(
                frame,
                f"ID {fiducial_id}",
                (center[0] + 10, center[1] - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )

    scale = 1.0

    for pose in cv_obj.cv_GetCornerPositions():
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 6, (0, 0, 255), -1)
        cv.circle(frame, (cx_s, cy_s), 8, (0, 0, 180), 2)
        cv.putText(frame, "C", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    for i, pose in enumerate(cv_obj.cv_GetPalletPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 5, (0, 0, 255), -1)
        cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    for i, pose in enumerate(cv_obj.cv_GetGoalPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 5, (0, 0, 255), -1)
        cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv.LINE_AA)

    robot_positions, robot_ids = cv_obj.cv_GetRobotPositions()
    for pose, fid in zip(robot_positions, robot_ids):
        cx, cy, orientation = pose[0], pose[1], pose[2]
        cx_s, cy_s = int(cx * scale), int(cy * scale)

        cv.circle(frame, (cx_s, cy_s), 5, (0, 0, 255), -1)

        arrow_len = 60 * scale
        end_x = int(cx_s + arrow_len * np.cos(orientation))
        end_y = int(cy_s - arrow_len * np.sin(orientation))
        cv.arrowedLine(frame, (cx_s, cy_s), (end_x, end_y),
                       (255, 0, 0), 2, tipLength=0.3)

        cv.putText(frame, f"R{fid}", (cx_s + 10, cy_s - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv.LINE_AA)

    summary = (f"Robots:{len(robot_ids)}  "
               f"Pallets:{len(cv_obj.cv_GetPalletPositions())}  "
               f"Goals:{len(cv_obj.cv_GetGoalPositions())}")
    cv.putText(frame, summary, (10, 22),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2, cv.LINE_AA)

    return frame


def main():
    """Kiểm thử hàng loạt ảnh tĩnh trong thư mục test."""
    input_dir = os.path.join(BASE_DIR, "vision", "computer_vision_fragment", "images_field_test")
    output_dir = os.path.join(BASE_DIR, "vision", "computer_vision_fragment", "field_test_result")

    os.makedirs(output_dir, exist_ok=True)

    search_pattern = os.path.join(input_dir, "field_test_*")
    image_paths = glob.glob(search_pattern)

    valid_exts = {".jpg", ".jpeg", ".png", ".bmp"}
    image_paths = [p for p in image_paths if os.path.splitext(p)[1].lower() in valid_exts]

    if not image_paths:
        print(f"[Test] LỖI: Không tìm thấy ảnh test khớp '{search_pattern}' với đuôi hợp lệ.")
        return

    print(f"[Test] Tìm thấy {len(image_paths)} ảnh. Bắt đầu xử lý...")

    for img_path in sorted(image_paths):
        print("\n" + "="*50)
        filename = os.path.basename(img_path)

        name, ext = os.path.splitext(filename)
        out_name = name.replace("field_test_", "field_test_result_")
        if out_name == name:
            out_name = f"result_{name}"

        output_path = os.path.join(output_dir, out_name + ext)

        try:
            cv_static = StaticImageCV(img_path)
            cv_static.load().run_pipeline()
            result = render_to_image(cv_static)
            cv.imwrite(output_path, result)
            print(f"[Test] THÀNH CÔNG: Kết quả lưu tại -> {output_path}")

        except Exception as e:
            print(f"[Test] LỖI khi xử lý {filename}: {e}")
            print(traceback.format_exc())

    print("\n" + "="*50)
    print(f"[Test] Xử lý hàng loạt hoàn thành! Kết quả lưu tại:\n{output_dir}")


if __name__ == "__main__":
    main()
