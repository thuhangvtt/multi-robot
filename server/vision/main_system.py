import sys
import os
import traceback
import cv2 as cv

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'vision'))

from cv_main import CV
from cv_visualize import CVVisualizer
import constants


def main():
    """Chạy vòng lặp kiểm thử thị giác máy tính và hiển thị visualizer."""
    print("=" * 55)
    print("  PARROT Vision System — Detection Testing Mode")
    print("=" * 55)

    try:
        cv_system = CV()
        visualizer = CVVisualizer()

        cv_system.start_camera(constants.WEBCAM_ID)
    except Exception as e:
        print(f"[Main] FATAL: Khởi tạo vision thất bại: {e}")
        print(traceback.format_exc())
        sys.exit(1)

    try:
        print("[Main] Đang khởi tạo sandbox (tìm corner markers)...")
        cv_system.cv_InitComputerVision()
    except Exception as e:
        print(f"[Main] FATAL: cv_InitComputerVision thất bại: {e}")
        print(traceback.format_exc())
        cv_system.stop_camera()
        sys.exit(1)

    print("[Main] Bắt đầu vòng lặp nhận diện. Nhấn 'q' hoặc ESC để dừng.")
    print("-" * 55)

    try:
        while True:
            try:
                cv_system.cv_runLocalizer()
                visualizer.visualize(cv_system)
            except Exception as e:
                print(f"[Main] LỖI: Vòng lặp vision: {e}")
                print(traceback.format_exc())

            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("[Main] Yêu cầu dừng.")
                break

    except KeyboardInterrupt:
        print("\n[Main] Bị ngắt bởi người dùng (Ctrl+C).")

    finally:
        try:
            cv_system.stop_camera()
        except Exception as e:
            print(f"[Main] WARNING: stop_camera thất bại: {e}")
            print(traceback.format_exc())
        cv.destroyAllWindows()
        print("[Main] Hệ thống đã tắt sạch.")


if __name__ == "__main__":
    main()
