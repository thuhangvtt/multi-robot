'''
PARROT Robot Controller — Main System Entry Point (Vision Testing Mode)

Runs the computer vision pipeline and displays detection results
in a live visualizer window. Used for testing ArUco marker detection
and sandbox calibration without the planner or controller.

Usage:
    cd server
    python vision/main_system.py

Controls:
    q / ESC — Quit
'''

import sys
import os

# =====================================================================
# Setup import paths so modules can use bare `import constants`,
# `from cv_fiducial import CV_Fiducial`, etc.
# =====================================================================
# Lùi lại một cấp để BASE_DIR trỏ về thư mục 'server'
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'vision'))

import cv2 as cv
from cv_main import CV
from cv_visualize import CVVisualizer
import constants


def main():
    print("=" * 55)
    print("  PARROT Vision System — Detection Testing Mode")
    print("=" * 55)

    # ------------------------------------------------------------------
    # 1. Initialize modules
    # ------------------------------------------------------------------
    cv_system = CV()
    visualizer = CVVisualizer()

    # ------------------------------------------------------------------
    # 2. Start camera capture thread
    # ------------------------------------------------------------------
    cv_system.start_camera(constants.WEBCAM_ID)

    # ------------------------------------------------------------------
    # 3. Calibrate sandbox (detect 4 corner ArUco markers)
    # ------------------------------------------------------------------
    print("[Main] Initializing sandbox (looking for corner markers)...")
    cv_system.cv_InitComputerVision()

    print("[Main] Starting detection loop. Press 'q' or ESC to quit.")
    print("-" * 55)

    # ------------------------------------------------------------------
    # 4. Main detection + visualization loop
    # ------------------------------------------------------------------
    try:
        while True:
            # Capture frame → warp → detect all markers
            cv_system.cv_runLocalizer()

            # Draw overlays and show
            visualizer.visualize(cv_system)

            # Handle quit key
            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                print("[Main] Quit requested.")
                break

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user (Ctrl+C).")

    finally:
        cv_system.stop_camera()
        cv.destroyAllWindows()
        print("[Main] System shut down cleanly.")


if __name__ == "__main__":
    main()
