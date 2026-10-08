'''
test_image.py — Static Image Vision Pipeline Test

Test the full ArUco detection + visualization pipeline on a single
still image (no camera needed). Useful for:
  - Verifying detection logic with known test images
  - CI/offline validation without hardware
  - Debugging detection or visualization issues frame-by-frame

Usage:
    cd server
    .venv\\Scripts\\python.exe test_image.py path/to/image.jpg

    # Or use the built-in test image (if CV_DEBUG_IMAGE_PATH is set):
    .venv\\Scripts\\python.exe test_image.py

Output:
    - Displays the visualized image in an OpenCV window (press any key to close)
    - Saves the result to test_output.png in the same directory

Controls:
    Any key — Close window
    s       — Save to custom path
'''

import sys
import os

# =====================================================================
# Setup import paths (same as main_system.py)
# =====================================================================
# Lùi lại một cấp để BASE_DIR trỏ về thư mục 'server'
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'vision'))

import cv2 as cv
import numpy as np
import constants
from cv_fiducial import CV_Fiducial
from cv_visualize import CVVisualizer


# Inline "static image" version of CV — no camera thread needed
class StaticImageCV:
    '''
    Mimics the CV class interface but uses a pre-loaded image instead
    of a live camera stream. Allows CVVisualizer to work unchanged.
    '''
    def __init__(self, image_path: str):
        self.image_path = image_path
        self.cv_fiducial = CV_Fiducial()
        self.latestSandboxImage = None
        self._raw_image = None
        self.sandbox_warped = False

    def load(self):
        '''Load image from disk and run sandbox detection.'''
        img = cv.imread(self.image_path)
        if img is None:
            raise FileNotFoundError(f"Cannot read image: {self.image_path}")
        self._raw_image = img
        print(f"[Test] Loaded image: {self.image_path}  ({img.shape[1]}x{img.shape[0]})")
        return self

    def run_pipeline(self):
        '''
        Replicate the two-phase pipeline from CV class:
          Phase 1: detect sandbox corners (cv_fiducial_setupSandbox)
          Phase 2: detect all markers in the warped view
        '''
        # Phase 1: detect corner ArUco markers and warp to top-down view
        print("[Test] Phase 1: detecting sandbox corners...")
        detected = self.cv_fiducial._cv_fiducial_detectSandboxCorners(self._raw_image)
        if not detected:
            print("[Test] WARNING: No sandbox corner markers found!")
            print("       Falling back to raw image as sandbox view.")
            self.latestSandboxImage = self._raw_image.copy()
        else:
            self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
                self._raw_image
            )
            self.sandbox_warped = True
            print(f"[Test] Sandbox warped: {self.latestSandboxImage.shape[1]}x{self.latestSandboxImage.shape[0]}")

        # Phase 2: detect all markers (robots, pallets, goals) in warped image
        print("[Test] Phase 2: detecting markers in sandbox view...")
        self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)

        # Print summary
        robot_poses, robot_ids = self.cv_fiducial.cv_fiducial_getRobotPositions()
        pallet_poses = self.cv_fiducial.cv_fiducial_getPalletPositions()
        goal_poses = self.cv_fiducial.cv_fiducial_getGoalPositions()
        corner_poses = self.cv_fiducial.cv_fiducial_getCornerPositions()
        print(f"[Test] Found: {len(robot_ids)} robot(s)  {len(pallet_poses)} pallet(s)"
              f"  {len(goal_poses)} goal(s)  {len(corner_poses)} corner(s) (in warped view)")
        if robot_ids:
            print(f"       Robot IDs: {robot_ids}")
        return self

    # CV interface methods expected by CVVisualizer
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


# Visualize to image (no imshow — returns annotated frame)
def render_to_image(cv_obj: StaticImageCV) -> np.ndarray:
    '''
    Run the CVVisualizer on a StaticImageCV object and return the
    annotated image array instead of calling cv.imshow().
    '''
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
    
    # Không thu nhỏ ảnh khi lưu file tĩnh (dùng tỷ lệ 1:1)
    scale = 1.0

    # Reuse CVVisualizer drawing internals directly
    viz = CVVisualizer()

    # Corners
    for pose in cv_obj.cv_GetCornerPositions():
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 6, (0, 0, 255), -1)
        cv.circle(frame, (cx_s, cy_s), 8, (0, 0, 180), 2)
        cv.putText(frame, "C", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    # Pallets
    for i, pose in enumerate(cv_obj.cv_GetPalletPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 5, (0, 0, 255), -1)
        cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    # Goals
    for i, pose in enumerate(cv_obj.cv_GetGoalPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 5, (0, 0, 255), -1)
        cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv.LINE_AA)

    # Robots (no trail for static image — only one frame)
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

    # Overlay: detection summary text
    summary = (f"Robots:{len(robot_ids)}  "
               f"Pallets:{len(cv_obj.cv_GetPalletPositions())}  "
               f"Goals:{len(cv_obj.cv_GetGoalPositions())}")
    cv.putText(frame, summary, (10, 22),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2, cv.LINE_AA)

    return frame


# Main
def main():
    import glob
    
    input_dir = os.path.join(BASE_DIR, "vision", "computer_vision_fragment", "images_field_test")
    output_dir = os.path.join(BASE_DIR, "vision", "computer_vision_fragment", "field_test_result")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get all images matching 'field_test_*'
    search_pattern = os.path.join(input_dir, "field_test_*")
    image_paths = glob.glob(search_pattern)
    
    # Filter for standard image extensions
    valid_exts = {".jpg", ".jpeg", ".png", ".bmp"}
    image_paths = [p for p in image_paths if os.path.splitext(p)[1].lower() in valid_exts]
    
    if not image_paths:
        print(f"[Test] ERROR: No test images found matching '{search_pattern}' with valid extensions.")
        print(f"Make sure you have images like 'field_test_1.jpg' inside {input_dir}")
        return
        
    print(f"[Test] Found {len(image_paths)} images. Processing...")
    
    for img_path in sorted(image_paths):
        print("\n" + "="*50)
        filename = os.path.basename(img_path)
        
        name, ext = os.path.splitext(filename)
        out_name = name.replace("field_test_", "field_test_result_")
        if out_name == name:  # fallback if replace didn't match exactly
            out_name = f"result_{name}"
            
        output_path = os.path.join(output_dir, out_name + ext)
        
        try:
            # --- Run pipeline ---
            cv_static = StaticImageCV(img_path)
            cv_static.load().run_pipeline()
        
            # --- Render annotated image ---
            result = render_to_image(cv_static)
            
            # --- Save output ---
            cv.imwrite(output_path, result)
            print(f"[Test] SUCCESS: Result saved -> {output_path}")
            
        except Exception as e:
            print(f"[Test] ERROR processing {filename}: {e}")

    print("\n" + "="*50)
    print(f"[Test] Batch processing complete! Results saved in:\n{output_dir}")


if __name__ == "__main__":
    main()
