"""Capture checkerboard images from the same webcam stream used by the vision system.

Run from this directory:
    python capture_calibration_images.py --camera 1

Controls:
    Images are saved automatically when the checkerboard is detected.
    Q/ESC - quit
"""

import argparse
import time
from pathlib import Path

import cv2 as cv


CHECKERBOARD_CORNERS = (9, 6)
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "images_calibration"


def find_checkerboard(gray):
    try:
        found, corners = cv.findChessboardCornersSB(
            gray, CHECKERBOARD_CORNERS, None
        )
        if found:
            return found, corners
    except AttributeError:
        pass

    flags = (
        cv.CALIB_CB_ADAPTIVE_THRESH
        + cv.CALIB_CB_NORMALIZE_IMAGE
        + cv.CALIB_CB_FILTER_QUADS
    )
    return cv.findChessboardCorners(gray, CHECKERBOARD_CORNERS, None, flags)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=1)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--interval", type=float, default=1.5)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    # Camo often exposes a usable stream through the default Windows backend,
    # while forcing DirectShow can produce a black frame on some systems.
    cap = cv.VideoCapture(args.camera)
    if not cap.isOpened():
        cap.release()
        cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera}")

    saved_count = len(list(args.output.glob("calibration*.*")))
    last_saved_at = 0.0
    print(f"Camera opened: {args.camera}")
    print("Move the 9x6 inner-corner checkerboard through the frame.")
    print(f"Images will save automatically every {args.interval:g}s when detected.")
    print("Press Q or ESC to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Cannot read a camera frame.")
                break

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            found, corners = find_checkerboard(gray)
            preview = frame.copy()
            if found:
                cv.drawChessboardCorners(
                    preview, CHECKERBOARD_CORNERS, corners, found
                )
                status = "READY - auto capture enabled"
                status_color = (0, 200, 0)
            else:
                status = "checkerboard not found"
                status_color = (0, 0, 255)

            height, width = frame.shape[:2]
            cv.putText(
                preview,
                f"{width}x{height} | saved: {saved_count} | {status}",
                (20, 35),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
                cv.LINE_AA,
            )
            cv.imshow("Calibration capture", preview)

            now = time.monotonic()
            if found and now - last_saved_at >= args.interval:
                saved_count += 1
                output_path = args.output / f"calibration_{saved_count:03d}.png"
                while output_path.exists():
                    saved_count += 1
                    output_path = args.output / f"calibration_{saved_count:03d}.png"
                if cv.imwrite(str(output_path), frame):
                    last_saved_at = now
                    print(f"Saved: {output_path} ({width}x{height})")

            key = cv.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()
