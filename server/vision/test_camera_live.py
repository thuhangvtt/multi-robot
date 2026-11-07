
import sys
import os
import cv2 as cv
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))

import constants

# ArUco detector (tham so noi long)
aruco_dict   = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
aruco_params = cv.aruco.DetectorParameters()
aruco_params.minMarkerPerimeterRate      = 0.01
aruco_params.polygonalApproxAccuracyRate = 0.12
detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)

COLOR_CORNER = (0,   255,  0)
COLOR_ROBOT  = (255, 100,  0)
COLOR_PALLET = (0,   180, 255)
COLOR_GOAL   = (180,   0, 255)
COLOR_OTHER  = (200, 200, 200)

def id_color(fid):
    if fid in constants.CORNER_FIDUCIALS: return COLOR_CORNER, "CORNER"
    if fid in constants.ROBOT_FIDUCIALS:  return COLOR_ROBOT,  "ROBOT"
    if fid in constants.PALLET_FIDUCIALS: return COLOR_PALLET, "PALLET"
    if fid in constants.GOAL_FIDUCIALS:   return COLOR_GOAL,   "GOAL"
    return COLOR_OTHER, "?"

cap = cv.VideoCapture(constants.WEBCAM_ID)
if not cap.isOpened():
    cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("LOI: Khong mo duoc camera!")
    sys.exit(1)

print("Camera opened. Nhan 'q' de thoat.")
print(f"Corner IDs can detect: {constants.CORNER_FIDUCIALS}")

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    corners_list, ids, _ = detector.detectMarkers(frame)

    detected_corners = []
    if ids is not None:
        ids_flat = ids.flatten()
        for (marker_corner, fid) in zip(corners_list, ids_flat):
            pts = marker_corner.reshape((4, 2)).astype(int)
            color, label = id_color(fid)

            cv.polylines(frame, [pts], isClosed=True, color=color, thickness=2)

            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            cv.circle(frame, (cx, cy), 5, color, -1)
            cv.putText(frame, f"ID{fid} {label}", (cx + 8, cy - 8),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv.LINE_AA)

            if fid in constants.CORNER_FIDUCIALS:
                detected_corners.append(fid)

    missing     = [c for c in constants.CORNER_FIDUCIALS if c not in detected_corners]
    status      = f"Corners: {sorted(detected_corners)}"
    missing_txt = f"Missing: {missing}" if missing else "ALL 4 CORNERS FOUND! OK"
    hud_color   = (0, 255, 0) if not missing else (0, 100, 255)

    cv.putText(frame, status,      (10, 28), cv.FONT_HERSHEY_SIMPLEX, 0.65, hud_color, 2)
    cv.putText(frame, missing_txt, (10, 55), cv.FONT_HERSHEY_SIMPLEX, 0.65, hud_color, 2)
    total = 0 if ids is None else len(ids)
    cv.putText(frame, f"Total detected: {total}", (10, 80),
               cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    h, w  = frame.shape[:2]
    scale = min(1.0, 1200 / w)
    disp  = cv.resize(frame, (int(w * scale), int(h * scale)))
    cv.imshow("Camera Live Test - ArUco Detection", disp)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
