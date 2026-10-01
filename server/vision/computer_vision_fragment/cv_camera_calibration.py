import numpy as np
import cv2 as cv
import glob
import os
import sys

# Fix encoding cho Windows
sys.stdout.reconfigure(encoding='utf-8')

# =====================================================================
# [QUAN TRỌNG] CÁC THAM SỐ CẦN THAY ĐỔI TRƯỚC KHI CHẠY CODE
# =====================================================================

# 1. Kích thước góc bên trong của bàn cờ (Inner corners). 
# Bàn cờ 1078 ô => (9, 6) góc giao nhau bên trong.
CHECKERBOARD_CORNERS = (9,6) 

# 2. Kích thước thực tế của 1 cạnh ô vuông (đơn vị: mm).
SQUARE_SIZE_MM = 25.0 

# 3. Đường dẫn tới thư mục chứa ảnh
IMAGE_FOLDER_PATH = "images_calibration/calibration*.jpg"

# 4. Giới hạn số ảnh (đặt 0 để chạy hết)
MAX_IMAGES = 0

# 5. Resize ảnh xuống max bao nhiêu pixel (cạnh dài nhất) để xử lý nhanh
RESIZE_MAX = 99999

# =====================================================================
# BẮT ĐẦU QUÁ TRÌNH HIỆU CHUẨN
# =====================================================================

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD_CORNERS[0] * CHECKERBOARD_CORNERS[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_CORNERS[0], 0:CHECKERBOARD_CORNERS[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE_MM 

objpoints = []
imgpoints = []

images = glob.glob(IMAGE_FOLDER_PATH)

if len(images) == 0:
    print(f"LỖI: Không tìm thấy ảnh nào trong đường dẫn: {IMAGE_FOLDER_PATH}")
    exit()

if MAX_IMAGES > 0:
    images = images[:MAX_IMAGES]

print(f"Xử lý {len(images)} bức ảnh, tìm bàn cờ {CHECKERBOARD_CORNERS[0]}x{CHECKERBOARD_CORNERS[1]} góc.\n")

successful_images = 0
img_size = None

for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"  [!] Không đọc được: {fname}")
        continue
    
    basename = os.path.basename(fname)
    h_orig, w_orig = img.shape[:2]
    
    # Resize ảnh xuống để DETECT nhanh
    scale = 1.0
    if max(h_orig, w_orig) > RESIZE_MAX:
        scale = RESIZE_MAX / max(h_orig, w_orig)
        img_small = cv.resize(img, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
    else:
        img_small = img
    
    gray_small = cv.cvtColor(img_small, cv.COLOR_BGR2GRAY)
    gray_orig = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_size = (w_orig, h_orig)  # Lưu kích thước GỐC
    
    print(f"  Đang xử lý {basename} ({w_orig}x{h_orig} -> {gray_small.shape[1]}x{gray_small.shape[0]})...", end=" ", flush=True)
    
    # Detect góc trên ảnh NHỎ (nhanh)
    found = False
    corners = None
    
    try:
        found, corners = cv.findChessboardCornersSB(gray_small, CHECKERBOARD_CORNERS, None)
    except AttributeError:
        pass
    
    if not found:
        flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FILTER_QUADS
        found, corners = cv.findChessboardCorners(gray_small, CHECKERBOARD_CORNERS, None, flags)
    
    if not found:
        found, corners = cv.findChessboardCorners(gray_small, CHECKERBOARD_CORNERS, None)
    
    if found:
        # Scale tọa độ góc về kích thước ẢNH GỐC
        if scale != 1.0:
            corners = corners / scale
        
        # Tinh chỉnh sub-pixel trên ẢNH GỐC (chính xác nhất)
        corners2 = cv.cornerSubPix(gray_orig, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        successful_images += 1
        print("OK!")
    else:
        print("THẤT BẠI")

print(f"\n{'='*50}")
print(f"Kết quả: {successful_images}/{len(images)} ảnh nhận diện thành công.")
print(f"{'='*50}")

if successful_images == 0:
    print("\nLỖI: Không có ảnh nào nhận diện được góc!")
    print("GỢI Ý: In bàn cờ chuẩn trên giấy A4 (cần có viền trắng bao quanh)")
    exit()

if successful_images < 3:
    print(f"\nCẢNH BÁO: Chỉ có {successful_images} ảnh. Nên có ít nhất 3 ảnh.")

print("\nĐang tính ma trận Camera...")

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

print("\n================== KẾT QUẢ ĐẦU RA ==================")
print("Copy 2 biến dưới đây vào file constants.py:\n")

print("CAMERA_MATRIX = np.array([ \\")
for row in cameraMatrix:
    print(f"    [{row[0]:.8e}, {row[1]:.8e}, {row[2]:.8e}], \\")
print("])\n")

print("DISTORTION_COEFFICIENTS = np.array([ \\")
print(f"    [{distCoeffs[0][0]:.8e}, {distCoeffs[0][1]:.8e}, {distCoeffs[0][2]:.8e}, {distCoeffs[0][3]:.8e}, {distCoeffs[0][4]:.8e}] \\")
print("])")
print("====================================================\n")

print(f"Sai số (Reprojection Error): {ret:.4f}")
print("(Dưới 1.0 là tốt)")
