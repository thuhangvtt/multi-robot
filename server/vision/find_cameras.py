
import cv2 as cv

print("Dang quet camera...")
for i in range(6):
    cap = cv.VideoCapture(i, cv.CAP_DSHOW)
    if cap.isOpened():
        ret, frame = cap.read()
        h, w = (frame.shape[0], frame.shape[1]) if ret and frame is not None else (0, 0)
        print(f"  Index {i}: MO DUOC — {w}x{h}")
        cap.release()
    else:
        cap = cv.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            h, w = (frame.shape[0], frame.shape[1]) if ret and frame is not None else (0, 0)
            print(f"  Index {i}: MO DUOC (no DSHOW) — {w}x{h}")
            cap.release()
        else:
            print(f"  Index {i}: khong co")
print("Xong!")
