'''
Computer Vision Module — Camera Capture & ArUco-based Localization.

Manages the camera stream via a background thread and delegates
all marker detection to the CV_Fiducial module. This module is
the single entry point for the vision pipeline.

Coordinate system note: Computer vision coordinates (Y-down) differ
from robot coordinates (Y-up). Downstream modules must handle conversion.
'''

import cv2 as cv
import numpy as np
import time
import threading
import constants
from constants import debugPrint
from cv_fiducial import CV_Fiducial


class CV:
    def __init__(self):
        self.cv_fiducial = CV_Fiducial()

        self.latestImageRaw = None
        self.latestSandboxImage = None

        # Camera thread state (managed by start_camera / stop_camera)
        self._cvImage = None
        self._camera_thread = None
        self._camera_running = False

    ##############################################
    #            Camera Management
    ##############################################

    def start_camera(self, webcam_id=None):
        '''Start the background camera capture thread.'''
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
        time.sleep(3)
        print("[CV] Camera ready.")

    def stop_camera(self):
        '''Stop the background camera capture thread and release resources.'''
        self._camera_running = False
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=2)
            self._camera_thread = None
        print("[CV] Camera stopped.")

    def _updateImage(self, webcam_id):
        '''Background thread: continuously reads frames from the webcam.'''
        cap = cv.VideoCapture(webcam_id)
        if not cap.isOpened():
            print(f"[CV] ERROR: Cannot open camera {webcam_id}")
            self._camera_running = False
            return
        print("[CV] Camera opened successfully.")
        while self._camera_running:
            ret, frame = cap.read()
            if ret:
                self._cvImage = frame
        cap.release()

    def _cv_CaptureImage(self):
        '''Returns the latest frame from the camera thread, blocking until available.'''
        while self._cvImage is None:
            print("[CV] Waiting on image...")
            time.sleep(0.1)
        return self._cvImage

    ##############################################
    #            Initialization
    ##############################################

    def cv_InitComputerVision(self):
        '''Initialize the sandbox: detect corner markers and compute perspective transform.'''
        self.latestSandboxImage = self.cv_fiducial.cv_fiducial_setupSandbox(
            self._cv_CaptureImage()
        )
        print("[CV] Computer Vision Field Ready")

    ##############################################
    #            Per-frame Localization
    ##############################################

    def cv_runLocalizer(self):
        '''Capture a frame, warp to sandbox view, and detect all ArUco markers.'''
        self.latestImageRaw = self._cv_CaptureImage()

        if self.latestImageRaw is None:
            debugPrint("[CV] No image captured")
            return

        self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
            self.latestImageRaw
        )

        # Detect all entities (robots, pallets, goals, corners) in the warped sandbox image
        self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)

    ##############################################
    #            Data Getters
    ##############################################

    def cv_getLatestSandboxImage(self):
        '''Returns the latest perspective-warped sandbox image.'''
        return self.latestSandboxImage

    def cv_GetRobotPositions(self):
        '''Returns (poses, ids) where each pose is [x, y, orientation] in warped image coords.'''
        return self.cv_fiducial.cv_fiducial_getRobotPositions()

    def cv_GetPalletPositions(self):
        '''Returns list of [x, y, orientation] for detected pallets.'''
        return self.cv_fiducial.cv_fiducial_getPalletPositions()

    def cv_GetGoalPositions(self):
        '''Returns list of [x, y, orientation] for detected goals.'''
        return self.cv_fiducial.cv_fiducial_getGoalPositions()

    def cv_GetCornerPositions(self):
        '''Returns list of [x, y, orientation] for corner markers in warped image.'''
        return self.cv_fiducial.cv_fiducial_getCornerPositions()

    def cv_GetSandboxSize(self):
        '''Returns (width_mm, height_mm) of the sandbox.'''
        return self.cv_fiducial.sandbox_width_mm, self.cv_fiducial.sandbox_height_mm

    def cv_GetAllMarkerData(self):
        '''Returns the full marker dict {id: (cx, cy, tl, tr, br, bl, orientation)} for advanced use.'''
        return self.cv_fiducial.cv_fiducial_markerDict
