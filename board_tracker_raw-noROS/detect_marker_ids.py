#!/usr/bin/env python3
import argparse
import time
from typing import Tuple

import cv2
import depthai as dai
import numpy as np


def get_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("This OpenCV build has no aruco module. Install opencv-contrib-python.")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    dict_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def get_detector_params():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(gray: np.ndarray, dictionary) -> Tuple[list, np.ndarray, list]:
    params = get_detector_params()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids, rejected


def main() -> None:
    parser = argparse.ArgumentParser(description="Show ArUco marker IDs from an OAK camera stream.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--dict", type=str, default="DICT_6X6_250")
    args = parser.parse_args()

    dictionary = get_dictionary(args.dict)

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        q = cam.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

        pipeline.start()

        last_print = 0.0
        while pipeline.isRunning():
            frame = q.get().getCvFrame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect_markers(gray, dictionary)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                now = time.time()
                if now - last_print > 0.5:
                    unique_ids = sorted(int(x) for x in ids.flatten())
                    print("Detected IDs:", unique_ids)
                    last_print = now

            cv2.putText(frame, "Press q to quit", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("detect_marker_ids", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
