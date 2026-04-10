#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate a camera from ChArUco images.")
    parser.add_argument("--images", type=str, default="calib_images")
    parser.add_argument("--rows", type=int, default=8, help="Number of squares vertically")
    parser.add_argument("--cols", type=int, default=11, help="Number of squares horizontally")
    parser.add_argument("--square", type=float, default=0.015, help="Square size in meters")
    parser.add_argument("--marker", type=float, default=0.011, help="Marker size in meters")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", choices=DICT_MAP.keys())
    parser.add_argument("--out", type=str, default="camera_calib.npz")
    args = parser.parse_args()

    image_dir = Path(args.images)
    image_paths = sorted([p for p in image_dir.iterdir() if p.suffix.lower() in {".png", ".jpg", ".jpeg"}])
    if not image_paths:
        raise RuntimeError(f"No images found in {image_dir}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[args.dict])
    board = cv2.aruco.CharucoBoard((args.cols, args.rows), args.square, args.marker, aruco_dict)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            print(f"Skipped {path.name}: no ArUco markers found")
            continue

        n_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=board,
        )

        if charuco_ids is None or len(charuco_ids) < 6:
            print(f"Skipped {path.name}: not enough ChArUco corners")
            continue

        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        image_size = gray.shape[::-1]
        print(f"Used {path.name}: {len(charuco_ids)} corners")

    if len(all_charuco_corners) < 10:
        raise RuntimeError(f"Only {len(all_charuco_corners)} valid calibration images found. Need at least 10.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
    )

    np.savez(
        args.out,
        rms=rms,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=image_size[0],
        image_height=image_size[1],
        rows=args.rows,
        cols=args.cols,
        square_size_m=args.square,
        marker_size_m=args.marker,
        dictionary=args.dict,
    )

    print(f"Calibration RMS error: {rms:.6f}")
    print("camera_matrix=")
    print(camera_matrix)
    print("dist_coeffs=")
    print(dist_coeffs)
    print(f"Saved calibration to {args.out}")


if __name__ == "__main__":
    main()
