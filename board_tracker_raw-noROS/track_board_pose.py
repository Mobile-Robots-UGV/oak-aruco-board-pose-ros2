#!/usr/bin/env python3
import argparse
import json
from typing import Dict, List, Tuple

import cv2
import depthai as dai
import numpy as np


class EmaFilter:
    def __init__(self, alpha: float = 0.25):
        self.alpha = alpha
        self.value = None

    def update(self, x: np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = x.astype(np.float64)
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value.copy()


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


def detect_markers(gray: np.ndarray, dictionary):
    params = get_detector_params()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def load_calibration(path: str) -> Tuple[np.ndarray, np.ndarray]:
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def marker_obj_points(top_left_xy: List[float], marker_size: float, rotation_deg: int = 0) -> np.ndarray:
    x0, y0 = top_left_xy
    x1 = x0 + marker_size
    y1 = y0 - marker_size

    pts = np.array([
        [x0, y0, 0.0],
        [x1, y0, 0.0],
        [x1, y1, 0.0],
        [x0, y1, 0.0],
    ], dtype=np.float32)

    rot = rotation_deg % 360
    if rot == 0:
        order = [0, 1, 2, 3]
    elif rot == 90:
        order = [3, 0, 1, 2]
    elif rot == 180:
        order = [2, 3, 0, 1]
    elif rot == 270:
        order = [1, 2, 3, 0]
    else:
        raise ValueError(f"rotation_deg must be one of 0, 90, 180, 270, got {rotation_deg}")

    return pts[order]


def build_correspondences(
    corners: List[np.ndarray],
    ids: np.ndarray,
    markers_cfg: Dict[str, Dict[str, List[float]]],
    marker_size: float,
) -> Tuple[np.ndarray, np.ndarray, List[int]]:
    object_points = []
    image_points = []
    used_ids = []

    for marker_corners, marker_id_arr in zip(corners, ids.flatten()):
        marker_id = int(marker_id_arr)
        key = str(marker_id)
        if key not in markers_cfg:
            continue
        rotation_deg = int(markers_cfg[key].get("rotation_deg", 0))
        obj = marker_obj_points(markers_cfg[key]["top_left_xy_m"], marker_size, rotation_deg)
        img = marker_corners.reshape(4, 2).astype(np.float32)
        object_points.append(obj)
        image_points.append(img)
        used_ids.append(marker_id)

    if not object_points:
        return None, None, []

    return np.vstack(object_points), np.vstack(image_points), used_ids


def board_center_pixel(corners: List[np.ndarray], ids: np.ndarray, used_ids: List[int]) -> Tuple[int, int]:
    pts = []
    used = set(used_ids)
    for marker_corners, marker_id_arr in zip(corners, ids.flatten()):
        marker_id = int(marker_id_arr)
        if marker_id in used:
            pts.extend(marker_corners.reshape(4, 2))
    pts = np.array(pts, dtype=np.float32)
    center = np.mean(pts, axis=0)
    return int(center[0]), int(center[1])


def main() -> None:
    parser = argparse.ArgumentParser(description="Track board pose from ArUco corner markers using an OAK camera.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", type=str, default="camera_calib.npz")
    parser.add_argument("--config", type=str, default="board_config.json")
    parser.add_argument("--alpha", type=float, default=0.25)
    parser.add_argument("--draw-axes", action="store_true")
    args = parser.parse_args()

    camera_matrix, dist_coeffs = load_calibration(args.calib)

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    dictionary = get_dictionary(cfg.get("dictionary", "DICT_6X6_250"))
    marker_size = float(cfg["marker_size_m"])
    markers_cfg = cfg["markers"]

    filt = EmaFilter(alpha=args.alpha)

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        q = cam.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

        pipeline.start()

        while pipeline.isRunning():
            frame = q.get().getCvFrame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect_markers(gray, dictionary)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                object_points, image_points, used_ids = build_correspondences(
                    corners, ids, markers_cfg, marker_size
                )

                if object_points is not None and len(object_points) >= 4:
                    ok_pnp, rvec, tvec = cv2.solvePnP(
                        object_points,
                        image_points,
                        camera_matrix,
                        dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )

                    if ok_pnp:
                        tvec = filt.update(tvec.reshape(3))
                        x_m, y_m, z_m = tvec.tolist()
                        cx, cy = board_center_pixel(corners, ids, used_ids)

                        cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                        cv2.putText(frame, f"used ids: {sorted(used_ids)}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, f"X: {x_m*100:.1f} cm", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, f"Y: {y_m*100:.1f} cm", (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, f"Z: {z_m*100:.1f} cm", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, f"pixel center: ({cx}, {cy})", (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, "Board origin = board center", (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                        if args.draw_axes and hasattr(cv2, "drawFrameAxes"):
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec.reshape(3, 1), 0.05)
                    else:
                        cv2.putText(frame, "solvePnP failed", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "Markers detected, but none match your config IDs", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "No markers detected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(frame, "Press q to quit", (20, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("track_board_pose", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
