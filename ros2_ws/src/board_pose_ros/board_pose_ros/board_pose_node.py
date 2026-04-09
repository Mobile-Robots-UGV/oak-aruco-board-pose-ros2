#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Tuple

import cv2
import depthai as dai
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
from tf2_ros import TransformBroadcaster


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
        [x0, y0, 0.0],  # TL
        [x1, y0, 0.0],  # TR
        [x1, y1, 0.0],  # BR
        [x0, y1, 0.0],  # BL
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


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    trace = np.trace(R)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


class BoardPoseNode(Node):
    def __init__(self):
        super().__init__("board_pose_node")

        self.declare_parameter("calib", "/home/tatwik-24/ros2_ws/install/board_pose_ros/share/board_pose_ros/config/camera_calib_oak.npz")
        self.declare_parameter("config", "/home/tatwik-24/ros2_ws/install/board_pose_ros/share/board_pose_ros/config/board_config.json")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("camera_frame", "oak_camera_frame")
        self.declare_parameter("board_frame", "board_frame")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("log_pose", True)
        self.declare_parameter("log_every_n", 10)
        self.declare_parameter("log_rpy_degrees", False)

        calib_path = str(self.get_parameter("calib").value)
        config_path = str(self.get_parameter("config").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        alpha = float(self.get_parameter("alpha").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.board_frame = str(self.get_parameter("board_frame").value)
        fps = float(self.get_parameter("fps").value)
        self.log_pose = bool(self.get_parameter("log_pose").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.log_rpy_degrees = bool(self.get_parameter("log_rpy_degrees").value)
        self.log_counter = 0

        self.camera_matrix, self.dist_coeffs = load_calibration(calib_path)

        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        self.dictionary = get_dictionary(cfg.get("dictionary", "DICT_6X6_250"))
        self.marker_size = float(cfg["marker_size_m"])
        self.markers_cfg = cfg["markers"]

        self.pose_pub = self.create_publisher(PoseStamped, "board_pose", 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, "board_rpy", 10)
        self.visible_pub = self.create_publisher(Bool, "board_visible", 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, "board_used_ids", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.filt = EmaFilter(alpha=alpha)

        infos = dai.Device.getAllAvailableDevices()
        self.get_logger().info(f"DepthAI devices: {infos}")
        if not infos:
            raise RuntimeError("No available OAK / DepthAI devices")

        self.device = dai.Device(infos[0])
        self.pipeline = dai.Pipeline(self.device)
        self.cam = self.pipeline.create(dai.node.Camera).build()
        self.queue = self.cam.requestOutput(
            (width, height),
            type=dai.ImgFrame.Type.BGR888p
        ).createOutputQueue()
        self.pipeline.start()

        self.timer = self.create_timer(1.0 / fps, self.tick)

        self.get_logger().info("board_pose_node started")
        self.get_logger().info(f"calib={calib_path}")
        self.get_logger().info(f"config={config_path}")
        self.get_logger().info(f"width={width}, height={height}")
        self.get_logger().info(
            f"log_pose={self.log_pose}, log_every_n={self.log_every_n}, "
            f"log_rpy_degrees={self.log_rpy_degrees}"
        )

    def publish_visible(self, visible: bool):
        msg = Bool()
        msg.data = visible
        self.visible_pub.publish(msg)

    def maybe_log(self, message: str):
        self.log_counter += 1
        if self.log_pose and self.log_every_n > 0 and (self.log_counter % self.log_every_n == 0):
            self.get_logger().info(message)

    def tick(self):
        frame = self.queue.get().getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray, self.dictionary)

        if ids is None or len(ids) == 0:
            self.publish_visible(False)
            self.maybe_log("visible=False ids=[]")
            return

        object_points, image_points, used_ids = build_correspondences(
            corners, ids, self.markers_cfg, self.marker_size
        )

        if object_points is None or len(object_points) < 4:
            self.publish_visible(False)
            detected_ids = ",".join(str(int(i)) for i in ids.flatten())
            self.maybe_log(f"visible=False detected_ids=[{detected_ids}] matched_ids=[]")
            return

        ok_pnp, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok_pnp:
            self.publish_visible(False)
            ids_str = ",".join(str(i) for i in sorted(used_ids))
            self.maybe_log(f"visible=False ids=[{ids_str}] solvePnP_failed=True")
            return

        tvec = self.filt.update(tvec.reshape(3))
        R, _ = cv2.Rodrigues(rvec)

        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
        roll, pitch, yaw = rotation_matrix_to_rpy(R)

        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = stamp
        rpy_msg.header.frame_id = self.camera_frame
        rpy_msg.vector.x = roll
        rpy_msg.vector.y = pitch
        rpy_msg.vector.z = yaw
        self.rpy_pub.publish(rpy_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = self.board_frame
        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        ids_msg = Int32MultiArray()
        ids_msg.data = sorted(used_ids)
        self.ids_pub.publish(ids_msg)

        self.publish_visible(True)

        ids_str = ",".join(str(i) for i in sorted(used_ids))
        if self.log_rpy_degrees:
            self.maybe_log(
                "visible=True "
                f"ids=[{ids_str}] "
                f"x={float(tvec[0]):.4f} "
                f"y={float(tvec[1]):.4f} "
                f"z={float(tvec[2]):.4f} "
                f"rx_deg={math.degrees(roll):.2f} "
                f"ry_deg={math.degrees(pitch):.2f} "
                f"rz_deg={math.degrees(yaw):.2f}"
            )
        else:
            self.maybe_log(
                "visible=True "
                f"ids=[{ids_str}] "
                f"x={float(tvec[0]):.4f} "
                f"y={float(tvec[1]):.4f} "
                f"z={float(tvec[2]):.4f} "
                f"rx={roll:.4f} "
                f"ry={pitch:.4f} "
                f"rz={yaw:.4f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BoardPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()