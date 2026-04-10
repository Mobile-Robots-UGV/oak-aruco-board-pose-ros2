 #!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import depthai as dai


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture ChArUco calibration images with a live GUI window.")
    parser.add_argument("--out", type=str, default="calib_images")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--cols", type=int, default=11)
    parser.add_argument("--square", type=float, default=0.015)
    parser.add_argument("--marker", type=float, default=0.011)
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", choices=DICT_MAP.keys())
    parser.add_argument("--min-corners", type=int, default=12)
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[args.dict])
    board = cv2.aruco.CharucoBoard((args.cols, args.rows), args.square, args.marker, aruco_dict)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    count = 0

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        q = cam.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

        pipeline.start()

        while pipeline.isRunning():
            frame = q.get().getCvFrame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            view = frame.copy()

            corners, ids, _ = detector.detectMarkers(gray)

            charuco_count = 0
            marker_count = 0

            if ids is not None and len(ids) > 0:
                marker_count = len(ids)
                cv2.aruco.drawDetectedMarkers(view, corners, ids)

                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=board,
                )

                if charuco_ids is not None and len(charuco_ids) > 0:
                    charuco_count = len(charuco_ids)
                    cv2.aruco.drawDetectedCornersCharuco(view, charuco_corners, charuco_ids)

            good = charuco_count >= args.min_corners

            cv2.putText(view, "SPACE: save image   q: quit", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(view, f"saved: {count}", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(view, f"markers: {marker_count}", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(view, f"charuco corners: {charuco_count}", (20, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if good else (0, 0, 255), 2)
            cv2.putText(view, f"status: {'GOOD' if good else 'BAD'}", (20, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if good else (0, 0, 255), 2)

            cv2.imshow("capture_calibration", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                if not good:
                    print(f"Not saved: only {charuco_count} ChArUco corners")
                    continue
                path = out_dir / f"calib_{count:03d}.png"
                cv2.imwrite(str(path), frame)
                print(f"Saved {path} with {charuco_count} ChArUco corners")
                count += 1

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()