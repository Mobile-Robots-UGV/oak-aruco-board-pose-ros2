import cv2

DEVICE = "/dev/v4l/by-id/usb-C7FOE20V3214300DE990_Integrated_Webcam_FHD_SN0001-video-index0"

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise RuntimeError(f"Could not open {DEVICE}")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

ok, frame = cap.read()
if not ok or frame is None:
    raise RuntimeError("Could not read frame")

print("actual:", frame.shape)
cap.release()
