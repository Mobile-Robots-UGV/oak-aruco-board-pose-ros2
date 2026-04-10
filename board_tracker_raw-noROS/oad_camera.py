import depthai as dai
import cv2

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    q = cam.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        frame = q.get().getCvFrame()
        cv2.imshow("oak_rgb", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
