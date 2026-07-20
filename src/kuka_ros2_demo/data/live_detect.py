from ultralytics import YOLO
import cv2

model = YOLO("best_v2.pt")

cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

while True:
    ret, frame = cam.read()
    if not ret:
        break

    # Run YOLO
    results = model(frame, conf=0.25, verbose=False)

    # Draw detections
    detection_frame = results[0].plot()

    # Show original feed
    cv2.imshow("Camera Feed", frame)

    # Show detections
    cv2.imshow("YOLO Detection", detection_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cam.release()
cv2.destroyAllWindows()
