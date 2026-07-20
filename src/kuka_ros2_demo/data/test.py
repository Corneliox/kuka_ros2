from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO(
    "/home/emil/kuka_ros2/src/kuka_ros2_demo/data/YOLOV11/runs/segment/runs/segment/hardware_finetune_v2/weights/best.pt"
)

cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.5)

    result = results[0]

    if result.masks is not None:
        for mask, box in zip(result.masks.xy, result.boxes):
            cls = int(box.cls.item())
            name = model.names[cls]

            points = np.array(mask, dtype=np.int32)

            M = cv2.moments(points)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    name,
                    (cx, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

    cv2.imshow("Segmentation", result.plot())

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
