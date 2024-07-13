import cv2
import argparse
from ultralytics import YOLO
import supervision as sv
import numpy as np


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 Live")
    parser.add_argument(
        "--webcam-resolution",
        default=[1280, 720],
        nargs=2,
        type=int
    )
    args = parser.parse_args()
    return args

def main(args=None):
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    model = YOLO("Trash Detection.pt")

    # bounding box of the objects
    # see https://supervision.roboflow.com/annotators/
    # bounding_box_annotator = sv.BoundingBoxAnnotator(
    #     thickness = 2,
    #     color=sv.ColorPalette.DEFAULT,
    #     color_lookup = sv.ColorLookup.CLASS
    # )

    # label_annotator = sv.LabelAnnotator(
    #     text_scale=0.5,
    #     text_color=sv.Color.WHITE,
    #     text_thickness = 1,
    #     text_padding=10,
    #     text_position=sv.Position.TOP_LEFT,
    #     color=sv.ColorPalette.DEFAULT,
    #     color_lookup = sv.ColorLookup.CLASS
    # )

    box_annotator = sv.BoxAnnotator(
        thickness = 2,
        text_thickness = 1,
        text_scale = 0.5
    )

    while True:
        ret, frame = cap.read()

        result = model.predict(source=frame, show_boxes=False, verbose=False, show=False, conf=0.20)[0]
        
        detections = sv.Detections.from_yolov8(result)

        # display the details of the detections at the top of each object's bounding boxes
        labels = [
            f"{model.model.names[class_id]} {confidence:0.2f} {result.boxes.xyxy[0][0]}"
            for _, confidence, class_id, _
            in detections
        ]

        # frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
        # frame = label_annotator.annotate(scene=frame, detections=detections, labels=labels)
        frame = box_annotator.annotate(scene=frame, detections=detections, labels=labels)


        cv2.imshow("yolov8", frame)

        if (cv2.waitKey(30) == 27):
            break


if __name__ == "__main__":
    main()