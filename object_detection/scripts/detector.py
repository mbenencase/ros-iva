import numpy as np
import torch
from beartype import beartype
from nptyping import NDArray, Shape, Float32, UInt8
from typing import NewType


Image = NewType("Image", NDArray[Shape["H, W, 3"], UInt8])
BoundingBoxes = NewType("BoundingBoxes", NDArray[Shape["N, 6"], Float32])


class Detector:
    def __init__(self) -> None:
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5n")

        # NOTE: Only cars for now
        self.model.classes = [2]

    @beartype
    def detect(self, img_bgr: Image) -> BoundingBoxes:
        img_rgb = np.array(img_bgr[:, :, ::-1])

        preds = self.model(img_rgb)
        return preds.xyxy[0].cpu().numpy().reshape(-1, 6)
