from abc import ABC, abstractmethod
import cv2
import numpy as np
from typing import List, Tuple
import time

class KeypointDetector(ABC):
    def __init__(self, image_path: str, verbosity: bool = False):
        self.image_path = image_path
        self.detector_name = "Base Keypoint Detector"
        self.verbosity = verbosity
        self.image = self._load_image()

    def _load_image(self) -> np.ndarray:
        image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Failed to load image from {self.image_path}")
        if self.verbosity:
            print(f"Loaded image: {self.image_path} with shape {image.shape}")
        return np.float32(image) / 255.0  # normalized to [0, 1]

    @abstractmethod
    def detect_keypoints(self) -> Tuple[List[Tuple[int, int]], np.ndarray]:
        pass

    def visualize(self, keypoints: List[Tuple[int, int]]) -> None:
        output_image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        for x, y in keypoints:
            cv2.circle(output_image, (x, y), 5, (0, 255, 0), 1)
        cv2.imshow(f'{self.detector_name} (Press ESC to Close)', output_image)
        time.sleep(1)
        while True:
            key = cv2.waitKey(1) & 0xFF  # poll for keypress
            if key == 27 and cv2.getWindowProperty(f'{self.detector_name} (Press ESC to Close)', cv2.WND_PROP_VISIBLE) < 1:  
                break
        cv2.destroyAllWindows()
