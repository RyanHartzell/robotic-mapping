import cv2
import numpy as np
from scipy.signal import convolve2d
from feature_detectors.base import KeypointDetector


class HarrisCornerDetector(KeypointDetector):
    def __init__(self, image_path: str, window_size: int = 3, k: float = 0.04, threshold: float = 0.1, gradient_method: str = 'scharr', verbosity=False):
        self.image_path = image_path
        self.detector_name = "Harris Corner Detector"
        self.window_size = window_size  # local area size
        self.k = k  # constant for Harris response function; typically [0.04, 0.06]
        self.threshold = threshold  # threshold for keypoint detection; find the highest response (r_max); only accept another keypoint if it's greater than threshold * r_max
        self.gradient_method = gradient_method
        self.verbosity = verbosity
        self.image = self._load_image()
        self.height, self.width = self.image.shape

        # sobel filters
        self.sobel_x = (1/8) * np.array([
            [-1, 0, 1],
            [-2, 0, 2],
            [-1, 0, 1]
        ], dtype=np.float32)

        self.sobel_y = (1/8) * np.array([
            [-1, -2, -1],
            [0, 0, 0],
            [1, 2, 1]
        ], dtype=np.float32)

        # scharr filters 
        self.scharr_x = (1/32) * np.array([
            [-3, 0, 3],
            [-10, 0, 10],
            [-3, 0, 3]
        ], dtype=np.float32)

        self.scharr_y = (1/32) * np.array([
            [-3, -10, -3],
            [0, 0, 0],
            [3, 10, 3]
        ], dtype=np.float32)

    def compute_gradients(self) -> tuple:
        """ Compute image gradients using Scharr or Sobel filters. """
        if self.gradient_method == 'scharr':
            kernel_x, kernel_y = self.scharr_x, self.scharr_y
        elif self.gradient_method == 'sobel':
            kernel_x, kernel_y = self.sobel_x, self.sobel_y
        else:
            raise ValueError("Gradient method must be 'scharr' or 'sobel'.")

        i_x = convolve2d(self.image, kernel_x, mode='same', boundary='fill', fillvalue=0)
        i_y = convolve2d(self.image, kernel_y, mode='same', boundary='fill', fillvalue=0)

        i_x2 = i_x ** 2
        i_y2 = i_y ** 2
        i_xy = i_x * i_y

        return i_x2, i_y2, i_xy
    
    def compute_structure_matrices(self, i_x2: np.ndarray, i_y2: np.ndarray, i_xy: np.ndarray) -> tuple:
        """ Compute structure matrix components by convolving squared gradients over a local area. """
        kernel = np.ones((self.window_size, self.window_size))

        s_x2 = convolve2d(i_x2, kernel, mode='same', boundary='fill', fillvalue=0)
        s_y2 = convolve2d(i_y2, kernel, mode='same', boundary='fill', fillvalue=0)
        s_xy = convolve2d(i_xy, kernel, mode='same', boundary='fill', fillvalue=0)

        return s_x2, s_y2, s_xy

    def compute_harris_response(self, s_x2: np.ndarray, s_y2: np.ndarray, s_xy: np.ndarray) -> np.ndarray:
        """ Compute the Harris response matrix using determinant and trace. """
        det_m = (s_x2 * s_y2) - (s_xy ** 2)
        trace_m = s_x2 + s_y2
        r = det_m - self.k * (trace_m ** 2)
        return r

    def detect_keypoints(self, r: np.ndarray) -> list:
        """ Identify keypoints where response exceeds threshold. """
        keypoints = []
        r_max = np.max(r)

        for y in range(self.height):
            for x in range(self.width):
                # unintuitve [y, x] logic is due to how computers stores image arrays; blame row-major ordering
                # TODO: non-maxima suppression
                if r[y, x] > self.threshold * r_max: 
                    keypoints.append((x, y))

        return keypoints

    def run(self) -> None:
            """ Run the Harris Corner Detection pipeline. """
            i_x2, i_y2, i_xy = self.compute_gradients()
            s_x2, s_y2, s_xy = self.compute_structure_matrices(i_x2, i_y2, i_xy)
            r = self.compute_harris_response(s_x2, s_y2, s_xy)
            keypoints = self.detect_keypoints(r)
            self.visualize(keypoints)