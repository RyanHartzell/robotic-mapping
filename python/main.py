from feature_detectors.harris import HarrisCornerDetector

detector = HarrisCornerDetector("./images/cathedral.png", window_size=3, k=0.04, threshold=0.1)
detector.run()