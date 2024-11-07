import cv2

class SerialCamera:
    def __init__(self, serial_port=0):
        """
        Initialize the camera.
        :param serial_port: The port for the camera (default is 0 for the first camera).
        """
        self.cap = cv2.VideoCapture(serial_port)
        if not self.cap.isOpened():
            raise ValueError(f"Unable to open camera at port {serial_port}")

    def read_frame(self):
        """
        Capture a frame from the camera.
        :return: The captured frame or None if capture fails.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def release(self):
        """
        Release the camera resource.
        """
        self.cap.release()