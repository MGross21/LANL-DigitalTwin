import cv2
import apriltag
import numpy as np
from scipy.spatial.transform import Rotation as R

class AprilTagPoseEstimator:
    def __init__(self, camera_matrix, dist_coeffs, tag_size=0.05):
        """
        Initialize the AprilTag pose estimator.
        :param camera_matrix: The intrinsic camera matrix (3x3).
        :param dist_coeffs: The distortion coefficients (1x5 or 1x8).
        :param tag_size: The size of the side of the AprilTag in meters.
        """
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size

    def estimate_pose(self, frame):
        """
        Detect AprilTags in the frame and estimate their pose.
        :param frame: The input frame from the camera.
        :return: A list of dictionaries with tag ID and pose (x, y, z, roll, pitch, yaw).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        results = []

        for detection in detections:
            # Define 3D points of the AprilTag corners in the tag's coordinate frame
            object_points = np.array([
                [-self.tag_size / 2, -self.tag_size / 2, 0],
                [ self.tag_size / 2, -self.tag_size / 2, 0],
                [ self.tag_size / 2,  self.tag_size / 2, 0],
                [-self.tag_size / 2,  self.tag_size / 2, 0]
            ], dtype=np.float32)

            # 2D points from the detected corners
            image_points = detection.corners.reshape(4, 2)

            # SolvePnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coeffs
            )

            if success:
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                rotation = R.from_matrix(rotation_matrix)

                # Extract Euler angles from rotation matrix (roll, pitch, yaw)
                roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

                # Append the result
                results.append({
                    'id': detection.tag_id,
                    'position': {
                        'x': tvec[0][0],
                        'y': tvec[1][0],
                        'z': tvec[2][0]
                    },
                    'orientation': {
                        'roll': roll,
                        'pitch': pitch,
                        'yaw': yaw
                    }
                })

        return results

    def display_detections(self, frame, results):
        """
        Draw the detections and pose information on the frame.
        :param frame: The frame to display.
        :param results: The list of detected tags with pose information.
        """
        for result in results:
            tag_id = result['id']
            x, y, z = result['position'].values()
            roll, pitch, yaw = result['orientation'].values()
            cv2.putText(
                frame, f"ID: {tag_id}, Pos: ({x:.2f}, {y:.2f}, {z:.2f})",
                (10, 30 + 20 * tag_id), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            cv2.putText(
                frame, f"Orientation (r,p,y): ({roll:.2f}, {pitch:.2f}, {yaw:.2f})",
                (10, 50 + 20 * tag_id), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
        return frame

# Example usage
if __name__ == "__main__":
    # Example camera parameters (replace with actual calibration data)
    camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    # Initialize the estimator
    estimator = AprilTagPoseEstimator(camera_matrix, dist_coeffs)

    # Open camera capture (replace with video file path if necessary)
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Estimate pose
        results = estimator.estimate_pose(frame)
        
        # Display detections
        frame = estimator.display_detections(frame, results)
        cv2.imshow("AprilTag Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
