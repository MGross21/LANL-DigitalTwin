from robo_ag_rl.scripts.robot.connect_robot import RobotArm
from robot_env import RobotEnv
from robo_ag_rl.scripts.camera.serial_camera import SerialCamera
from robo_ag_rl.scripts.detection.april_tag_detection import AprilTagPoseEstimator

def main():
    # Initialize the robot arm with the address and optional connection parameters
    robot = RobotArm(address="192.168.1.100", connection_params={"port": 3000})

    try:
        robot.connect()
        # Example joint angles to send to the robot
        joint_angles = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        robot.set_joint_positions(joint_angles)

        # Get and print the current joint positions
        current_positions = robot.get_joint_positions()
        print("Current joint positions:", current_positions)

    except ConnectionError as e:
        print(f"Connection error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        robot.disconnect()

def main():
    # Initialize the environment
    env = RobotEnv()
    
    # Initialize the camera
    camera = SerialCamera()

    # Initialize the AprilTag detector
    apriltag_detector = AprilTagPoseEstimator()

    # Main loop for the robot environment
    while True:
        # Capture an image from the camera
        image = camera.capture_image()
        
        # Detect AprilTags in the image
        tag_data = apriltag_detector.detect_tags(image)

        # Process each detected tag
        for tag_id, position in tag_data.items():
            print(f"Detected tag {tag_id} at position {position}")

        # Here you would typically call your environment's update method
        # For example:
        # state, action = env.get_state_action()
        # reward = env.update(state, action)


if __name__ == "__main__":
    main()
