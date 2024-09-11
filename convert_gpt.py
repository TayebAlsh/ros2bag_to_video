import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import subprocess
import sys

class ImageExtractor(Node):
    def __init__(self):
        super().__init__('image_extractor')
        self.bridge = CvBridge()
        self.cam1_frame_num = 0
        self.cam2_frame_num = 0

        fps = 60  # Set the frame rate to 60 fps

        self.cam1_video_writer = cv2.VideoWriter(
            'cam1_output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (640, 480)
        )
        self.cam2_video_writer = cv2.VideoWriter(
            'cam2_output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (640, 480)
        )

        self.create_subscription(CompressedImage, '/cam1/camera/image_raw/compressed', self.cam1_image_callback, 10)
        self.create_subscription(CompressedImage, '/cam2/camera/image_raw/compressed', self.cam2_image_callback, 10)

    def cam1_image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.cam1_video_writer.write(cv_image)
        self.cam1_frame_num += 1

    def cam2_image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.cam2_video_writer.write(cv_image)
        self.cam2_frame_num += 1

    def __del__(self):
        self.cam1_video_writer.release()
        self.cam2_video_writer.release()

def extract_frames_from_bag(bag_path):
    rclpy.init()
    node = ImageExtractor()
    
    # Play the bag using subprocess with increased read-ahead-queue-size
    process = subprocess.Popen(['ros2', 'bag', 'play', bag_path, '--read-ahead-queue-size', '1000'])
    
    # Wait for the bag to finish playing
    while process.poll() is None:
        rclpy.spin_once(node)

    rclpy.shutdown()

def convert_frames_to_video(frame_dir, output_video_path):
    cmd = [
        'ffmpeg',
        '-framerate', '60',  # Set the frame rate to 60 fps
        '-i', os.path.join(frame_dir, 'frame_%04d.png'),
        '-c:v', 'libx264',
        '-profile:v', 'high',
        '-crf', '20',
        '-pix_fmt', 'yuv420p',
        output_video_path
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Error executing ffmpeg:")
        print(result.stderr)

if __name__ == '__main__':
    if len(sys.argv) < 2:  # Check if a bag path was provided
        print("Usage: python3 convert.py /path/to/your_bag_file.db3")
        sys.exit(1)

    bag_path = sys.argv[1]

    # Extract frames from the bag file
    extract_frames_from_bag(bag_path)

    # Convert frames to video for both cameras
    convert_frames_to_video('./cam1_frames', 'cam1_video.mp4')
    convert_frames_to_video('./cam2_frames', 'cam2_video.mp4')