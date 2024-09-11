import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import subprocess
import sys

class ImageExtractor(Node):
    def __init__(self, output_dir):
        super().__init__('image_extractor')
        self.bridge = CvBridge()
        self.cam1_video_writer = None
        self.cam2_video_writer = None
        self.cam1_frame_num = 0
        self.cam2_frame_num = 0
        self.fps = 30  # Set the frame rate to 30 fps
        self.output_dir = output_dir

        self.create_subscription(CompressedImage, '/cam1/camera/image_raw/compressed', self.cam1_image_callback, 10)
        self.create_subscription(CompressedImage, '/cam2/camera/image_raw/compressed', self.cam2_image_callback, 10)

    def initialize_video_writer(self, filename, frame_size):
        return cv2.VideoWriter(
            filename, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, frame_size
        )

    def cam1_image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        if self.cam1_video_writer is None:
            frame_size = (cv_image.shape[1], cv_image.shape[0])
            output_path = os.path.join(self.output_dir, 'cam1_output.mp4')
            self.cam1_video_writer = self.initialize_video_writer(output_path, frame_size)
        self.cam1_video_writer.write(cv_image)
        self.cam1_frame_num += 1

    def cam2_image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        if self.cam2_video_writer is None:
            frame_size = (cv_image.shape[1], cv_image.shape[0])
            output_path = os.path.join(self.output_dir, 'cam2_output.mp4')
            self.cam2_video_writer = self.initialize_video_writer(output_path, frame_size)
        self.cam2_video_writer.write(cv_image)
        self.cam2_frame_num += 1

    def __del__(self):
        if self.cam1_video_writer:
            self.cam1_video_writer.release()
        if self.cam2_video_writer:
            self.cam2_video_writer.release()

def extract_frames_from_bag(bag_path):
    rclpy.init()
    output_dir = os.path.dirname(bag_path)
    node = ImageExtractor(output_dir)
    
    # Play the bag using subprocess with increased read-ahead-queue-size
    process = subprocess.Popen(['ros2', 'bag', 'play', bag_path, '--read-ahead-queue-size', '5000'])
    
    # Wait for the bag to finish playing
    while process.poll() is None:
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 2:  # Check if a bag path was provided
        print("Usage: python3 convert_bag.py /path/to/your_bag_file.db3")
        sys.exit(1)

    bag_path = sys.argv[1]

    # Extract frames from the bag file and save directly to video
    extract_frames_from_bag(bag_path)