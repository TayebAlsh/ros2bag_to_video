import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import subprocess
import sys

class ImageExtractor(Node):
    def __init__(self, bag_path, video_output_cam1='output_cam1.mp4', video_output_cam2='output_cam2.mp4', fps=30):
        super().__init__('image_extractor')
        self.bridge = CvBridge()
        self.fps = fps

        # Construct output file paths based on the bag file path
        bag_dir = os.path.dirname(bag_path)
        self.video_output_cam1 = os.path.join(bag_dir, video_output_cam1)
        self.video_output_cam2 = os.path.join(bag_dir, video_output_cam2)

        self.out_cam1 = None
        self.out_cam2 = None

        self.create_subscription(CompressedImage, '/cam1/camera/image_raw/compressed', self.image_callback_cam1, 10)
        self.create_subscription(CompressedImage, '/cam2/camera/image_raw/compressed', self.image_callback_cam2, 10)

    def image_callback_cam1(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        if self.out_cam1 is None:
            height, width, _ = cv_image.shape
            self.out_cam1 = cv2.VideoWriter(self.video_output_cam1, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
        self.out_cam1.write(cv_image)

    def image_callback_cam2(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        if self.out_cam2 is None:
            height, width, _ = cv_image.shape
            self.out_cam2 = cv2.VideoWriter(self.video_output_cam2, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
        self.out_cam2.write(cv_image)

    def close_writers(self):
        if self.out_cam1 is not None:
            self.out_cam1.release()
        if self.out_cam2 is not None:
            self.out_cam2.release()

def extract_frames_from_bag(bag_path):
    rclpy.init()
    node = ImageExtractor(bag_path)
    
    # Play the bag using subprocess
    process = subprocess.Popen(['ros2', 'bag', 'play', bag_path])
    
    # Wait for the bag to finish playing
    while process.poll() is None:
        rclpy.spin_once(node)
    
    # Close video writers
    node.close_writers()

if __name__ == '__main__':
    if len(sys.argv) < 2:  # Check if a bag path was provided
        print("Usage: python3 convert.py /path/to/your_bag_file.db3")
        sys.exit(1)

    bag_path = sys.argv[1]

    extract_frames_from_bag(bag_path)