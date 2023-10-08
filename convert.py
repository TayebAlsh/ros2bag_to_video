import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import subprocess
import os
import sys

class ImageExtractor(Node):
    def __init__(self, output_directory='./frames'):
        super().__init__('image_extractor')
        self.bridge = CvBridge()
        self.frame_num = 0
        self.output_directory = output_directory

        # Create the directory if it doesn't exist
        os.makedirs(self.output_directory, exist_ok=True)
        
        self.create_subscription(CompressedImage, '/cam1/camera/image_raw/compressed', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame_path = os.path.join(self.output_directory, f'frame_{self.frame_num:04d}.png')
        cv2.imwrite(frame_path, cv_image)
        self.frame_num += 1

def extract_frames_from_bag(bag_path):
    rclpy.init()
    node = ImageExtractor()
    
    # Play the bag using subprocess
    process = subprocess.Popen(['ros2', 'bag', 'play', bag_path])
    
    # Wait for the bag to finish playing
    while process.poll() is None:
        rclpy.spin_once(node)

    rclpy.shutdown()

def convert_frames_to_video(frame_dir, output_video_path):
    cmd = [
        'ffmpeg',
        '-framerate', '30',
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

    # The output file 
    output_video_path = "output_video.mp4"
    
    extract_frames_from_bag(bag_path)
    convert_frames_to_video('./frames', output_video_path)
