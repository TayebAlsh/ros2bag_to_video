# ROS2 Bag Video Extractor
This package provides a simple Python script to extract video frames from a ROS2 bag file (*.db3) and then compiles those frames into a video format using ffmpeg.

## Prerequiste
Before using the video extractor, ensure you have the following software installed:
 - ROS2 (Foxy, Galactic, or any later distribution should work). Tested with Humble.
 - Python 3.6 or later
 - OpenCV (used for image operations)
 - **ffmpeg** (used for video compilation)

## Installation
1. Install ROS2: Follow the instructions for your operating system on the official ROS2 website (Humble in this case)
2. Install Python dependencies:
```bash
sudo apt install python3-opencv
```
4. Install ffmpeg:
```bash
sudo apt update
sudo apt install ffmpeg
```
## Usage
Clone the repo, and run the script
```bash
python3 convert.py /path/to/your_bag_file.db3
```

Once executed, the script will:
- Extract individual frames from the specified bag file and save them to a ./frames directory.
- Compile those frames into a video named output_video.mp4.
