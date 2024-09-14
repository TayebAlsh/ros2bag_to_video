import rosbag2_py
import argparse
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from atl_msgs.msg import Depth  # Assuming the /depth topic is of type atl_msgs/Depth
import time

def read_depth_and_plot(bag_path, topic_name='/depth'):
    # Initialize the rosbag2 reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    # Open the bag file
    reader.open(storage_options, converter_options)

    # Get all available topics and their types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Check if the topic exists in the bag
    if topic_name not in type_map:
        print(f"Topic {topic_name} not found in the bag.")
        return

    print(f"Reading messages from {topic_name}...")

    timestamps = []
    depth_values = []

    start_time = None

    # Iterate through the messages in the bag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == topic_name:
            # Deserialize the message using the correct type
            msg = deserialize_message(data, Depth)

            # Set the start time
            if start_time is None:
                start_time = timestamp

            # Convert the timestamp to seconds relative to the start of the bag
            time_in_seconds = (timestamp - start_time) / 1e9  # Convert nanoseconds to seconds

            # Append the time and depth values to lists
            timestamps.append(time_in_seconds)
            depth_values.append(msg.depth)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, depth_values, label='Depth', color='b')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Depth')
    plt.title(f'Depth vs Time for {topic_name}')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Read and plot depth data from a ROS 2 bag file.")
    parser.add_argument('bag_file', type=str, help="Path to the ROS 2 bag file")
    parser.add_argument('--topic', type=str, default='/depth', help="Name of the topic to read from")
    args = parser.parse_args()

    # Call the function to read depth data and plot it
    read_depth_and_plot(args.bag_file, args.topic)
