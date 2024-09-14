import rosbag2_py
import argparse
from rclpy.serialization import deserialize_message
from atl_msgs.msg import Depth  # Adjusted to match the type of the /depth topic

def read_and_print_depth(bag_path, topic_name='/depth'):
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
    
    # Iterate through the messages in the bag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == topic_name:
            # Deserialize the message using the correct type
            msg = deserialize_message(data, Depth)
            print(f"Timestamp: {timestamp}")
            print(f"Depth data: {msg.depth}")  # Assuming msg.depth contains the depth value
            print("-----")

if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Read depth data from a ROS 2 bag file.")
    parser.add_argument('bag_file', type=str, help="Path to the ROS 2 bag file")
    parser.add_argument('--topic', type=str, default='/depth', help="Name of the topic to read from")
    args = parser.parse_args()

    # Call the function to read depth data
    read_and_print_depth(args.bag_file, args.topic)
