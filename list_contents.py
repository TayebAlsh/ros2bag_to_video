import rosbag
import os
import sys

def parse_rosbag(bag_file):
    if not os.path.exists(bag_file):
        print(f"File {bag_file} does not exist.")
        return

    bag = rosbag.Bag(bag_file)
    topic_info = {}

    for topic, msg, t in bag.read_messages():
        msg_size = len(msg.serialize())
        if topic not in topic_info:
            topic_info[topic] = {'type': msg._type, 'size': 0, 'count': 0}
        topic_info[topic]['size'] += msg_size
        topic_info[topic]['count'] += 1

    bag.close()

    print(f"Contents of {bag_file}:")
    for topic, info in topic_info.items():
        print(f"Topic: {topic}")
        print(f"  Type: {info['type']}")
        print(f"  Total Size: {info['size']} bytes")
        print(f"  Message Count: {info['count']}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python parse_rosbag.py <path_to_rosbag>")
        sys.exit(1)

    bag_file = sys.argv[1]
    parse_rosbag(bag_file)