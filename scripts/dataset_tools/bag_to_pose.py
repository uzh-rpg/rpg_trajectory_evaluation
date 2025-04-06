#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

from rosbags.highlevel import AnyReader


def extract(bag_path, pose_topic, msg_type_str, out_filename):
    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == pose_topic]
        if not connections:
            raise RuntimeError(f"Topic {pose_topic} not found in bag.")

        with open(out_filename, 'w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
            n = 0

            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, conn.msgtype)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                if msg_type_str == "PoseWithCovarianceStamped":
                    p = msg.pose.pose.position
                    o = msg.pose.pose.orientation
                elif msg_type_str == "PoseStamped":
                    p = msg.pose.position
                    o = msg.pose.orientation
                elif msg_type_str == "TransformStamped":
                    p = msg.transform.translation
                    o = msg.transform.rotation
                elif msg_type_str == "Odometry":
                    p = msg.pose.pose.position
                    o = msg.pose.pose.orientation
                else:
                    raise RuntimeError("Unexpected message type at runtime")

                f.write(f'{stamp:.12f} {p.x:.12f} {p.y:.12f} {p.z:.12f} '
                        f'{o.x:.12f} {o.y:.12f} {o.z:.12f} {o.w:.12f}\n')
                n += 1

            print(f"Wrote {n} messages to {out_filename}")


def main():
    parser = argparse.ArgumentParser(description='Extract pose messages from a bag file (ROS 2 format, no ROS deps).')
    parser.add_argument('bag', help='Path to ROS 2 bag folder (.db3 or .mcap)')
    parser.add_argument('topic', help='Pose topic to extract')
    parser.add_argument('--msg_type', default='PoseStamped', help='Message type')
    parser.add_argument('--output', default='stamped_poses.txt', help='Output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print(f"Extracting from {args.bag}, topic: {args.topic}, type: {args.msg_type}")
    extract(args.bag, args.topic, args.msg_type, out_fn)


if __name__ == '__main__':
    main()
