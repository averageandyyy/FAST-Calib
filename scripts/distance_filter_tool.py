#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 helper for estimating FAST-Calib distance filter bounds.

The tool reads a ROS2 rosbag2 directory, exports a sensor_msgs/msg/PointCloud2
topic to an ASCII PCD, then lets you pick at least four points around the board
in Open3D. It writes suggested x/y/z min/max crop values beside the PCD.
"""

import os
import sys

import numpy as np
import open3d as o3d
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"


def save_pcd_with_intensity(points, intensities, output_path):
    """Save points as ASCII PCD with x/y/z/intensity fields."""
    n_points = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {n_points}
HEIGHT 1
POINTS {n_points}
DATA ascii
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(header)
        for (x, y, z), intensity in zip(points, intensities):
            f.write(f"{x} {y} {z} {intensity}\n")
    print(f"[PCD] Saved point cloud with intensity to: {output_path}")


def open_reader(bag_uri):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def pointcloud2_topics(bag_uri):
    reader = open_reader(bag_uri)
    return [
        topic.name
        for topic in reader.get_all_topics_and_types()
        if topic.type == POINTCLOUD2_TYPE
    ]


def find_intensity_field(msg):
    """Detect a usable intensity field in PointCloud2 fields."""
    candidates = {"intensity", "reflectivity", "i", "ref"}
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def first_pointcloud2_message(bag_uri, topic_name):
    reader = open_reader(bag_uri)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == topic_name:
            return deserialize_message(data, PointCloud2)
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_uri,
    output_dir,
    topic_name,
    pcd_name="sensor_PointCloud2_inten_ascii.pcd",
):
    """Export one ROS2 PointCloud2 topic from a rosbag2 directory into PCD."""
    print(f"[Bag] Opening rosbag2: {bag_uri}")

    first_msg = first_pointcloud2_message(bag_uri, topic_name)
    if first_msg is None:
        print(f"[ERROR] No PointCloud2 messages found on topic '{topic_name}'", file=sys.stderr)
        return None

    intensity_field = find_intensity_field(first_msg)
    if intensity_field:
        print(f"[Bag] Detected intensity field: {intensity_field}")
        field_names = ["x", "y", "z", intensity_field]
    else:
        print("[Bag] No intensity field found; exporting intensity as 0.0")
        field_names = ["x", "y", "z"]

    all_points = []
    all_intensities = []

    reader = open_reader(bag_uri)
    print(f"[Bag] Reading PointCloud2 data from topic '{topic_name}'...")
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, PointCloud2)
        for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            all_points.append([point[0], point[1], point[2]])
            all_intensities.append(point[3] if intensity_field else 0.0)

    if not all_points:
        print("[ERROR] PointCloud2 topic contained no readable points.", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, np.asarray(all_intensities, dtype=np.float32), output_path)
    return output_path


def select_and_save_points(pcd_folder, target_pcd_name):
    """Pick points in Open3D and save x/y/z min/max crop values."""
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD file does not exist: {pcd_path}", file=sys.stderr)
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} contains no points.", file=sys.stderr)
        return

    print(f"\nProcessing: {target_pcd_name}")
    print("Hold Shift and left-click at least 4 board points, then press Q to close.")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Select points - {target_pcd_name}")
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    selected_indices = vis.get_picked_points()
    if not selected_indices:
        print(f"[ERROR] No points selected; no file saved for {target_pcd_name}", file=sys.stderr)
        return
    if len(selected_indices) < 4:
        print(f"[ERROR] Selected {len(selected_indices)} points, fewer than 4.", file=sys.stderr)
        return

    selected_indices = selected_indices[:4]
    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]

    mins = selected_points.min(axis=0)
    maxs = selected_points.max(axis=0)

    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, "w", encoding="utf-8") as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] Saved selected points and bounds to: {save_file}")
    print("Point cloud processing complete.")


def main():
    if len(sys.argv) > 1:
        bag_uri = sys.argv[1]
    else:
        bag_uri = os.getcwd()
        print(f"No bag path specified; using current directory: {bag_uri}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"No output directory specified; using current directory: {output_dir}")

    requested_topic = sys.argv[3] if len(sys.argv) > 3 else None

    if not os.path.exists(bag_uri):
        print(f"[ERROR] Bag path '{bag_uri}' does not exist.", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(output_dir):
        print(f"[ERROR] Output directory '{output_dir}' does not exist.", file=sys.stderr)
        sys.exit(1)

    topics = pointcloud2_topics(bag_uri)
    if not topics:
        print("[ERROR] No sensor_msgs/msg/PointCloud2 topics found in bag.", file=sys.stderr)
        sys.exit(1)

    topic_name = requested_topic or topics[0]
    if topic_name not in topics:
        print(f"[ERROR] Topic '{topic_name}' is not a PointCloud2 topic in this bag.", file=sys.stderr)
        print(f"Available PointCloud2 topics: {', '.join(topics)}", file=sys.stderr)
        sys.exit(1)

    print(f"[Detect] Using PointCloud2 topic: {topic_name}")
    pcd_path = convert_pointcloud2_bag_to_pcd(
        bag_uri=bag_uri,
        output_dir=output_dir,
        topic_name=topic_name,
    )
    if pcd_path is None:
        print("[ERROR] PCD generation failed.", file=sys.stderr)
        sys.exit(1)

    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path),
    )


if __name__ == "__main__":
    main()
