#!/usr/bin/env python3
"""
stereo_feature_tracker.py

Extract robust feature tracks from a ROS1 rosbag of stereo image streams using DISK + LightGlue.
This module performs:
  1. Stereo matching on each vertex (a stereo pair) with RANSAC filtering.
  2. Temporal matching across adjacent vertices (left images) to propagate feature track IDs.
  3. Visualization of tracked features between consecutive stereo pairs.
  4. Output CSV files in a Maplab‐like format:
       • landmarks.csv:  landmark_id, x, y, z (unknown positions set to sentinel values)
       • tracks.csv:     time, vertex_id, frame_id (0 for left, 1 for right), keypoint_id, u, v, uncertainty, scale, track_id
       • descriptors.csv: Each row is the descriptor vector corresponding to the detection (rows in same order as tracks.csv)
       • observations.csv: vertex_id, frame_id, keypoint_id, landmark_id (vertex_id corresponds to a stereo pair)

Usage:
  python stereo_feature_tracker.py --bag path/to/rosbag.bag --camera_yaml cameras.yaml --output_dir output_csvs [--visualize]
"""

from lightglue import LightGlue, DISK
from lightglue.utils import load_image, rbd
from lightglue import viz2d
import torch

torch.set_grad_enabled(False)

import argparse
import os
import cv2
import numpy as np
import pandas as pd
import time

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
import matplotlib.pyplot as plt


def visualize_temporal_tracking(prev_vertex, curr_vertex, winname):
    """
    Visualize tracked features between two consecutive vertices using the images.
    The two images are concatenated horizontally and lines are drawn between keypoints sharing the same track id.
    """
    prev_img = prev_vertex.get("img", None)
    curr_img = curr_vertex.get("img", None)
    if prev_img is None or curr_img is None:
        print("Visualization: images not available in vertex data.")
        return

    prev_img_color = cv2.cvtColor(prev_img, cv2.COLOR_GRAY2BGR)
    curr_img_color = cv2.cvtColor(curr_img, cv2.COLOR_GRAY2BGR)
    concat_img = np.hstack((prev_img_color, curr_img_color))
    width_prev = prev_img.shape[1]

    # Build a dictionary mapping track id to keypoint location in the current vertex.
    curr_tracks = {}
    for i, kp in enumerate(curr_vertex["feats"]["keypoints"][0]):
        tid = curr_vertex["track_ids"][i]
        if tid != -1:
            curr_tracks[tid] = (int(kp[0]), int(kp[1]))

    for i, kp in enumerate(prev_vertex["feats"]["keypoints"][0]):
        tid = prev_vertex["track_ids"][i]
        if tid != -1 and tid in curr_tracks:
            pt_prev = (int(kp[0]), int(kp[1]))
            pt_curr = curr_tracks[tid]
            pt_curr_shifted = (pt_curr[0] + width_prev, pt_curr[1])
            cv2.circle(concat_img, pt_prev, 3, (0, 255, 0), -1)
            cv2.circle(concat_img, pt_curr_shifted, 3, (0, 255, 0), -1)
            cv2.line(concat_img, pt_prev, pt_curr_shifted, (0, 255, 0), 1)

    cv2.imshow(winname, concat_img)
    cv2.waitKey(5)
    # cv2.destroyAllWindows()


def load_camera_parameters(yaml_file):
    with open(yaml_file, "r") as f:
        params = yaml.safe_load(f)
    return params

def decode_compressed_image(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    return img

def ransac_filter_matches(kpts0, kpts1, matches, ransac_thresh=3.0, confidence=0.99):
    """
    Filter matches using the fundamental matrix via RANSAC.
    :param kpts0: NumPy array (N x 2) of keypoints from image0.
    :param kpts1: NumPy array (N x 2) of keypoints from image1.
    :param matches: List or array of tuples (i, j) indicating a match between keypoint i in kpts0 and j in kpts1.
    :returns: Filtered list of matches.
    """
    if len(matches) < 8:
        return matches
    pts0 = np.array([kpts0[i] for i, j in matches], dtype=np.float32)
    pts1 = np.array([kpts1[j] for i, j in matches], dtype=np.float32)
    F, mask = cv2.findFundamentalMat(pts0, pts1, cv2.FM_RANSAC, ransac_thresh, confidence)
    if mask is None:
        return matches
    mask = mask.ravel().astype(bool)
    filtered = [m for m, valid in zip(matches, mask) if valid]
    return filtered


def odd_int(value):
    ivalue = int(value)
    if ivalue % 2 == 0:
        raise argparse.ArgumentTypeError("%s is not an odd integer" % value)
    return ivalue


def main():
    parser = argparse.ArgumentParser(
        description="Extract robust stereo feature tracks with temporal tracking using DISK + LightGlue."
    )
    parser.add_argument("bag", help="Path to the ROS1 rosbag.", type=str)
    parser.add_argument("--left_topic", default="/zed2i/zed_node/left_raw/image_raw_gray/compressed", help="Left image topic.")
    parser.add_argument("--right_topic", default="/zed2i/zed_node/right_raw/image_raw_gray/compressed", help="Right image topic.")
    parser.add_argument("--camera_yaml", default="cameras.yaml", help="Path to camera parameters YAML file.")
    parser.add_argument(
        "--nms_window_size",
        type=odd_int,
        default=15,
        help="Odd window size for NMS (non-maximum suppression) to suppress repetitive feature points."
    )
    parser.add_argument("--output_dir", required=True, help="Directory to save output CSV files.")
    parser.add_argument("--max_frames", default=-1, type=int,
                        help="Maximum number of frames to process (-1 for no limit).")
    parser.add_argument("--visualize", action="store_true", help="Visualize temporal tracking between vertices.")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    # cam_params = load_camera_parameters(args.camera_yaml)
    # print("Loaded camera parameters from", args.camera_yaml)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    disk_model = DISK(max_num_keypoints=2048, nms_window_size=args.nms_window_size).eval().to(device)
    matcher = LightGlue(features="disk").eval().to(device)
    global_track_id = 0

    # Read the rosbag and collect left/right messages.
    left_msg_times = []
    right_msg_times = []

    typestore = get_typestore(Stores.ROS1_NOETIC)
    with Reader(args.bag) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == args.left_topic:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                left_msg_times.append(t)
            elif connection.topic == args.right_topic:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                right_msg_times.append(t)

    # Pair left and right messages using a tolerance (10 ms).
    tolerance = 0.01  # seconds
    paired_timestamps = []  # Each element: (left_timestamp, right_timestamp)
    used_right = set()
    for t_left in sorted(left_msg_times):
        best_t_right = None
        best_diff = float("inf")
        for t_right in right_msg_times:
            if t_right in used_right:
                continue
            diff = abs(t_left - t_right)
            if diff < best_diff and diff <= tolerance:
                best_diff = diff
                best_t_right = t_right
        if best_t_right is not None:
            used_right.add(best_t_right)
            paired_timestamps.append((t_left, best_t_right))
    print(f"Found {len(paired_timestamps)} stereo pairs (vertices) within {tolerance*1000:.0f} ms "
          f" tolerance out of {len(left_msg_times)} left and {len(right_msg_times)} right messages.")
    print("First 10 pairs of timestamps:", paired_timestamps[:10])
    print()

    # Global containers for CSV data.
    tracks_data = []       # For tracks.csv
    descriptors_data = []  # For descriptors.csv
    observations_data = [] # For observations.csv
    landmarks_set = set()
    vertices_data = []
    timed_vertices = []

    vertex_id = 0
    left_msgs = {}
    right_msgs = {}
    reader = Reader(args.bag)
    reader.open()
    frame_rate = 30.0
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == args.left_topic:
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            left_msgs[t] = msg
        elif connection.topic == args.right_topic:
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            right_msgs[t] = msg
        if vertex_id == len(paired_timestamps):
            break
        t_left = paired_timestamps[vertex_id][0]
        t_right = paired_timestamps[vertex_id][1]
        if t_left not in left_msgs or t_right not in right_msgs:
            continue
        if 0 < args.max_frames < vertex_id:
            break
        left_msg = left_msgs[t_left]
        right_msg = right_msgs[t_right]
        left_img_np = decode_compressed_image(left_msg)
        right_img_np = decode_compressed_image(right_msg)
        left_img_tensor = torch.tensor(left_img_np / 255.0, dtype=torch.float).unsqueeze(0).unsqueeze(0).to(device)
        right_img_tensor = torch.tensor(right_img_np / 255.0, dtype=torch.float).unsqueeze(0).unsqueeze(0).to(device)

        if hasattr(left_msg.header.stamp, "to_sec"):
            left_time = left_msg.header.stamp.to_sec()
            right_time = right_msg.header.stamp.to_sec()
        else:
            left_time = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
            right_time = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec * 1e-9


        tic = time.time()
        # 1. Extract features for left and right images.
        feats_left = disk_model.extract(left_img_tensor)
        feats_right = disk_model.extract(right_img_tensor)

        # 2. Perform stereo matching between current left and right images.
        result_lr = matcher({'image0': feats_left, 'image1': feats_right})
        result_lr = rbd(result_lr)
        matches_lr = [(i, int(j)) for i, j in enumerate(result_lr["matches0"]) if j != -1]
        
        matches_lr_filtered = ransac_filter_matches(
            np.vstack([kp.cpu().detach().numpy() for kp in feats_left["keypoints"][0]]).astype(np.float32),
            np.vstack([kp.cpu().detach().numpy() for kp in feats_right["keypoints"][0]]).astype(np.float32),
            matches_lr,
        )

        # 3. Temporal matching between consecutive vertices.
        if vertex_id > 0:
            prev_vertex = vertices_data[-1]
            # Temporal left-to-left matching.
            result_ll = matcher({"image0": prev_vertex['feats_left'], "image1": feats_left})
            result_ll = rbd(result_ll)
            matches_ll = [(i, int(j)) for i, j in enumerate(result_ll["matches0"]) if j != -1]
            matches_ll_filtered = ransac_filter_matches(
                np.vstack([kp.cpu().detach().numpy() for kp in prev_vertex["feats_left"]["keypoints"][0]]).astype(np.float32),
                np.vstack([kp.cpu().detach().numpy() for kp in feats_left["keypoints"][0]]).astype(np.float32),
                matches_ll,
            )
            # Temporal right-to-right matching.
            result_rr = matcher({"image0": prev_vertex['feats_right'], "image1": feats_right})
            result_rr = rbd(result_rr)
            matches_rr = [(i, int(j)) for i, j in enumerate(result_rr["matches0"]) if j != -1]
            matches_rr_filtered = ransac_filter_matches(
                np.vstack([kp.cpu().detach().numpy() for kp in prev_vertex["feats_right"]["keypoints"][0]]).astype(np.float32),
                np.vstack([kp.cpu().detach().numpy() for kp in feats_right["keypoints"][0]]).astype(np.float32),
                matches_rr,
            )
        else:
            matches_ll_filtered = []
            matches_rr_filtered = []
        toc = time.time()

        # 4. Update feature tracks.
        # Initialize current vertex track id lists.
        left_track_ids = [-1] * feats_left["keypoints"].shape[1]
        right_track_ids = [-1] * feats_right["keypoints"].shape[1]

        # First: propagate from previous left (matches_ll).
        if vertex_id > 0:
            prev_left_ids = prev_vertex["left_track_ids"]
            for i_prev, i_curr in matches_ll_filtered:
                if prev_left_ids[i_prev] != -1:
                    left_track_ids[i_curr] = prev_left_ids[i_prev]

        # Second: use stereo matching (matches_lr) in current vertex.
        for i, j in matches_lr_filtered:
            if left_track_ids[i] == -1 and right_track_ids[j] == -1:
                tid = global_track_id
                global_track_id += 1
                left_track_ids[i] = tid
                right_track_ids[j] = tid
            elif left_track_ids[i] != -1:
                tid = left_track_ids[i]
                right_track_ids[j] = tid
            elif right_track_ids[j] != -1:
                tid = right_track_ids[j]
                left_track_ids[i] = tid

        # Finally: propagate from previous right (matches_rr).
        if vertex_id > 0:
            prev_right_ids = prev_vertex["right_track_ids"]
            for i_prev, i_curr in matches_rr_filtered:
                if prev_right_ids[i_prev] != -1 and right_track_ids[i_curr] == -1:
                    right_track_ids[i_curr] = prev_right_ids[i_prev]
        toc2 = time.time()
        frame_rate = 0.7 * frame_rate + 0.3 / (toc2 - tic)
        if vertex_id % 100 == 50:
            print(f"Vertex {vertex_id}: {toc - tic:.3f}s, {toc2 - toc:.3f}s, frame rate: {frame_rate:.2f} fps")

        # 5. Visualization.
        show_vertex = 6
        if args.visualize and len(vertices_data) > show_vertex:
            visualize_temporal_tracking({
                "img": vertices_data[-show_vertex]["left_img"],
                "feats": vertices_data[-show_vertex]["feats_left"],
                "track_ids": vertices_data[-show_vertex]["left_track_ids"],
            }, {
                "img": left_img_np,
                "feats": feats_left,
                "track_ids": left_track_ids,
            }, 'left-left')

            visualize_temporal_tracking({
                "img": left_img_np,
                "feats": feats_left,
                "track_ids": left_track_ids,
            }, {
                "img": right_img_np,
                "feats": feats_right,
                "track_ids": right_track_ids,
            }, 'left-right')

            visualize_temporal_tracking({
                "img": vertices_data[-show_vertex]["right_img"],
                "feats": vertices_data[-show_vertex]["feats_right"],
                "track_ids": vertices_data[-show_vertex]["right_track_ids"],
            }, {
                "img": right_img_np,
                "feats": feats_right,
                "track_ids": right_track_ids,
            }, 'right-right')
        
        # 6. Save current vertex data.
        vertex_data = {
            "vertex_id": vertex_id,
            "feats_left": feats_left,
            "feats_right": feats_right,
            "left_track_ids": left_track_ids,
            "right_track_ids": right_track_ids,
            "left_img": left_img_np,
            "right_img": right_img_np,
        }
        # only keep at most 10 vertices in memory
        max_vertices = 10
        if len(vertices_data) > max_vertices:
            vertices_data.pop(0)
        if vertex_id == 0:
            vertices_data = [vertex_data]
        else:
            vertices_data.append(vertex_data)

        # 7. Update global CSV data.
        timed_vertices.append({
            "vertex_id": vertex_id,
            "time": left_time,
            "position_x": 0,
            "position_y": 0,
            "position_z": 0,
            "orientation_quaternion_x": 0,
            "orientation_quaternion_y": 0,
            "orientation_quaternion_z": 0,
            "orientation_quaternion_w": 1,
            "velocity_x": 0,
            "velocity_y": 0,
            "velocity_z": 0,
            "accelerometer_bias_x": 0,
            "accelerometer_bias_y": 0,
            "accelerometer_bias_z": 0,
            "gyroscope_bias_x": 0,
            "gyroscope_bias_y": 0,
            "gyroscope_bias_z": 0,
        })
        for i, kp in enumerate(feats_left["keypoints"][0]):
            tracks_data.append({
                "time": left_time,
                "vertex_id": vertex_id,
                "frame_id": 0,
                "keypoint_id": i,
                "u": kp[0].item(),
                "v": kp[1].item(),
                "uncertainty": 1.0,
                "scale": float(kp.size()[0]) if hasattr(kp, "size") and callable(kp.size) else 1.0,
                "track_id": left_track_ids[i],
            })

            descriptors_data.append(feats_left["descriptors"][0][i].tolist())
            if left_track_ids[i] != -1:
                observations_data.append({
                    "vertex_id": vertex_id,
                    "frame_id": 0,
                    "keypoint_id": i,
                    "landmark_id": left_track_ids[i],
                })
                landmarks_set.add(left_track_ids[i])
        for i, kp in enumerate(feats_right["keypoints"][0]):
            tracks_data.append({
                "time": right_time,
                "vertex_id": vertex_id,
                "frame_id": 1,
                "keypoint_id": i,
                "u": kp[0].item(),
                "v": kp[1].item(),
                "uncertainty": 1.0,
                "scale": float(kp.size()[0]) if hasattr(kp, "size") and callable(kp.size) else 1.0,
                "track_id": right_track_ids[i],
            })
            descriptors_data.append(feats_right["descriptors"][0][i].tolist())
            if right_track_ids[i] != -1:
                observations_data.append({
                    "vertex_id": vertex_id,
                    "frame_id": 1,
                    "keypoint_id": i,
                    "landmark_id": right_track_ids[i],
                })
                landmarks_set.add(right_track_ids[i])

        vertex_id += 1
        # remove those msgs earlier than t_left - 1 and t_right - 1
        for t in list(left_msgs.keys()):
            if t < t_left - 1:
                del left_msgs[t]
        for t in list(right_msgs.keys()):
            if t < t_right - 1:
                del right_msgs[t]
        if vertex_id % 200 == 0:
            # descriptors_csv = os.path.join(args.output_dir, f"descriptors_{vertex_id}.csv")
            # pd.DataFrame(descriptors_data).to_csv(descriptors_csv, index=False, header=False)
            # print(f"Saved descriptors to: {descriptors_csv}")
            descriptors_data = []

        torch.cuda.empty_cache()
    reader.close()

    # Build landmarks data.
    print(f'#vertices: {vertex_id}, #landmarks: {len(landmarks_set)}, global tracks: {global_track_id}')
    landmarks_data = [{"landmark_id": tid, "x": -1, "y": -1, "z": -1} for tid in sorted(landmarks_set)]

    # Save CSV files.
    vertices_csv = os.path.join(args.output_dir, "vertices.csv")
    tracks_csv = os.path.join(args.output_dir, "tracks.csv")
    observations_csv = os.path.join(args.output_dir, "observations.csv")
    landmarks_csv = os.path.join(args.output_dir, "landmarks.csv")

    df = pd.DataFrame(timed_vertices)
    df['time'] = df['time'].apply(lambda x: f"{x:.9f}")
    df.to_csv(vertices_csv, index=False)

    df = pd.DataFrame(tracks_data)
    df['time'] = df['time'].apply(lambda x: f"{x:.9f}")
    df.to_csv(tracks_csv, index=False)

    pd.DataFrame(observations_data).to_csv(observations_csv, index=False)
    pd.DataFrame(landmarks_data).to_csv(landmarks_csv, index=False)

    descriptors_csv = os.path.join(args.output_dir, f"descriptors_{vertex_id}.csv")
    pd.DataFrame(descriptors_data).to_csv(descriptors_csv, index=False, header=False)
    print(f"Saved final descriptors to: {descriptors_csv}")

    print(f"Saved vertices to: {vertices_csv}")
    print(f"Saved tracks to: {tracks_csv}")
    print(f"Saved observations to: {observations_csv}")
    print(f"Saved landmarks to: {landmarks_csv}")


if __name__ == '__main__':
    main()
