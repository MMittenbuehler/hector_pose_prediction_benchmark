#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np
import os
import argparse
from pytransform3d import rotations as pr
import plotly.io as pio
pio.kaleido.scope.mathjax = None

def load_data(data_path):
    if os.path.isfile(data_path):
        data = np.genfromtxt(data_path, names=True, delimiter=",")
        return data
    else:
        return None

def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot pose prediction evaluation')
    parser.add_argument('data_file', metavar='FILE', type=str,
                        help='data file to be processed')

    args = parser.parse_args()
    return args

def plot_comparison(data, labels: list):
    fig = go.Figure()

    for label in labels:
        fig.add_trace(
            go.Scatter(
                x=data["time"],
                y=data[label],
                name=label
            )
        )

    if not os.path.exists("images"):
        os.mkdir("images")
    fig.write_image("images/" + "_".join(labels) + ".pdf")
    #fig.show()

def compute_label_metrics(data, label1, label2):
    diff = np.abs(data[label1] - data[label2])

    print(f"{label1} - {label2}")
    angle = "roll" in label1 or "pitch"
    compute_metrics(diff, angle)
    print("")

def compute_angular_error(data, label1_prefix, label2_prefix):
    pose_suffixes = ["orientation_roll", "orientation_pitch", "orientation_yaw"]
    label1 = [label1_prefix + "_" + suffix for suffix in pose_suffixes]
    label2 = [label2_prefix + "_" + suffix for suffix in pose_suffixes]

    angles = np.zeros(data.size)
    for i, row in enumerate(data[label1 + label2]):
        q1 = pr.quaternion_from_euler(row[label1], 0, 1, 2, True)
        q2 = pr.quaternion_from_euler(row[label2], 0, 1, 2, True)
        angle_axis = pr.quaternion_diff(q1, q2)
        angles[i] = angle_axis[3]

    print(f"{label1_prefix} - {label2_prefix}: rotation error")
    compute_metrics(angles, True)
    print("")

def compute_metrics(data, angle=False):
    mean = np.mean(data)
    std = np.std(data)
    maximum = np.max(np.abs(data))
    if angle:
        print(f"mean: {mean:0.4f} +/- {std}, deg: {np.rad2deg(mean):0.4f} +/- {np.rad2deg(std):0.4f}", )
        print(f"max: {maximum}, deg: {np.rad2deg(maximum)}")
    else:
        print(f"mean: {mean:0.4f} +/- {std:0.4f}")
        print(f"max: {maximum:0.4f}, deg: {np.rad2deg(maximum):0.4f}")


def main():
    args = parse_arguments()
    data = load_data(args.data_file)
    filename = os.path.basename(args.data_file)

    stability_comparison_labels = ["estimated_stability", "predicted_stability"]
    position_z_comparison_labels = ["ground_truth_position_z", "predicted_position_z"]
    roll_comparison_labels = ["ground_truth_orientation_roll", "predicted_orientation_roll"]
    pitch_comparison_labels = ["ground_truth_orientation_pitch", "predicted_orientation_pitch"]

    plot_comparison(data, stability_comparison_labels)
    plot_comparison(data, position_z_comparison_labels)
    plot_comparison(data, roll_comparison_labels + pitch_comparison_labels)

    compute_label_metrics(data, *stability_comparison_labels)
    compute_label_metrics(data, *position_z_comparison_labels)
    compute_label_metrics(data, *roll_comparison_labels)
    compute_label_metrics(data, *pitch_comparison_labels)

    compute_angular_error(data, "ground_truth", "predicted")


if __name__ == '__main__':
    main()


