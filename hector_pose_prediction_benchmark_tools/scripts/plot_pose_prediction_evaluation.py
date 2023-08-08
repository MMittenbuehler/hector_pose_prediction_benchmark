#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np
import os
import argparse

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

def compute_metrics(data, label1, label2):
    diff = np.abs(data[label1] - data[label2])
    mean = np.mean(diff)
    print(f"{label1} - {label2}")
    if "roll" in label1 or "pitch" in label1:
        print("mean: ", mean, "deg:", np.rad2deg(mean))
    else:
        print("mean:", mean)
    print("")


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

    compute_metrics(data, *stability_comparison_labels)
    compute_metrics(data, *position_z_comparison_labels)
    compute_metrics(data, *roll_comparison_labels)
    compute_metrics(data, *pitch_comparison_labels)


if __name__ == '__main__':
    main()


