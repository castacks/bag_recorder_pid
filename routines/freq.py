#!/usr/bin/env python3
import yaml
import argparse
from tabulate import tabulate
import csv
import os

def estimate_frequency(metadata_path, precision=2):
    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    bag_info = metadata['rosbag2_bagfile_information']
    res = {}
    for topic_info in bag_info['topics_with_message_count']:
        topic = topic_info['topic_metadata']['name']
        message_count = topic_info['message_count']
        msg_type = topic_info['topic_metadata']['type']

        # Duration comes from 'message_time' section
        tmin = bag_info['starting_time']['nanoseconds_since_epoch'] * 1e-9
        tmax = bag_info['duration']['nanoseconds'] * 1e-9 + tmin

        duration = tmax - tmin
        if duration > 0:
            rate = message_count / duration
        else:
            rate = 0.0

        res[topic] = (rate, message_count, msg_type)

    # Format and print output
    table_data = []
    for i, k in enumerate(sorted(res.keys())):
        rate, count, msg_type = res[k]
        row = [k, f"{rate:.{precision}f}Hz", count, msg_type]
        table_data.append(row)

    print(tabulate(table_data, headers=['Topic', 'Rate', 'Count', 'Type'], tablefmt='pipe'))
    return table_data

def bag_freq(output: str, precision: int=2):
    metadata_path = os.path.join(output, "metadata.yaml")
    rates = estimate_frequency(metadata_path, precision)
    return rates

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Estimate topic frequencies from ROS 2 bag metadata.yaml")
    parser.add_argument("--bag-dir", help="Path to bag, not metadata.yaml")
    parser.add_argument("--exp-dir", required=False, help="Path to experiment with multiple bags inside it")
    parser.add_argument("--precision", type=int, default=2, help="Decimal precision for rate output")
    parser.add_argument("--save", action="store_true", help="Save CSV of rate report")
    args = parser.parse_args()

    if args.bag_dir and args.exp_dir:
        print("Cannot provide bag and experiment directory.")
        exit(1)
    
    if args.bag_dir:
        bag_freq(args.bag_dir, args.precision)
    elif args.exp_dir:
        all_rates = []
        root_dir = args.exp_dir
        for bag_dir in os.listdir(root_dir):
            bag_dir = os.path.join(root_dir, bag_dir)
            rates = bag_freq(bag_dir, args.precision)
            all_rates.append(rates)
        if args.save:
            with open("rates.csv", "w") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Topic", "Rate", "Count", "Type"])
                for bag_rates in all_rates:
                    writer.writerows(bag_rates)

