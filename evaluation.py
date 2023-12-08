import rosbag
import numpy as np
import csv
import os
import yaml
import argparse
import glob
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations
from tools_ate import compute_ate

"""
The intermediate format for trajectory used in this script is the 2D Python native list,
with each row in the format of [timestamp tx ty tz qx qy qz qw].
"""

def load_yaml_file(file_path):
  with open(file_path, 'r') as file:
    return yaml.safe_load(file)

def save_yaml_file(data, file_path):
  with open(file_path, 'w') as file:
    yaml.safe_dump(data, file, default_flow_style=False)

class OdometryBenchmark:
  def __init__(self, results_folder):
    self.results_folder = results_folder
    self.datasets = load_yaml_file(os.path.join(results_folder, 'datasets.yaml'))
    self.algorithms = load_yaml_file(os.path.join(results_folder, 'algorithms.yaml'))
    self.ate_results = {}
    self.rpe_results = {}
    for dataset_name, dataset_config in self.datasets.items():
      for sequence_name in dataset_config['sequences']:
        self.ate_results[(dataset_name, sequence_name)] = {}
        self.rpe_results[(dataset_name, sequence_name)] = {}

  def find_odometry_topic(self, bag_file):
    with rosbag.Bag(bag_file, 'r') as bag:
      for topic, type_info in bag.get_type_and_topic_info().topics.items():
        if type_info.msg_type == 'nav_msgs/Odometry':
          return topic
    return None

  def read_poses_from_rosbag(self, bag_file):
    poses = []
    topic = self.find_odometry_topic(bag_file)
    if topic:
      with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
          timestamp = msg.header.stamp.to_sec()
          t = msg.pose.pose.position
          q = msg.pose.pose.orientation
          poses.append([timestamp, t.x, t.y, t.z, q.x, q.y, q.z, q.w])
    else:
      raise ValueError(f"No topic with type nav_msgs/Odometry found in the bag file {bag_file}")
    return poses

  def read_poses_from_file(self, file_path):
    poses = []
    with open(file_path, 'r') as file:
      for line in file:
        line = line.strip()
        if line and not line.startswith('#'):
          pose = [float(value) for value in line.split()]
          poses.append(pose)
    if poses and len(poses[0]) != 8:
      raise ValueError(f"Unknown file format. Each line has {len(poses[0])} values. \
          Expected 8 values per line in the format 'timestamp tx ty tz qx qy qz qw'.")
    return poses

  def save_results_to_csv(self, file_name, results, algorithms):
    with open(file_name, 'w', newline='') as csvfile:
      writer = csv.writer(csvfile)
      header = ['dataset', 'sequence'] + list(algorithms.keys())
      writer.writerow(header)
      for (dataset, sequence), algos in results.items():
        row = [dataset, sequence] + [f"{algos.get(algo, ''):.3f}" for algo in algorithms.keys()]
        writer.writerow(row)

  def get_ground_truth_poses(self, dataset_name, sequence_name):
    dataset_config = self.datasets[dataset_name]
    gt_folder = dataset_config['ground_truth_folder']
    gt_type = dataset_config['ground_truth_type']
    for seq in dataset_config['sequences']:
      if seq == sequence_name:
        # try to match files in the ground truth folder
        gt_file = glob.glob(f"{gt_folder}/{sequence_name}/{'*.'}{gt_type}")
        if len(gt_file) == 0:
          raise ValueError(f"Ground truth file not found: {os.path.join(gt_folder, sequence_name)}.")
        print(f"Found ground truth file {gt_file[0]}")
        if gt_type == 'bag':
          return self.read_poses_from_rosbag(gt_file[0])
        elif gt_type == 'txt' or gt_type == 'csv':
          return self.read_poses_from_file(gt_file[0])
    raise ValueError(f"Ground truth file not found: folder {gt_folder}.")

  def process(self):
    for algorithm_name in self.algorithms.keys():
      for dataset_name, dataset_config in self.datasets.items():
        for sequence_name in dataset_config['sequences']:
          sequence_path = os.path.join(self.results_folder, algorithm_name, dataset_name, sequence_name)
          result_bag_file = os.path.join(sequence_path, 'results.bag')
          est_poses = self.read_poses_from_rosbag(result_bag_file)
          gt_poses = self.get_ground_truth_poses(dataset_name, sequence_name)

          # Compute ATE and save results
          rot, trans, trans_error = compute_ate(est_poses, gt_poses)
          self.ate_results[(dataset_name, sequence_name)][algorithm_name] = np.mean(trans_error)
          print(f'Processed algorithm {algorithm_name}, dataset {dataset_name}, sequence {sequence_name}')

    # Save all results to a CSV file
    csv_file_path = os.path.join(self.results_folder, 'ate_results.csv')
    self.save_results_to_csv(csv_file_path, self.ate_results, self.algorithms)
    print(f"Evaluation completed and results saved to {csv_file_path}")

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Evaluate odometry results.')
  parser.add_argument('results_folder', type=str, help='Path to the results folder')
  args = parser.parse_args()

  benchmark = OdometryBenchmark(args.results_folder)
  benchmark.process()
