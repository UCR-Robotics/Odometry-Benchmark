import argparse
import subprocess
import yaml
import csv
import time
import psutil
import os
import glob
import rosbag
from datetime import datetime, timedelta

def load_yaml_file(file_path):
  with open(file_path, 'r') as file:
    return yaml.safe_load(file)

def save_yaml_file(data, file_path):
  with open(file_path, 'w') as file:
    yaml.safe_dump(data, file, default_flow_style=False)

def save_csv_file(csv_header, data, file_path):
  with open(file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(csv_header)
    writer.writerows(data)

class AutoRunner:
  def __init__(self):
    self.waiting_time = 3
    self.results_folder = ""
    self.datasets_config = {}
    self.algorithms_config = {}
    self.dataset_cache = {}
    self.runner_cache = {}
    self.processes = []
    self.roscore_process = None

  def create_process(self, args, stdout_log=None, stderr_log=None):
    if stdout_log and stderr_log:
      with open(stdout_log, 'w') as f:
        with open(stderr_log, 'w') as e:
          return subprocess.Popen(args, shell=True, stdout=f, stderr=e)
    elif stdout_log:
      with open(stdout_log, 'w') as f:
        return subprocess.Popen(args, shell=True, stdout=f, stderr=subprocess.DEVNULL)
    else:
      return subprocess.Popen(args, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

  def terminate_process(self, process):
    if process is None or not psutil.pid_exists(process.pid):
      return
    parent = psutil.Process(process.pid)
    children = parent.children(recursive=True)
    for child in children:
      child.terminate()
    parent.terminate()

  def cleanup_sequence(self):
    for process in self.processes:
      self.terminate_process(process)
    self.processes = []
    time.sleep(self.waiting_time)

  def cleanup_all(self):
    for process in self.processes:
      self.terminate_process(process)
    self.terminate_process(self.roscore_process)
    time.sleep(3)

  def get_sequence_duration(self, dataset, sequence):
    # Get the rosbag files for this sequence
    sequence_path = os.path.join(dataset['data_folder'], sequence)
    if os.path.isfile(f"{sequence_path}.bag"):
      rosbag_files = [f"{sequence_path}.bag"]
    elif os.path.isdir(sequence_path):
      rosbag_files = glob.glob(os.path.join(sequence_path, '*.bag'))
    else:
      print(f"No rosbag file or directory found for sequence {sequence_path}.")

    # Calculate the duration for all rosbags in the sequence
    sequence_duration = 0
    for rosbag_file in rosbag_files:
      try:
        with rosbag.Bag(rosbag_file, 'r') as bag:
          duration = bag.get_end_time() - bag.get_start_time()
        sequence_duration += duration
      except Exception as e:
        print(f"Could not read rosbag file '{rosbag_file}': {e}")
    return sequence_duration

  def estimate_total_time(self, datasets_config, algorithms_config):
    total_time = 0
    for dataset_name, dataset in datasets_config.items():
      for sequence in dataset['sequences']:
        # Get the total duration for this sequence
        sequence_key = f"{dataset_name}_{sequence}"
        if sequence_key in self.dataset_cache:
          sequence_duration = self.dataset_cache[sequence_key]
        else:
          sequence_duration = self.get_sequence_duration(dataset, sequence)

        # Update the cache with the total duration for this sequence
        self.dataset_cache[sequence_key] = sequence_duration
        total_time += sequence_duration

    return total_time * len(algorithms_config)

  def run_sequence(self, algorithm_name, algorithm, dataset_name, dataset, sequence):
    sequence_folder = f"{self.results_folder}/{algorithm_name}/{dataset_name}/{sequence}"
    os.makedirs(sequence_folder, exist_ok=True)

    # Process 1: run algorithm
    for i, command in enumerate(algorithm['launch_commands'], start=1):
      stdout_log = f"{sequence_folder}/stdout_{i}.txt"
      stderr_log = f"{sequence_folder}/stderr_{i}.txt"
      process = self.create_process(command, stdout_log=stdout_log, stderr_log=stderr_log)
      self.processes.append(process)

    # Process 2: play rosbag (dataset sequences)
    if os.path.exists(f"{dataset['data_folder']}/{sequence}.bag"):
      rosbag_path = f"{dataset['data_folder']}/{sequence}.bag"
    else:
      rosbag_path = f"{dataset['data_folder']}/{sequence}/*.bag"
    remap_lidar_topic = f"{dataset['lidar_topic']}:={algorithm['input_topics']['lidar_topic']}"
    remap_imu_topic = f"{dataset['imu_topic']}:={algorithm['input_topics']['imu_topic']}"
    more_args = f"-d {self.waiting_time} {' '.join(algorithm.get('rosbag_args') or [])}"
    rosbag_play_cmd = " ".join(["rosbag play", rosbag_path, remap_lidar_topic, remap_imu_topic, more_args])
    rosbag_play_process = self.create_process(rosbag_play_cmd)
    self.processes.append(rosbag_play_process)

    # Process 3: record rosbag (as the results from algorithms)
    topics_to_record = ' '.join(algorithm.get('output_topics') or ['-a'])
    output_bag = f"{sequence_folder}/results.bag"
    rosbag_record_cmd = f"rosbag record {topics_to_record} -O {output_bag}"
    rosbag_record_process = self.create_process(rosbag_record_cmd)
    self.processes.append(rosbag_record_process)

    # Print the estimated running time, while waiting for the rosbag play to finish
    sequence_duration = self.dataset_cache[f"{dataset_name}_{sequence}"]
    begin_time = time.time()
    cpu_usage = []
    while rosbag_play_process.poll() is None:
      elapsed_time = time.time() - begin_time
      print(f"Estimated running time: {elapsed_time:.2f} / {sequence_duration:.2f} seconds.", end = '\r', flush=True)
      cpu_usage.append([elapsed_time, psutil.cpu_percent(interval=None)])
      time.sleep(0.1)

    # Update runner cache (to mark this sequence completed) and save cpu usage
    self.runner_cache[f"{algorithm_name}_{dataset_name}_{sequence}"] = True
    save_yaml_file(self.runner_cache, f"{self.results_folder}/.runner_cache.yaml")
    save_csv_file(['Time Elapsed', 'CPU Usage (%)'], cpu_usage, f"{sequence_folder}/cpu_usage.csv")

    # run cleanup commands if any
    if 'cleanup_commands' in algorithm:
      for command in algorithm['cleanup_commands']:
        os.environ['ALG_FOLDER'] = f"{self.results_folder}/{algorithm_name}"
        os.environ['SEQ_FOLDER'] = sequence_folder
        subprocess.run(command, shell=True, executable='/bin/bash', env=os.environ.copy())

  def run(self):
    for algorithm_name, algorithm in self.algorithms_config.items():
      for dataset_name, dataset in self.datasets_config.items():
        for sequence in dataset['sequences']:
          if f"{algorithm_name}_{dataset_name}_{sequence}" in self.runner_cache:
            print(f"Skipping algorithm {algorithm_name}, dataset {dataset_name}, sequence {sequence}.")
            continue
          print(f"Running algorithm {algorithm_name}, dataset {dataset_name}, sequence {sequence}.")
          self.run_sequence(algorithm_name, algorithm, dataset_name, dataset, sequence)
          self.cleanup_sequence()

  def init(self, results_folder, datasets_config_path, algorithms_config_path):
    # Determine if we continue from a previous run, and create the results folder accordingly
    if results_folder is not None and os.path.exists(results_folder):
      self.results_folder = results_folder
      if os.path.exists(f"{results_folder}/.runner_cache.yaml"):
        self.runner_cache = load_yaml_file(f"{results_folder}/.runner_cache.yaml")
        print("Found existing runner cache, skipping completed sequences.")
    else:
      self.results_folder = "results_" + datetime.now().strftime("%m%d%H%M")
      os.makedirs(self.results_folder, exist_ok=True)

    # Load the datasets configuration file
    if os.path.exists(f"{self.results_folder}/datasets.yaml"):
      self.datasets_config = load_yaml_file(f"{self.results_folder}/datasets.yaml")
    else:
      self.datasets_config = load_yaml_file(datasets_config_path)
      save_yaml_file(self.datasets_config, f"{self.results_folder}/datasets.yaml")

    # Load the algorithms configuration file
    if os.path.exists(f"{self.results_folder}/algorithms.yaml"):
      self.algorithms_config = load_yaml_file(f"{self.results_folder}/algorithms.yaml")
    else:
      self.algorithms_config = load_yaml_file(algorithms_config_path)
      save_yaml_file(self.algorithms_config, f"{self.results_folder}/algorithms.yaml")

    # Load the dataset cache and estimate the total running time
    if os.path.exists('.dataset_cache.yaml'):
      self.dataset_cache = load_yaml_file('.dataset_cache.yaml')
    print(f"Estimating total running time...")
    total_time = self.estimate_total_time(self.datasets_config, self.algorithms_config)
    save_yaml_file(self.dataset_cache, '.dataset_cache.yaml')
    print(f"Estimated total running time: {total_time:.2f} seconds or {total_time/3600.0:.2f} hours.")
    print(f"Estimated completion time: {(datetime.now() + timedelta(seconds=total_time)).strftime('%Y-%m-%d %H:%M:%S')}")

    # Start roscore
    self.roscore_process = self.create_process("roscore")
    time.sleep(3)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Load YAML configuration for datasets and algorithms.')
  parser.add_argument('results_folder', type=str, nargs='?', help='Path to the results folder.')
  parser.add_argument('--datasets', type=str, default='datasets.yaml', help='Path to the datasets configuration file.')
  parser.add_argument('--algorithms', type=str, default='algorithms.yaml', help='Path to the algorithms configuration file.')
  args = parser.parse_args()

  auto_runner = AutoRunner()
  auto_runner.waiting_time = 10
  try:
    auto_runner.init(args.results_folder, args.datasets, args.algorithms)
    auto_runner.run()
    print("Finished running all algorithms on all datasets.")
  except KeyboardInterrupt:
    print("KeyboardInterrupt: Shutting down...")
  finally:
    auto_runner.cleanup_all()
