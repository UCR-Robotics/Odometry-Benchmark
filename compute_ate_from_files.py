import numpy as np
import argparse
from tools_ate import compute_ate

def read_poses_from_file(file_path):
    poses = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            line = line.replace(',', ' ')
            if line and not line.startswith('#'):
                pose = [float(value) for value in line.split()]
                poses.append(pose)
    if poses and len(poses[0]) != 8:
        raise ValueError(f"Unknown file format. Each line has {len(poses[0])} values. \
            Expected 8 values per line in the format 'timestamp tx ty tz qx qy qz qw'.")
    return poses

def main():
    parser = argparse.ArgumentParser(description='Read poses from a file.')
    parser.add_argument('est_file', type=str, help='Path to the estimated trajectory.')
    parser.add_argument('gt_file', type=str, help='Path to the ground truth trajectory.')
    args = parser.parse_args()

    est_poses = read_poses_from_file(args.est_file)
    gt_poses = read_poses_from_file(args.gt_file)
    rot, trans, trans_error = compute_ate(est_poses, gt_poses, verbose=True)
    print(f"trans_error = {np.mean(trans_error):.3f}")

if __name__ == "__main__":
    main()
