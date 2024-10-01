from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations
import numpy as np


# calculate translation and rotation difference of 2 poses
def pose_diff(p1,p2,mat=False):
    T1=tf.transformations.quaternion_matrix([p1.orientation.x,p1.orientation.y,p1.orientation.z,p1.orientation.w])
    T1[0:3,3]=[p1.position.x, p1.position.y, p1.position.z]

    T2=tf.transformations.quaternion_matrix([p2.orientation.x,p2.orientation.y,p2.orientation.z,p2.orientation.w])
    T2[0:3,3]=[p2.position.x, p2.position.y, p2.position.z]

    T_diff=tf.transformations.inverse_matrix(T1)@T2
    if(mat):
        return T_diff
    translation_diff=np.linalg.norm(T_diff[0:3,3])

    tmp=(np.trace(T_diff[0:3,0:3])-1)/2
    if(tmp>1):
        tmp=1
    if(tmp<-1):
        tmp=-1
    rotation_diff=np.abs(np.math.acos(tmp))*180/np.math.pi

    return translation_diff,rotation_diff

def pose_distance(p1:PoseStamped, p2:PoseStamped):
    return np.math.sqrt((p1.pose.position.x-p2.pose.position.x)**2+(p1.pose.position.y-p2.pose.position.y)**2+(p1.pose.position.z-p2.pose.position.z)**2)

def calculate_path_distance_meter(path:Path):
    ret=[0]
    for i in range(1,len(path.poses)):
        ret.append(pose_distance(path.poses[i],path.poses[i-1]))
    
    for i in range(1, len(ret)):
        ret[i]+=ret[i-1]
    return ret

def calculate_path_distance_seconds(path:Path):
    ret=[]
    for i in range(1,len(path.poses)):
        ret.append(path.header.stamp.to_sec())
    return ret

def find_closest_index(L, t, beginning=0):
    if(not beginning<len(L)):
        return -1
    difference = abs(L[beginning].header.stamp.to_sec() - t)
    best = beginning
    end = len(L)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(L[middle].header.stamp.to_sec() - t) < difference:
            difference = abs(L[middle].header.stamp.to_sec() - t)
            best = middle
        if t == L[middle].header.stamp.to_sec():
            return middle
        elif L[middle].header.stamp.to_sec() > t:
            end = middle
        else:
            beginning = middle + 1
    return best


def associate(path:Path, gt:Path, verbose = False):
    path_associated = Path()
    path_associated.header = path.header
    gt_associated = Path()
    gt_associated.header = gt.header
    gt_idx = 0
    gt_interval = (gt.poses[-1].header.stamp.to_sec() - gt.poses[0].header.stamp.to_sec())/(2*len(gt.poses))
    dropped = 0
    for pose in path.poses:
        gt_idx = find_closest_index(gt.poses,pose.header.stamp.to_sec(),gt_idx)
        if(gt_idx<0 or abs(gt.poses[gt_idx].header.stamp.to_sec() - pose.header.stamp.to_sec())> gt_interval):
            dropped += 1
            continue
        path_associated.poses.append(pose)
        gt_associated.poses.append(gt.poses[gt_idx])
        gt_idx += 1
    
    if(verbose):
        print("Dropped {} velodyne frames for no ground truth frame is able to assign to it.".format(dropped))
    return path_associated, gt_associated

# kitti style relative pose error
def rpe(path:Path, gt:Path, step_size:int=10, window_size:int=100, use_time_distance=False, verbose = False):
    idx=0
    path,gt = associate(path, gt, verbose)

    if(use_time_distance):
        gt_dist=calculate_path_distance_seconds(gt)
    else:
        gt_dist=calculate_path_distance_meter(gt)

    sum_dist_error=0
    sum_rotation_error=0
    samples=0

    while(idx+window_size<len(path.poses)):
        path_start = path.poses[idx]
        path_end = path.poses[idx+window_size]
        path_diff = pose_diff(path_start.pose, path_end.pose, True)
        
        gt_start = gt.poses[idx]
        gt_end = gt.poses[idx+window_size]
        gt_diff=pose_diff(gt_start.pose,gt_end.pose,True)

        traj_diff=tf.transformations.inverse_matrix(path_diff)@gt_diff
        gt_dist_over_window = (gt_dist[idx+window_size]-gt_dist[idx])
        if(gt_dist_over_window == 0):
            idx+=step_size
            continue

        translation_diff_over_window=np.linalg.norm(traj_diff[0:3,3])/gt_dist_over_window
        cos_theta=max(min((np.trace(traj_diff[0:3,0:3])-1)/2,1),-1)
        rotation_diff_over_window=(np.abs(np.math.acos(cos_theta))*180/np.math.pi)/gt_dist_over_window
        
        sum_dist_error+=translation_diff_over_window
        sum_rotation_error+=rotation_diff_over_window
        samples+=1

        idx += step_size

    return samples, sum_dist_error, sum_rotation_error
