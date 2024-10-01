from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations
import numpy as np

# https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate.py
def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """
    np.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1,keepdims=True)
    data_zerocentered = data - data.mean(1,keepdims=True)
    
    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = np.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1,keepdims=True) - rot * model.mean(1,keepdims=True)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def path2numpy(path:Path):
    ret = np.zeros((3,len(path.poses)))
    for i in range(len(path.poses)):
        ret[:,i] = np.array([path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z])

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

def ate(path:Path, gt:Path, associate_path=True, verbose = False):
    if(associate_path):
        path, gt = associate(path, gt, verbose)
    model = path2numpy(path)
    data = path2numpy(gt)
    rot, trans, trans_error = align(model, data)
    return rot, trans, trans_error
    