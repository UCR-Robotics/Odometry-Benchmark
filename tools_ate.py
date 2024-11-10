import numpy as np

# https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate.py
def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn, numpy array)
    data -- second trajectory (3xn, numpy array)
    
    Output:
    rot -- rotation matrix (3x3, numpy array)
    trans -- translation vector (3x1, numpy array)
    trans_error -- translational error per point (1xn, numpy array)
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

def find_closest_index(L:list, t, beginning=0):
    """Find the index of the element in L whose timestamp is closest to t.
    
    Input:
    L -- a list of poses (nx8, Python list)
    t -- a timestamp (float)
    beginning -- the index to start searching from (int)

    8 elements in each row in L: [timestamp, x, y, z, qx, qy, qz, qw]
    sorting applies to the first element in each row, regardless of the rest
    
    Output:
    best -- the index of the element in L whose timestamp is closest to t (int)
    """
    if not beginning < len(L):
        return -1
    difference = abs(L[beginning][0] - t)
    best = beginning
    end = len(L)
    while beginning < end:
        middle = int((end + beginning) / 2)
        if abs(L[middle][0] - t) < difference:
            difference = abs(L[middle][0] - t)
            best = middle
        if t == L[middle][0]:
            return middle
        elif L[middle][0] > t:
            end = middle
        else:
            beginning = middle + 1
    return best

def associate(path:list, gt:list, verbose=False):
    """Associate the poses in the estimated path with the ground truth.
    
    Input:
    path -- the estimated trajectory (nx8, Python list of lists)
    gt -- the ground truth (mx8, Python list of lists)
    
    8 elements in each list: [timestamp, x, y, z, qx, qy, qz, qw]
    sorting applies to the first element in each row, regardless of the rest

    Output:
    path_associated -- kx8, k<=n, Python list of lists
    gt_associated -- kx8, k<=n, Python list of lists
    """
    path_associated = []
    gt_associated = []
    gt_idx = 0
    gt_interval = (gt[-1][0] - gt[0][0])/(2*len(gt))
    dropped = 0
    for pose in path:
        gt_idx = find_closest_index(gt, pose[0], gt_idx)
        if(gt_idx<0 or abs(gt[gt_idx][0] - pose[0])> gt_interval):
            dropped += 1
            continue
        path_associated.append(pose)
        gt_associated.append(gt[gt_idx])
        gt_idx += 1
    
    if(verbose):
        print("Dropped {} correspondences in the path association.".format(dropped))
    return path_associated, gt_associated

def compute_ate(path:list, gt:list, associate_path=True, verbose=False):
    """Compute the absolute trajectory error.
    
    Input:
    path -- the estimated trajectory (nx8, Python list of lists)
    gt -- the ground truth (mx8, Python list of lists)
    
    8 elements in each list: [timestamp, x, y, z, qx, qy, qz, qw]
    
    Output:
    rot -- rotation matrix (3x3, numpy array)
    trans -- translation vector (3x1, numpy array)
    trans_error -- translational error per point (1xn, numpy array)
    """
    if(associate_path):
        path_associated, gt_associated = associate(path, gt, verbose)
    print("path length: {}, gt length: {}".format(len(path), len(gt)))
    print("path_associated length: {}, gt_associated length: {}".format(len(path_associated), len(gt_associated)))
    model = np.array(path_associated)[:, 1:4].transpose()
    data = np.array(gt_associated)[:, 1:4].transpose()
    rot, trans, trans_error = align(model, data)
    return rot, trans, trans_error
