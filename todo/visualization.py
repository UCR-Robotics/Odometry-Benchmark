import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import yaml
import tf.transformations
import numpy as np
import readline

from ate import ate
from rpe import rpe
from utils import calculate_path_distance, path_statistics, transform_pose

RED     = "\033[1;31m"  
BLUE    = "\033[1;34m"
CYAN    = "\033[1;36m"
GREEN   = "\033[0;32m"
ORANGE  = "\033[93m"
RESET   = "\033[0;0m"
BOLD    = "\033[;1m"
REVERSE = "\033[;7m"

def display_commands():
    print("=====================================================================================")
    print("ls:                     list all datasets")
    print("ls DATASET:             list all sequences of the given dataset")
    print("set DATASET SEQUENCE:   set the dataset and sequence to be displayed")
    print("align:                  align current displayed paths using estimation from ate")
    print("information:            display the information of the displayed dataset and sequence")
    print("end:                    exit visualization mode")
    print()
    print()

def list(commands, gt):
    if(len(commands)==1):
        for dataset in gt.keys():
            print("{}{}{}:".format(BLUE, dataset, RESET))
            for sequence in gt[dataset].keys():
                print("  {}{}{}".format(CYAN, sequence, RESET))
    elif(len(commands)==2):
        if(commands[1] in gt):
            print("{}{}{}:".format(BLUE, commands[1], RESET))
            for sequence in gt[commands[1]].keys():
                print("  {}{}{}".format(CYAN, sequence, RESET))
        else:
            print("{}Invalid dataset {}{}{}!{}".format(RED, RESET, commands[1], RED, RESET))
    else:
        print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))

def set_current(commands, results, gt, publishers):
    if(len(commands)!=3):
        print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))
        return False
    dataset = commands[1]
    sequence = commands[2]
    if(not dataset in gt):
        print("{}Invalid dataset \"{}{}{}\"!{}".format(RED, RESET, dataset,RED, RESET))
        print("Valid datasets are:")
        list(["ls"], gt)
        return False
    if(not sequence in gt[dataset]):
        print("{}Invalid sequence \"{}{}{}\"!{}".format(RED, RESET, sequence,RED, RESET))
        print("Valid sequence in {}{}{} are:".format(GREEN, dataset, RESET))
        list(["ls", dataset],gt)
        return False
    
    for algorithm in results.keys():
        publishers[algorithm].publish(results[algorithm][dataset][sequence])
    
    publishers["gt"].publish(gt[dataset][sequence])
    print("{}{}{}.{}{}{} is the current dataset. Path published.".format(BLUE,dataset,RESET,CYAN,sequence,RESET))
    
    return True

def information(commands, results, gt):
    if(commands[0] == None or commands[1] == None):
        print("Please set a valid dataset and sequence to display information!")
        return
    
    dataset = commands[0]
    sequence = commands[1]
    distance = calculate_path_distance(gt[dataset][sequence])[-1]
    max_vel, avg_vel, max_ang, avg_ang, total_dist, total_time = path_statistics(gt[dataset][sequence])
    print("Current sequence: {}{}: {}{}{}".format(GREEN, dataset, BLUE, sequence, RESET))
    print("Total distance: {:.4f} meters".format(distance))
    print("  maximum linear velocity: {:.4f} m/s".format(max_vel))
    print("  average linear velocity: {:.4f} m/s".format(avg_vel))
    print("  maximum angular velocity: {:.4f} rad/s".format(max_ang))
    print("  average angular velocity: {:.4f} rad/s".format(avg_ang))
    print("  total time: {:.4f} s".format(total_time))

    for algorithm in results:
        path = results[algorithm][dataset][sequence]
        rot, trans, trans_error = ate(path, gt[dataset][sequence], True)
        samples, sum_dist_error, sum_rotation_error = rpe(path, gt[dataset][sequence], use_time_distance=False)
        print("{}{}{}, ate: {:.6f}, rpe: {:.6f}".format(GREEN, algorithm, RESET, np.mean(trans_error), sum_dist_error/samples))
    return

def align(commands, results, gt, publishers):
    if(commands[0] == None or commands[1] == None):
        print("Please set a valid dataset and sequence to align trajectories!")
        return
    
    dataset = commands[0]
    sequence = commands[1]

    for algorithm in results:
        rot, trans, trans_error = ate(results[algorithm][dataset][sequence], gt[dataset][sequence], True)
        T = np.eye(4)
        T[0:3,0:3] = rot
        T[0:3,3] = trans.ravel()
        path = Path()
        path.header = results[algorithm][dataset][sequence].header
        for pose in results[algorithm][dataset][sequence].poses:
            ps=PoseStamped()
            ps.header = pose.header
            ps.pose = transform_pose(T, pose.pose)
            path.poses.append(ps)
        publishers[algorithm].publish(path)
    print("Aligned trajectories published!")

def create_completor(gt):
    commands = ["ls", "set", "align", "information", "quit", "end"]
    results = []
    def completor(text:str, state:int):
        nonlocal results
        if(state!=0):
            return results[state]
        
        if(text.startswith("ls")):
            cmds = text.split()
            if(len(cmds)==1):
                results = ["ls "+dataset +' ' for dataset in gt] + [None]
                return results[state]
            elif(len(cmds)==2):
                datasets = [dataset for dataset in gt if dataset.startswith(cmds[1])]
                results = ["ls "+dataset+' '  for dataset in datasets] + [None]
                return results[state]
            else:
                return [None]
        elif(text.startswith("set")):
            cmds = text.split(' ')
            if(len(cmds)==1):
                results = ["set "+dataset+' '  for dataset in gt] + [None]
                return results[state]
            elif(len(cmds)==2):
                datasets = [dataset for dataset in gt if dataset.startswith(cmds[1])]
                results = ["set "+dataset+' '  for dataset in datasets] + [None]
                return results[state]
            elif(len(cmds)==3):
                if(not cmds[1] in gt):
                    return [None]
                results = ["set " + cmds[1] + " " + sequence+' '  for sequence in gt[cmds[1]] if sequence.startswith(cmds[2])]+[None]
                return results[state]
            else:
                return [None]
        else:
            results = [cmd+' '  for cmd in commands if text in cmd] + [None]
            return results[state]
            
    return completor

def visualize(results, gt):
    rospy.init_node('results_visualizer', disable_signals=True)

    # initialize publishers
    publishers={}
    for algorithm in results.keys():
        publishers[algorithm] = rospy.Publisher("/{}_path".format(algorithm),Path, latch=True, queue_size=10)

    publishers["gt"] = rospy.Publisher("/gt_path",Path, latch=True, queue_size=10)

    current_dataset = None
    current_sequence = None
    display_commands()
    
    readline.set_completer_delims('\t')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(create_completor(gt))

    try:
        while(True):
            commands = input().split()
            if(len(commands)==0):
                display_commands()
                if(current_dataset is not None):
                    for algorithm in results.keys():
                        publishers[algorithm].publish(results[algorithm][current_dataset][current_sequence])
                    
                    publishers["gt"].publish(gt[current_dataset][current_sequence])
                    print("Visualization Refreshed.")
            elif(commands[0] == "ls"):
                list(commands, gt)
            elif(commands[0] == "set"):
                if(set_current(commands, results, gt, publishers)):
                    current_dataset = commands[1]
                    current_sequence = commands[2]
            elif(commands[0] == "information"):
                information([current_dataset, current_sequence], results, gt)
            elif(commands[0] == "align"):
                align([current_dataset, current_sequence], results, gt, publishers)
            elif(commands[0] == "end" or commands[0] == "quit"):
                break
            else:
                print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))
    except KeyboardInterrupt as e:
        print('\nShutting down...')

    return