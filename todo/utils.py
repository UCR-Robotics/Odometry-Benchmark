import os
import argparse
import shutil
import string
import json
import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import yaml
import tf.transformations
import numpy as np

# During the statistics calculation of the ground truth, 
# movement between two frames will count only if it is greater than this value. 
mobile_thres = 0.01
# the velocity of a frame is the average velocity in a window centered at the frame.
median_radius = 2   

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

def calculate_path_distance(path:Path):
    ret=[0]
    for i in range(1,len(path.poses)):
        ret.append(pose_distance(path.poses[i],path.poses[i-1]))
    
    for i in range(1, len(ret)):
        ret[i]+=ret[i-1]
    return ret

# max_vel, avg_vel, max_ang, avg_ang, total_dist, total_time
vel_range=median_radius
def path_statistics(path:Path):
    max_velocity=0
    max_ang=0
    total_distance=0
    total_d_angular=0
    time=0
    for i in range(vel_range,len(path.poses)-vel_range):
        dist,d_ang=pose_diff(path.poses[i].pose,path.poses[i-1].pose)
        dt=(path.poses[i].header.stamp-path.poses[i-1].header.stamp).to_sec()
        # if(dist>mobile_thres):
        #     total_distance+=dist
        #     total_d_angular+=d_ang
        #     time+=dt
        total_distance+=dist
        total_d_angular+=d_ang
        time+=dt
        vel,ang=pose_diff(path.poses[i-vel_range].pose,path.poses[i+vel_range].pose)
        vel/=(path.poses[i+vel_range].header.stamp-path.poses[i-vel_range].header.stamp).to_sec()
        ang/=(path.poses[i+vel_range].header.stamp-path.poses[i-vel_range].header.stamp).to_sec()
        if(vel>max_velocity):
            max_velocity=vel
        if(ang>max_ang):
            max_ang=ang
    return max_velocity,total_distance/time, max_ang, total_d_angular/time, total_distance, time


def transform_pose(T, pose:Pose)->Pose:
    T2=tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    T2[0:3,3]=[pose.position.x, pose.position.y, pose.position.z]
    
    result=T@T2
    result[0:3,0:3]=result[0:3,0:3]@T[0:3,0:3].T
    result_p=result[0:3,3]
    result_q=tf.transformations.quaternion_from_matrix(result)
    ret=Pose()
    ret.position.x=result_p[0]
    ret.position.y=result_p[1]
    ret.position.z=result_p[2]
    ret.orientation.x=result_q[0]
    ret.orientation.y=result_q[1]
    ret.orientation.z=result_q[2]
    ret.orientation.w=result_q[3]
    return ret