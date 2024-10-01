#!/usr/bin/env python3

import rospy
import rosbag
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class BagToPathPublisher:
  def __init__(self, bag_file_path, odom_topic_name):
    # Initialize the ROS node
    rospy.init_node('bag_to_path_publisher', anonymous=True)

    # Create a publisher for the Path message
    self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)

    # Initialize the Path message
    self.path_msg = Path()
    self.path_msg.header.frame_id = 'odom'  # Set the frame ID to match your setup

    # Store the bag file path and topic name
    self.bag_file_path = bag_file_path
    self.odom_topic_name = odom_topic_name

    # Read all odometry data from the bag file and build the path
    self.build_path_from_bag()

  def build_path_from_bag(self):
    # Open the rosbag file
    with rosbag.Bag(self.bag_file_path, 'r') as bag:
      for topic, msg, t in bag.read_messages(topics=[self.odom_topic_name]):
        if topic == self.odom_topic_name:
          # Convert Odometry to PoseStamped
          pose_stamped = PoseStamped()
          pose_stamped.header = msg.header
          pose_stamped.pose = msg.pose.pose

          # Append the pose to the path
          self.path_msg.poses.append(pose_stamped)

    rospy.loginfo(f"Loaded {len(self.path_msg.poses)} poses into the path.")

  def publish_path(self):
    # Set the timestamp of the path message
    self.path_msg.header.stamp = rospy.Time.now()

    # Publish the complete Path message once
    self.path_pub.publish(self.path_msg)
    rospy.loginfo("Published the complete path.")

if __name__ == '__main__':
  try:
    # Provide the path to your bag file and the odometry topic name
    bag_file_path = '/home/hteng/Documents/0-FAIRI/WT-traj/delta10_lamp_2024-09-04-13-05-39_0.bag'  # Replace with your bag file path
    odom_topic_name = '/delta10/lo_frontend/odometry'         # Replace with the odometry topic name in your bag

    # Create an instance of the publisher class
    bag_to_path_publisher = BagToPathPublisher(bag_file_path, odom_topic_name)

    # Publish the path once
    bag_to_path_publisher.publish_path()

    # Keep the node alive to ensure the message is received by RViz
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo('ROS node interrupted. Exiting...')

