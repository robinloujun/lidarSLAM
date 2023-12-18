#!env python
# -*- coding: utf-8 -*-

"""
This file is to convert the velodyne data and the ground truth pose of Kitti Odometry Benchmark to rosbag file
It is based on the functions from kiiti2bag
"""

import sys

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)
    
import tf
import os
#import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
import datetime as dt
from datetime import datetime
from std_msgs.msg import Header
#from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
from sensor_msgs.msg import PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform, PoseStamped
#from cv_bridge import CvBridge
import numpy as np
import argparse

__author__ = 'Jun Lou'

def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv

def get_static_transform(from_frame_id, to_frame_id, transform):
    " Get the tf_msg needed parameters from names of frame IDs and a transformation matrix"
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg

def save_static_transforms(bag, transforms, global_timestamps):
    "Save the tf_static (velodyne to ground truth pose) if the sequence belongs to [ 00 - 10 ] "
    print("Exporting static transformations")

    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in global_timestamps:
        time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)

def save_dynamic_tf(bag, kitti, global_timestamps):
    print("Exporting time dependent transformations")

    for timestamp, tf_matrix in zip(global_timestamps, kitti.poses):
        tf_msg = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        tf_stamped.header.frame_id = 'world'
        tf_stamped.child_frame_id = 'base_link'
        
        T_world_to_base = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        
        t = tf_matrix[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(tf_matrix)
        transform = Transform()

        transform.translation.x = t[0]
        transform.translation.y = t[1]
        transform.translation.z = t[2]

        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        tf_stamped.transform = transform
        tf_msg.transforms.append(tf_stamped)

        bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)
    
def save_pose(bag, kitti, frame_id, topic, global_timestamps):
    print("Exporting ground truth poses")
    
    pose_msg = PoseStamped()
    
    iterable = zip(global_timestamps, kitti.poses)
    bar = progressbar.ProgressBar()    
    
    for timestamp, pose_tf_matrix in bar(iterable):
        if timestamp is None:
            continue
        
        tr = pose_tf_matrix[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(pose_tf_matrix)
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(tr[0])
        pose_msg.pose.position.y = float(tr[1])
        pose_msg.pose.position.z = float(tr[2])
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        
        pose_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))

        bag.write(topic, pose_msg, t=pose_msg.header.stamp)    

def save_velo_data(bag, kitti, velo_frame_id, topic, global_timestamps):
    print("Exporting velodyne data")
    velo_data_dir = os.path.join(kitti.sequence_path, 'velodyne')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    
    iterable = zip(global_timestamps, velo_filenames)
    bar = progressbar.ProgressBar()
    
    for dt, filename in bar(iterable):
        if dt is None:
            continue
        
        velo_filename = os.path.join(velo_data_dir, filename)
        
        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)
        
        bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)
    
def main():
    
    parser = argparse.ArgumentParser(description = "Convert KITTI Odometry Benchmark to ROS bag file")
    odometry_sequences = [];
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))
    
    parser.add_argument("dir", nargs = "?", default = os.getcwd(), help = "base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-s", "--sequence", choices = odometry_sequences,help = "sequence of the odometry dataset (between 00 - 21).")
    args = parser.parse_args()
    
    compression = rosbag.Compression.NONE
    
    if args.sequence == None:
        print("Sequence option is not given. It is mandatory for odometry dataset.")
        print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
        sys.exit(1)
        
    bag = rosbag.Bag("kitti_odometry_seq_{}.bag".format(args.sequence), 'w', compression=compression)
    
    kitti = pykitti.odometry(args.dir, args.sequence)
    if not os.path.exists(kitti.sequence_path):
        print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
        sys.exit(1)

    # Load the calib.txt for tf_static between cam0 and velo
    kitti._load_calib() 
    #kitti._load_timestamps() # Not needed cause it is in the format of datetime.timedelta
    
    with open(os.path.join(kitti.sequence_path, 'times.txt')) as f:
        lines = f.readlines()
        global_timestamps = []
        for line in lines:
            if len(line) == 1:
                continue
            delta_t = dt.timedelta(seconds=float(line))
            #t = datetime.utcnow() + delta_t
            t = datetime(1992, 11, 25) + delta_t
            global_timestamps.append(t)
    
    # Check if the timestamp file exist
    if len(global_timestamps) == 0:
        print('Did you forget to download the timestamp files? It is in the data_odometry_calib.zip.')
        sys.exit(1)
    
    # Load the ground truth file for sequence 00-10 
    if args.sequence in odometry_sequences[:11]:
        print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
        kitti._load_poses()
        
    try:
        # Define the rostopics and frame IDs
        pose_frame_id = 'pose'
        pose_topic = '/ground_truth_pose'
        velo_frame_id = 'velo_link'
        velo_topic = '/velodyne_points'
        
        T_base_link_to_velo = np.eye(4, 4)
        T_base_link_to_velo[0:3, 3] = [2.71/2.0-1.68-0.27, 0, 1.73]
        T_base_link_to_cam0 = np.eye(4, 4)
        T_base_link_to_cam0[0:3, 3] = [2.71/2.0-1.68, 0, 1.65]
        
        # Load the transformation matrix between cam0 (base of ground truth) and velo
        T_cam_velo = kitti.calib.T_cam0_velo
        
        # tf_static
        transforms = [
            ('base_link', velo_frame_id, T_base_link_to_velo),
            ('base_link', pose_frame_id, T_base_link_to_cam0)
        ]
        
        util = pykitti.utils.read_calib_file(os.path.join(args.dir,'sequences',args.sequence, 'calib.txt'))
        
        # Export
        save_static_transforms(bag, transforms, global_timestamps)
        save_dynamic_tf(bag, kitti, global_timestamps)
        save_pose(bag, kitti, pose_frame_id, pose_topic, global_timestamps)
        save_velo_data(bag, kitti, velo_frame_id, velo_topic, global_timestamps)
    
    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()

if __name__ == '__main__':
    main()
    