# Workspace

## Todo

- [ ] build the basic framwork of ICP based on libpointmatcher

## Open-sourced Packages

### [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
An "Iterative Closest Point" library for 2-D/3-D mapping in Robotics.  
[\[Tutorial\]](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/index.md) for libpointmatcher.

### Open3D ( [homepage](http://www.open3d.org/docs/) / [Git Repo](https://github.com/IntelVCL/Open3D) )
A modern library for 3D data processing (ICP integrated), but mainly used in Python.  
Note to use `make -j4` instead of `make -j`, otherwise there will be the problem "virtual memory exhausted"  
[\[Tutorial\]](http://www.open3d.org/docs/tutorial/C++/cplusplus_interface.html#cplusplus-interface-tutorial) for C++ Interface

### [limo - Github](https://github.com/johannes-graeter/limo) [limo - paper](https://arxiv.org/pdf/1807.07524.pdf)  
Lidar-Monocular Visual Odometry 2  

- Note for initiating the workspace(mkdir a src folder before configuration) ```mkdir -p catkin_ws/src``` and then ```catkin init``` in ${catkin_ws}  
- Note for unittests: no arguement --this  
    ```catkin run_tests --profile limo_release```
- Note for rospack: source setup.bash instead of setup.sh  
    ```source catkin_ws/devel_limo_release/setup.bash```

### [kitti2bag](https://github.com/tomas789/kitti2bag)
Python tool for converting KITTI dataset to ROS bag file.

#### Convert the odometry dataset
Odometry dataset cannot be converted directly by the kitti2bag, bus it is a subset of the raw_data dataset and here is a list of correspondence:

<details>
  <summary>List of correspondence between odometry and raw data</summary>
  <p> <table border="">
  <tr>
    <th>Odom</th>     <th>Raw</th>     <th>Start</th>     <th>End</th>      <th>Catogory</th>       <th>rosbag</th>
  </tr>
  <tr>
    <td>00:</td>     <td>2011_10_03_drive_0027</td>     <td>000000</td>     <td>004540</td>     <td>Residential</td>        <td>Converted</td>
  </tr>
  <tr>
    <td>01:</td>     <td>2011_10_03_drive_0042</td>     <td>000000</td>     <td>001100</td>     <td>Road</td>       <td>Converted</td>
  </tr>
  <tr>
    <td>02:</td>     <td>2011_10_03_drive_0034</td>     <td>000000</td>     <td>004660</td>     <td>Residential</td>        <td>Converted</td>
  </tr>
  <tr>
    <td>03:</td>     <td>2011_09_26_drive_0067</td>     <td>000000</td>     <td>000800</td>     <td>not available</td>
  </tr>
  <tr>
    <td>04:</td>     <td>2011_09_30_drive_0016</td>     <td>000000</td>     <td>000270</td>     <td>Road</td>       <td>Converted</td>
  </tr>
  <tr>
    <td>05:</td>     <td>2011_09_30_drive_0018</td>     <td>000000</td>     <td>002760</td>     <td>Residential</td>        <td>Converted</td>
  </tr>
  <tr>
    <td>06:</td>     <td>2011_09_30_drive_0020</td>     <td>000000</td>     <td>001100</td>     <td>Residential</td>        <td>Converted</td>
  </tr>
  <tr>
    <td>07:</td>     <td>2011_09_30_drive_0027</td>     <td>000000</td>     <td>001100</td>     <td>Residential</td>
  </tr>
  <tr>
    <td>08:</td>     <td>2011_09_30_drive_0028</td>     <td>001100</td>     <td>005170</td>     <td>Residential</td>
  </tr>
  <tr>
    <td>09:</td>     <td>2011_09_30_drive_0033</td>     <td>000000</td>     <td>001590</td>     <td>Residential</td>
  </tr>
  <tr>
    <td>10:</td>     <td>2011_09_30_drive_0034</td>     <td>000000</td>     <td>001200</td>     <td>Residential</td>
  </tr>
</table></p>
</details>

### QtKittiVisualizer
a QT-based visualizer called [QtKittiVisualizer](https://github.com/MarkMuth/QtKittiVisualizer) for point cloud and tracklet sequences  
Note that the PLC version > 1.8 needed, which should be installed [from source](https://github.com/PointCloudLibrary/pcl/releases)

## Dataset
KITTI - Visual Odometry / SLAM Evaluation 2012   
- [grayscale](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip) (22GB)
- [color](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip) (65GB)
- [laser data](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip) (80GB)
- [ground truth poses](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip) (4MB), which is already [downloaded](https://agithub.com/kailaili/lidarDirectionalSLAM/tree/master/3_MATLAB/dataset/poses)
- [calibration files](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip)

## Done

- [x] Edit the Proposal (2018.08.23) (2018.08.29) (2018.09.04)
- [x] Visualization of KITTI Odometry Ground Truth (2018.08.24) -> [trajectory_mapping.m](https://github.com/kailaili/lidarDirectionalSLAM/blob/master/3_matlab/trajectory_mapping.m#L1)
- [x] Visualization of KITTI LiDAR point cloud (2018.09.12) -> [pcl_visual.cpp
](https://github.com/kailaili/lidarDirectionalSLAM/blob/master/2_cpp/pcl_visual.cpp#L1)
