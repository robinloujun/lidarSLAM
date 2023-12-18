## Record rosbag with the Velodyne VLP 16

### Before you record (at the first time)
- Install some dependencies:
```shell
sudo apt-get install libpcap-dev ros-kinetic-velodyne
```

- Configure the IP address
	- Access the Gnome Menu (Super key), type "Networks Connections" then run it. Select the connection's name and click on "edit". Choose the IPV4 Settings tab and change the “Method” field to "Manual".
	- Click on "add" and set the IP address field to 192.168.1.100.
	- Set the “Netmask” to 255.255.255.0 and the "Gateway" to 192.168.1.1.
	- To finish it click on "save". 
	- Restart network card with `sudo /etc/init.d/networking restart`.
	- Check the LiDAR configuration at address: 192.168.1.201.
<p align='center'>
    <img src="/2_cpp/Velodyne/ip_config.png" alt="drawing" width="400"/>
</p>


- Construct catkin workspace for ros-drivers
```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ros-drivers/velodyne.git
cd .. 
rosdep install --from-paths src --ignore-src --rosdistro [kinetic / melodic] -y  
catkin_make
source devel/setup.bash
```

- Generate the calibration yaml file
```shell
rosrun velodyne_pointcloud gen_calibration.py path/VLP-16.xml
```

- Launch the VLP 16 and visualize the point cloud
```shell
roslaunch velodyne_pointcloud VLP16_points.launch calibration:=path/VLP-16.yaml
rosrun rviz rviz -d ./vlp.rviz
```

### Record the rosbag

- Start recording with `bash ./startRecordVLP.sh`

- Finish recoding with `bash ./stopRecordVLP.sh`, do not use `Ctrl + C`

Reference: [velodyne/Tutorials/Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
