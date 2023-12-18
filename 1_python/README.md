# Usage

put the file into the folder where you'd like to save the rosbag
### Example (both for sequence 04)
#### From raw data dataset: 
<!--current path `/media/robin/Harddisk_thesis/Raw_Data/`-->
```
cd ${/path/to/Raw_Data}
python kitti2bag.py -t 2011_09_30 -r 0016 raw_synced .
```
#### From odometry dataset:  
```
cd ${/Path/to/Your_File_Name}
python odom2bag.py -s 04 .
```
The files are supposed to be arranged into the following structure:
```
{$Your_File_Name}
    odom2bag.py
    sequences
        00
            velodyne
                000000.bin
                ...
            calib.txt (storing the calibration info)
            times.txt (storing the time stamps)
        01
        ...
    poses (storing the ground truth of seq 00 - 10)          
        00.txt
        01.txt
        ...
```
<!---current path `/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/`-->


