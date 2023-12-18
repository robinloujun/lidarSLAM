Following are versions of Operating System and the dependencies on my laptop:

|Name           |Version                |Name           |Version                |
|---------------|-----------------------|---------------|-----------------------|
|Ubuntu         | 16.04.11 LTS (64 bit) |Qt5            | 5.9.0                 |
|gcc            | 5.4.0                 |Boost          | 1.58.0.1              |
|Git            | 2.7.4                 |Eigen          | 3.3~beta1-2           |
|CMake          | 3.5.1                 |OpenCV         | 2.4.9                 |
|libnabo        | [from source](https://github.com/ethz-asl/libnabo#quick-compilation-and-installation-under-unix)|VTK            | 6.2.0                 |
|libpointmatcher| [from source](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md#5-installing-libpointmatcher)|PCL| 1.8.1|

<details>
  <summary>You can install/ upgrade/ check versions with the following commands</summary>
  <table border="">
  <tr>
    <th>Name</th>     <th>Commands to check the version</th>     <th>Install or manual upgrade</th>
  </tr>
  <tr>
    <td>Ubuntu:</td>     <td>lsb_release -r</td>     <td></td>
  </tr>
  <tr>
    <td>Architecture of OS:</td>     <td>getconf LONG_BIT</td>     <td></td>
  </tr>
  <tr>
    <td>Compiler:</td>     <td>gcc --version</td>     <td></td>
  </tr>
  <tr>
    <td>Git:</td>     <td>git --version</td>     <td>sudo apt-get install git-core</td>
  </tr>
  <tr>
    <td>CMake:</td>     <td>cmake --version</td>     <td>sudo apt-get install cmake cmake-gui</td>
  </tr>
  <tr>
    <td>Boost:</td>     <td>dpkg -s libboost-dev | grep Version</td>     <td>sudo apt-get install libboost-all-dev</td>
  </tr>
  <tr>
    <td>Eigen:</td>     <td>dpkg -s libeigen3-dev | grep Version</td>     <td>sudo apt-get install libeigen3-dev</td>
  </tr>
  <tr>
    <td>OpenCV:</td>     <td>dpkg -l | grep libopencv</td>     <td>Install <a href="https://docs.opencv.org/3.4.1/d7/d9f/tutorial_linux_install.html">OpenCV</a> from source</td>
  </tr>  
  <tr>
    <td>VTK:</td>     <td>Use <a href="https://www.vtk.org/Wiki/VTK/Examples/Cxx/Utilities/CheckVTKVersion">CheckVTKVersion</a>
    to check the version</td>     <td>Install <a href="https://www.vtk.org/Wiki/VTK/Building/Linux">VTK</a> from source if needed, but it is not recommanded to install the latest version of vtk while using older pcl because there is a code cleanup in later vtk</td>
  </tr>    
  <tr>
    <td>PCL:</td>     <td>ldconfig -v | grep pcl</td>     <td>Install <a href="http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php">PCL</a> from source, note to checkout to tag pcl-1.8.1 if python-pcl later needed</td>
  </tr>   
    
</table>
</details>

---
Install the [Paraview](https://www.paraview.org/Wiki/ParaView:Build_And_Install#Introduction) to visualize the pointclouds , 
note that here Qt5 should be upgraded to Version 5.9.0, while the latest version in repositories is 5.5.1.
