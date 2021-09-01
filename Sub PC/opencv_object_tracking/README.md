# ROS Kinetic and Wrapper (Intel&reg; RealSense&trade; Devices)
These are packages for using Intel RealSense cameras (D400 series SR300 camera and T265 Tracking Module) with ROS.

## Install ROS Kinetic
#### Setup sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
#### Setup keys
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
#### Update
```
sudo apt-get update
```
#### Installation
```
sudo apt-get install ros-kinetic-desktop-full
```
#### Environment Setup
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Dependencies
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Workspace
#### Make Folder
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

#### Git clone source file
```
catkin_make
```

### Bashrc
#### Load ROS Kinetic Setup
```
source ~/catkin_ws/devel/setup.bash
```

#### Configure ROS Network (As 2nd PC)
```
export ROS_MASTER_URI=http://165.132.139.127:11311
export ROS_IP=165.132.139.120
```

#### Configure ROS alias command
```
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
```

## Wrapper installation instructions

### There are 2 sources to install realsense2_camera from:

* ### Method 1: The ROS distribution:

  *Ubuntu*

    realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:
    
    ```sudo apt-get install ros-$ROS_DISTRO-realsense2-camera```

    This will install both realsense2_camera and its dependents, including librealsense2 library and matching udev-rules.

    Notice:
    * The version of librealsense2 is almost always behind the one availeable in RealSense&trade; official repository.
    * librealsense2 is not built to use native v4l2 driver but the less stable RS-USB protocol. That is because the last is more general and operational on a larger variety of platforms.
    * realsense2_description is available as a separate debian package of ROS distribution. It includes the 3D-models of the devices and is necessary for running launch files that include these models (i.e. rs_d435_camera_with_model.launch). It can be installed by typing:
    `sudo apt-get install ros-$ROS_DISTRO-realsense2-description`

* ### Method 2: The RealSense&trade; distribution:
     > This option is demonstrated in the [.travis.yml](https://github.com/intel-ros/realsense/blob/development/.travis.yml) file. It basically summerize the elaborate instructions in the following 2 steps:


   ### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0

- Register the server's public key:  
  ```
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-  key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  ```
  In case the public key still cannot be retrieved, check and specify proxy settings: 
  ```
  export http_proxy="http://<proxy>:<port>"
  ```
  , and rerun the command. See additional methods in the following [link] (https://unix.stackexchange.com/questions/361213/unable-to-add-gpg-key-with-apt-key-behind-a-proxy).  

- Add the server to the list of repositories:  
  ```
  sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
  ```

- Install the libraries (see section below if upgrading packages):  
  ```
  sudo apt-get install librealsense2-dkms
  sudo apt-get install librealsense2-utils
  ```
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

- Optionally install the developer and debug packages:  
  ```
  sudo apt-get install librealsense2-dev
  sudo apt-get install librealsense2-dbg
  ```

- Reconnect the Intel RealSense depth camera and run: 
  ```
  realsense-viewer
  ```

- Verify that the kernel is updated :    
  ```
  modinfo uvcvideo | grep "version:"
  ```
  should include `realsense` string
  
- Refresh the local packages cache by invoking:  
  ```
  sudo apt-get update
  sudo apt-get upgrade
  ```
  
- To upgrade selected packages only a more granular approach can be applied:  
  ```
  sudo apt-get --only-upgrade install <package1 package2 ...>
  sudo apt-get --only-upgrade install  librealsense2-utils librealsense2-dkms`
  ```
  
   #### OR
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.48.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 2: Install Intel&reg; RealSense&trade; ROS from Sources
- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src/
   ```
- Clone the latest Intel&reg; RealSense&trade; ROS from [here](https://github.com/intel-ros/realsense/releases) into 'catkin_ws/src/'
   ```bashrc
   git clone https://github.com/IntelRealSense/realsense-ros.git
   cd realsense-ros/
   git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
   cd ..
   ```

  ```bash
  catkin_init_workspace
  cd ..
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  ```

## Usage Instructions

### Start the camera node
To start the camera node in ROS:

```bash
roslaunch realsense2_camera rs_camera.launch
```

### Published Topics
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/sample
- /diagnostics

>Using an L515 device the list differs a little by adding a 4-bit confidence grade (pulished as a mono8 image):
>- /camera/confidence/camera_info
>- /camera/confidence/image_rect_raw
>
>It also replaces the 2 infrared topics with the single available one:
>- /camera/infra/camera_info
>- /camera/infra/image_raw


The "/camera" prefix is the default and can be changed. Check the rs_multiple_devices.launch file for an example.
If using D435 or D415, the gyro and accel topics wont be available.

### Point Cloud
Here is an example of how to start the camera node and make it publish the point cloud using the pointcloud option.
```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```
Then open rviz to watch the pointcloud:
<p align="center"><img src="https://user-images.githubusercontent.com/17433152/35396613-ddcb1d6c-01f5-11e8-8887-4debf178d0cc.gif" /></p>

### Aligned Depth Frames
Here is an example of how to start the camera node and make it publish the aligned depth stream to other available streams such as color or infra-red.
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
<p align="center"><img width=50% src="https://user-images.githubusercontent.com/17433152/35343104-6eede0f0-0132-11e8-8866-e6c7524dd079.png" /></p>

### Set Camera Controls Using Dynamic Reconfigure Params
The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/55330573-065d8600-549a-11e9-996a-5d193cbd9a93.PNG" /></p>

### Work with multiple cameras
**Important Notice:** Launching multiple T265 cameras is currently not supported. This will be addressed in a later version. 

Here is an example of how to start the camera node and streaming with two cameras using the [rs_multiple_devices.launch](./realsense2_camera/launch/rs_multiple_devices.launch).
```bash
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=<serial number of the first camera> serial_no_camera2:=<serial number of the second camera>
```
The camera serial number should be provided to `serial_no_camera1` and `serial_no_camera2` parameters. One way to get the serial number is from the [rs-enumerate-devices](https://github.com/IntelRealSense/librealsense/blob/58d99783cc2781b1026eeed959aa3f7b562b20ca/tools/enumerate-devices/readme.md) tool.
```bash
rs-enumerate-devices | grep Serial
```

Another way of obtaining the serial number is connecting the camera alone, running
```bash
roslaunch realsense2_camera rs_camera.launch
```
and looking for the serial number in the log printed to screen under "[INFO][...]Device Serial No:".

Another way to use multiple cameras is running each from a different terminal. Make sure you set a different namespace for each camera using the "camera" argument:

```bash
roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=<serial number of the first camera>
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=<serial number of the second camera>
...

```
