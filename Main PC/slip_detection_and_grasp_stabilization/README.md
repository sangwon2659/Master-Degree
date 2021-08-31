# Install NVIDIA Driver, CUDA 10.0 and cuDNN v7.4.2 on Ubuntu 16.04

## Install the latest NVIDIA driver
Update package lists, download and install NVIDIA driver
```
sudo apt-get update
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt install nvidia-XXX(Check Recommended Version)
```

### Reboot
Restart the computer
```
reboot
```

### Testing
Lets test if all worked well
```
nvidia-smi
```
If appears something like:

```
Sun Jan 27 15:33:47 2019       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI XXX       Driver Version: XXX.XX                                 |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce XXXX        Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   49C    P5    N/A /  N/A |    495MiB /  2004MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|    0      1091      G   /usr/lib/xorg/Xorg                           285MiB |
|    0      2014      G   compiz                                       103MiB |
|    0      2263      G   ...quest-channel-token=6295865577498169014   103MiB |
+-----------------------------------------------------------------------------+
```
The driver was installed.

## Download CUDA Toolkit
Download the CUDA Toolkit on NVIDIA official website
https://developer.nvidia.com/cuda-downloads

You have to make some choices about your machine to download the file
### My machine:
- Linux
- x86_64
- Ubuntu
- 16.04
- runfile (local)

### Go to the download folder
After download the file, go to the download folder:
```
cd Downloads
```

### Run the file
```
sudo sh cuda_10.0.130_410.48_linux.run
```

You have to make some choices in the terminal:

- Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 384.81? ``` n ```
- Install the CUDA 10.0 Toolkit? ``` y ```
- Do you want to install a symbolic link at /usr/local/cuda? ``` y ```
- Install the CUDA 10.0 Samples? ``` y ```

Leave the rest as default. ``` ENTER ```

### Testing
Let's test if everything worked well

## Install cuDNN
### Download cuDNN v7.4.2 (Dec 14, 2018), for CUDA 10.0
Download the cuDNN on NVIDIA official website
https://developer.nvidia.com/cudnn

NOTE: *You have to sign up at NVIDIA website before*

Download the three packages:
- cuDNN Runtime Library for Ubuntu16.04 (Deb)
- cuDNN Developer Library for Ubuntu16.04 (Deb)
- cuDNN Code Samples and User Guide for Ubuntu16.04 (Deb)

### Go to downloads folder
```
cd Downloads
```

### Install the .deb packages
```
sudo dpkg -i libcudnn7_7.4.2.24-1+cuda10.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.4.2.24-1+cuda10.0_amd64.deb 
sudo dpkg -i libcudnn7-doc_7.4.2.24-1+cuda10.0_amd64.deb
```

### Export CUDA path
```
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.0/bin:$PATH
```

### Reload bash
```
source ~/.bashrc
```

# Install ROS Kinetic
## Installation Process
### Setup sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### Setup keys
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### Installation
```
sudo apt-get install ros-kinetic-desktop-full
```
### Environment Setup
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Dependencies
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Bashrc
### Load ROS Kinetic Setup
```
source ~/catkin_ws/devel/setup.bash
```

### Configure ROS Network
```
export ROS_LOCALIP=165.132.139.127
export ROS_MASTER_URI=http://${ROS_LOCALIP}:11311
```

### Configure ROS alias command
```
alias cs='cd ~/catkin_ws/src/slip_detection_and_grasp_stabilization/scripts'
alias cm='cd ~/catkin_ws && catkin_make'
```

## Workspace
### Make Folder
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
cm
```

### Git clone source file
```
cm
cs
chmod +x *.py
```
