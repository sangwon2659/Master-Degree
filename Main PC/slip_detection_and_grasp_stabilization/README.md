# Install NVIDIA Driver 410, CUDA 10.0 and cuDNN v7.5.1 on Ubuntu 16.04

## Install the latest NVIDIA driver
Update package lists, download and install NVIDIA driver  
(nvidia-410 for ubuntu 16.04 // nvidia-driver-410 for ubuntu 18.04)
```
sudo apt-get update
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
sudo apt-get install nvidia-410
sudo apt-mark hold nvidia-410
sudo reboot
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
  

## CUDA Toolkit 10.0
CUDA Toolkit on NVIDIA official website
https://developer.nvidia.com/cuda-downloads

You have to make some choices about your machine to download the file
### My machine:
- Linux
- x86_64
- Ubuntu
- 16.04
- runfile (local)

With the choices made, the following will show (may have to put in key again according to comments in console)
https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=deblocal  
Download the file and execute the following

```
cd Downloads
sudo dpkg -i cuda-repo-ubuntu1604-10-0-local-10.0.130-410.48_1.0-1_amd64.deb
sudo apt-key add /var/cuda-repo-10-0-local-10.0.130-410.48/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda
sudo apt-mark hold cuda
```

You may have to make some choices in the terminal:

- Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 384.81? ``` n ```
- Install the CUDA 10.0 Toolkit? ``` y ```
- Do you want to install a symbolic link at /usr/local/cuda? ``` y ```
- Install the CUDA 10.0 Samples? ``` y ```

Leave the rest as default. ``` ENTER ```

### Export CUDA path
Save the following in bashrc
```
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.0/bin:$PATH
```

### Reload bash
```
source ~/.bashrc
```

### Reboot
Restart the computer
```
reboot
```

### Testing
```
nvcc  --version
```
  

## Install cuDNN
### Download cuDNN v7.5.1 (April 22, 2019), for CUDA 10.0
Download 'cuDNN Library for Linux' on NVIDIA official website
https://developer.nvidia.com/rdp/cudnn-archive

NOTE: *You have to sign up at NVIDIA website before*

### Path setup
Type the following to know the desired path

```
which nvcc
```
### Copying files according to path 
Execute the following
```
tar xzvf cudnn-10.0-linux-x64-v7.5.1.10.tgz
sudo cp cuda/lib64/* /usr/local/cuda-10.0/lib64/
sudo cp cuda/include/* /usr/local/cuda-10.0/include/
sudo chmod a+r /usr/local/cuda-10.0/lib64/libcudnn*
sudo chmod a+r /usr/local/cuda-10.0/include/cudnn.h
``` 

***
OR do the following (NOT RECOMMENDED)
  
Download the three packages:
- cuDNN Runtime Library for Ubuntu16.04 (Deb)
- cuDNN Developer Library for Ubuntu16.04 (Deb)
- cuDNN Code Samples and User Guide for Ubuntu16.04 (Deb)

### Install the .deb packages
```
cd Downloads
sudo dpkg -i libcudnn7_7.5.1.10-1+cuda10.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.5.1.10-1+cuda10.0_amd64.deb
sudo dpkg -i libcudnn7-doc_7.5.1.10-1+cuda10.0_amd64.deb
```
***  

### Testing
```
cat /usr/local/cuda/include/cudnn.h | grep CUDNN_MAJOR -A 2
```
  

# Install Tensorflow & Keras
### Install pip
```
sudo apt-get install python-pip python-dev
```
### Linux-64bit GPU python2.7 Version
```
export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow-0.9.0-cp27-none-linux_x86_64.whl
```
### Install Tensorflow
```
sudo pip install --upgrade $TF_BINARY_URL
```
### Install Keras
```
sudo pip install keras
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
