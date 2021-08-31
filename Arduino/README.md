# ROS Arduino
## Install Arduino IDE
### Download Arduino IDE Linux Version
https://www.arduino.cc/en/Main/Software

### Extract and install
```
cd Downloads
cd arduino-X.X.X/
./install.sh
```

### Permission to serial port
```
cd /dev
ls -al
sudo usermod -a -G dialout <username>
ls -al
```

### Setup
```
gedit ~/.bashrc
export  PATH=$PATH:$HOME/tools/arduino-1.6.4
source ~/.bashrc
```

### Ros serial arduino install
```
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```

### Ros lib install (arduino IDE path = where install.sh was executed)
```
cd <arduino IDE path>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

### Check ros_lib installation
```
cd <arduino IDE path>/libraries
ls -al
```

### Starting node
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```
