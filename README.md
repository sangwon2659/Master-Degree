# Masters
Masters Degree at Yonsei University

Slip Detection and Grasp Stabilization with Low-Cost Normal Force Sensor Arrays

Hardware:  
-10 Honeywell FSS Sensors  
-Robotis Linear Gripper  
-Linear Railway with a Step Motor  
-3 Arduinos  
-2 PC with Ubuntu 16.04 & ROS Kinetic  
-Real Sense D435i  
-3 Objects of Different Textures with an ArUco Marker Attached  

Summary:  
FSS Data collection done with the 3 objects and the ArUco marker system  
Data converted to sequences of covariance and frequency data and trained through a lookback multi-layered perceptron network (slip or no-slip binary classification network)  
Network is applied back to the hardware with an additional grasp stabilization algorithm  
