# mic_monitor

A simple ROS2 package for detecting noise levels using a USB microphone array (e.g., ReSpeaker 4 Mic Array).


## Overview
`mic_monitor` captures audio input, computes the sound energy, and publishes the value to a ROS2 topic `/noise_level`.  
It can be integrated into the GuardianBot project to enable sound-triggered behaviors.


## Installation
```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select mic_monitor
source install/setup.bash
 
##Check available microphones
ros2 run mic_monitor list_mics

##ros2 run mic_monitor list_mics
ros2 run mic_monitor mic_monitor_node

##View the output
ros2 topic echo /noise_level

##Parameters
##Name             Default   Description                 
##device_index       0       Microphone input device index 
##noise_threshold   0.02     Threshold for noise detection 


