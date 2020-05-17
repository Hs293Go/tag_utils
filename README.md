# tag utils

A collection of rosnodes to run a AprilTag3 visual fiducial system on an Nvidia Jetson Nano

## Using AprilTag 4
The Nvidia Jetson SD card image is pre-installed with opencv 4, but ROS melodic is tested with opencv 3.2. On the other hand, the installed version already provides GStreamer support, while opencv installed as a ubuntu package does not

To use opencv4 preinstalled in the Jetson Nano, vision\_opencv must be recompiled from a un-merged pull request of the source. The discussion about opencv4 compatibility is at: https://github.com/ros-perception/vision\_opencv/issues/272

First clone the vision\_opencv repo
```
    git clone https://github.com/ros-perception/vision_opencv.git
```

Next fetch the [pull request](https://github.com/ros-perception/vision_opencv/pull/259) that provides opencv4 compatibility and check it out
```
    git fetch origin pull/259/head:cv4 && git checkout cv4 
```

Finally, build vision\_ws with either `catkin_make` or `catkin build`.
