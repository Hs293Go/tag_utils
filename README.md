# tag utils

A collection of rosnodes to run a AprilTag3 visual fiducial system on an Nvidia Jetson Nano

## Camera Calibration
This package provides a script to streamline the camera calibration process. Prior to carrying out camera calibration, install requisite packages
```
sudo apt-get install ros-melodic-camera-calibration
sudo apt-get install ros-melodic-camera-calibration-parsers
```

When carrying out camera calibration through a ssh connection, X-forwarding must be enabled to display the UI of `cameracalibrator.py`. Hence the ssh command must be
```
ssh -X hostname@host
```

Next, run
```
roslaunch tag_utils calibrate_camera.launch size:=10X8 square:=0.2
```
where `10x8` and `0.2` should be replaced with the actual checkerboard's grid count along each edge and size of each individual grid. 

The UI of `cameracalibrator.py` consists of a video window, 3 progress bars and the buttons **calibrate**, **save** and **commit**. Start calibrating by moving the checkerboard in the video window
 - Move the checkerboard horizontally and vertically to fill the **x** and **y** progress bar
 - Move the checkerboard close and far to fill the **size** progress bar
 - Tilt the checkerboard to its sides to fill the **skew** progress bar

Once all progress bars are sufficiently filled, the **calibrate** button will be highlighted. Click on it and wait for up to several minutes. The UI will be frozen as this happens. Once the UI is responsive again, 
 - Click on **save** to keep raw data
 - Click on **commit** to write the data to the `camera_info` folder

If `cameracalibrator.py` did not commit data to the `camera_info` folder, take the raw `.txt` file, then run
```
mv result.txt result.ini    # Blindly converting to .ini format is normal
rosrun camera_calibration_parsers convert result.ini camera_info.yaml
```

## Using AprilTag 4
The Nvidia Jetson SD card image is pre-installed with opencv 4, but ROS melodic is tested with opencv 3.2. On the other hand, the installed version already provides GStreamer support, while opencv installed as a ubuntu package does not

To use opencv4 preinstalled in the Jetson Nano, vision\_opencv must be recompiled from a un-merged pull request of the source. The discussion about opencv4 compatibility is at: https://github.com/ros-perception/vision_opencv/issues/272

First clone the vision\_opencv repo
```
    git clone https://github.com/ros-perception/vision_opencv.git
```

Next fetch the [pull request](https://github.com/ros-perception/vision_opencv/pull/259) that provides opencv4 compatibility and check it out
```
    git fetch origin pull/259/head:cv4 && git checkout cv4 
```

Finally, build vision\_opencv with either `catkin_make` or `catkin build`.
