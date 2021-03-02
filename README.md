# ORB_SLAM2_Dev
This repository has the process to setup ORB_SLAM2 and how to use it.

THe official GitHub repository for ORBS-SLAM2 is linked [here](https://github.com/raulmur/ORB_SLAM2).
## System:

The below code is tested on:

 - Ubuntu 18.05 LTS
 - NVIDIA GTX 950M

## Pre requisites :

 - ROS
 - OpenCV
 - g2o
 - DBoW2
 - Pangolin

## Installation

Once these are installed, go to your home folder and run the following commands

    $ git clone https://github.com/raulmur/ORB_SLAM2.git
    $ cd ORB_SLAM2
    $ chmod +x build.sh
    $ ./build.sh

A compile issue might arise here. Go to the **/include** folder and add 

    #include <unistd.h>

into **System.h**

## Testing Installation

Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

Execute the following command. Change TUMX.yaml to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change PATH_TO_SEQUENCE_FOLDER to the uncompressed sequence folder.

    $ ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_rpy/
    
You should be able to see the ORB SLAM2 Working now.

Other examples for 

- Monocular / Stereo / RGB-D images

and,

- KITTI / TUM / EuROC MAV datasets

can be found [here](https://github.com/raulmur/ORB_SLAM2).

## ROS Installation

    $ mkdir orb_slam2_ws
    $ cd orb_slam2_ws

    $ mkdir src
    $ cd src
    $ catkin_init_workspace

    $ cd ..
    $ catkin_make
    $ cd src
    $ git clone https://github.com/tianchenliu/ORB_SLAM2
    $ git clone https://github.com/stevenlovegrove/Pangolin

    $ cd Pangolin
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
    $ cd ..

    $ cd ~/orb_slam2_ws/src/ORB_SLAM2/Thirdparty/DBoW2/
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release
    $ make

    $ cd ~/orb_slam2_ws/src/ORB_SLAM2/Thirdparty/g2o/
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release
    $ make

    $ cd ~/orb_slam2_ws/src/ORB_SLAM2
    $ mkdir build
    $ cd build
    $ cmake .. -DROS_BUILD_TYPE=Release
    $ make

    $ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/orb_slam2_ws/src/ORB_SLAM2/Examples/ROS
    $ chmod +x build_ros.sh
    $ ./build_ros.sh

 ## Seting up USB-CAM
  
    $ git clone https://github.com/ros-drivers/usb_cam.git
    $ cd src/usb_cam-develop
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    
  Edit device input (e.g., value=”/dev/video0”) for usb_cam-test.launch file
  
  Check where the input is coming from using this 
  
    $ ls -ltrh /dev/video*
    $ ffplay /dev/video0
    
Edit **ORB_SLAM2/Example/ROS/ORB_SLAM2/src/ros_mono.cc** from 

    ros::Subscriber sub = nodeHandler.subscribe("/image/image_raw", 1, &ImageGrabber::GrabImage,&igb);

to 

    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);

Rebuild the ros package:

    ./build_ros.sh
## Stereo Mode example

Open 3 tabs

    $ roscore

From ORB_SLAM2 package, execute:

    $ rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml false
    
    $ rosbag play --pause /path/to/V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw

## Camera callibration

Open 3 tabs

    $ roscore
    $ roslaunch usb_cam usb_cam-test.launch 
    $ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/usb_cam/image_raw camera:=/usb_cam

Run the code and keep moving the checker board until the **Calibrate** button pops blue. 
Click and wait. Hit **save** to get the callibration data stored in the **/tmp** folder.

The callibration data will be saved

Create a new camera parameter file, in KITTI or TUM format. 
Refer [this stackoverflow link](https://stackoverflow.com/questions/34023303/opencv-store-camera-matrix-and-distortion-coefficients-as-mat) and [this](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) for the parameters.

