# ORB_SLAM2_Dev
This repository has the process to setup ORB_SLAM2 and how to use it.

The official GitHub repository for ORBS-SLAM2 is linked [here](https://github.com/raulmur/ORB_SLAM2).
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
 - Eigen

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
    
###  Install evo 

The [evo](https://github.com/MichaelGrupp/evo/) package should be installed for visualizing SLAM and odometry data  

    $ sudo pip install evo --upgrade --no-binary evo
    
### Camera calibration

Open 3 tabs

    $ roscore
    
Open a new tab

    $ source devel/setup.bash
    
    $ roslaunch usb_cam usb_cam-test.launch 
    
Open a new tab

    $ source devel/setup.bash
    
    $ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/usb_cam/image_raw camera:=/usb_cam

Run the code and keep moving the checker board until the **Calibrate** button pops blue. 
Click and wait. Hit **save** to get the callibration data stored in the **/tmp** folder.

The callibration data will be saved

Create a new camera parameter file, in KITTI or TUM format. 
Refer [this stackoverflow link](https://stackoverflow.com/questions/34023303/opencv-store-camera-matrix-and-distortion-coefficients-as-mat) and [this](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) for the parameters.

## KITTI Dataset Example

Go to ORB_SLAM2 Package and execute:

    $ ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml Examples/Stereo/kitti_dataset/sequences/00

To plot the path for the kitti dataset

Open a new tab

    $ evo_traj kitti KeyFrameTrajectory.txt --plot
    
In KeyFrameTrajectory.txt file, every row has 8 entries containing time stamp (in seconds), position and orientation: 'timestamp x y z q_x q_y q_z q_w'

So to achieve what you want to do, you could for example load the file as a table (similar to a .csv file) and then the columns 2 to 4 are your x, y, z values (or 1 to 3 if you count from 0)

## Custom USB-Video example

Open 4 tabs

    $ roscore
    
Open a new tab

    $ source devel/setup.bash

    $ roslaunch usb_cam usb_cam-test.launch
    
Open a new tab

    $ source devel/setup.bash
    
    $ rosrun ORB_SLAM2 Mono src/ORB_SLAM2/Vocabulary/ORBvoc.txt src/ORB_SLAM2/Examples/Monocular/nexigo_callibration_data/nexigo_cam.yaml 
    
Open a new tab

    $ evo_traj tum KeyFrameTrajectory.txt --plot
    
## Stereo Mode example

Open 3 tabs

    $ roscore
    
Open a new tab

    $ source devel/setup.bash

    $ rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml false
    
Open a new tab

    $ source devel/setup.bash
    
    $ rosbag play --pause /path/to/V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw

## Converting pointcloud to Occupancy grid

The pointcloud data is stored by the ORBSLAM in a txt file. This can be converted to occupancy grid, visualized in RVIZ and used for robot navigation

Another option is to use pointcloud_to_laserscan package. But LIDAR is always assumed to be noisy, and ORBSLAM uses much more reliable points only. So, this method is not used.

Instead we will try and generate a map from the .txt file.

## Agricultural application
