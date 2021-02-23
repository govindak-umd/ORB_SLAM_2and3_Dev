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
