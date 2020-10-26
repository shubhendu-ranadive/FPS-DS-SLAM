# FPS-DS-SLAM (Work under Progress)
**Author:** [Shubhendu Ranadive](https://github.com/shubhendu-ranadive)

FPS-DS-SLAM is a robust visual SLAM library that can **identify and remove dynamic feature points** for stereo camera configurations. More specificly, the Mask R-CNN is applied to extract dynamic objects from input frame. Then a mask is applied to feature extractor to remove dynamic keypoints in each level of image pyramid.

The system shows superior result in camera trajectory estimation of KITTI dataset compared to origin ORB-SLAM2.

# 1. License

FPS-DS-SLAM is built upon ORB-SLAM2 and under the same license accordingly.

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).


# 2. Prerequisites
The library is tested only in Ubuntu 18.04 and OpenCV 3.4.9. OpenCV dnn module is required for dnn inference.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features, and to apply CNN model inference. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.4.5 with dnn module support**.

Recommend install OpenCV from source with OpenCL turned on. This can dramatically speed up CNN inference with OpenCL GPU support.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## git-lfs 
We use [git-lfs](https://git-lfs.github.com/) to track big fils in ModelsCNN/ directory.

# 3. Building Dynamic-ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/shubhendu-ranadive/FPS-DS-SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd FPS-DS-SLAM
chmod +x build.sh
./build.sh
```

This will create executables **test**, **mask_rcnn**, **stereo_kitti** and **stereo_euroc** in *Examples* folder.

# 4. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER results/CamPoses.txt ModelsCNN/
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
# 7. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the KITTI datasets for stereo cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 8. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

# 9. Saving Map Points
The map points are saved automatically to *MapPointsSave.xyz* file. This xan be converted to pcd or ply using *Point Cloud Library* [PCL](https://pointclouds.org/downloads/)

# Acknowledgement

The entire project is build upon [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
