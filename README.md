# ORBSLAMM
**Authors:** [Hayyan Afeef Daoud](), [Aznul Qalid Md. Sabri](https://umexpert.um.edu.my/aznulqalid), [Chu Kiong Loo](https://umexpert.um.edu.my/ckloo-um) and [Ali Mohammed Mansoor](https://umexpert.um.edu.my/ali-mansoor)

**Current version:** 1.0.0 

**ORBSLAMM** is a real-time Simultaneous Localization and **Multi-Mapping** system. It handles navigation in a similar way to humans when tracking is lost. This is achieved by creating a new map when tracking is lost, later when the new map intersects with a previously seen map the two are merged (the new knowledge is accumulated) in a loop closure event. This, in contrast to conventional mono-map SLAM systems which use relocalization, ensures information preservation and a better utilization of resources. ORBSLAMM is also flexible to work with multiple robots navigating the environment. This enables exploring and mapping the environment much faster.
ORBSLAMM is built on top of the famous [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). We tested the system using the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) and the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset). To our Knowledge, ORBSLAMM is the first SLAM system that works simultaneously on two sequences of the KITTI dataset (seq 00 and seq 07) in real-time and merge them in one map, using the shared portion of the environment, despite the difference in cameras' calibrations and recording days of these two sequences (see the video below).

##### Videos showing the performance of ORBSLAMM:
<a href="https://www.youtube.com/watch?v=zoFz52jG_YM" target="_blank"><img src="https://img.youtube.com/vi/zoFz52jG_YM/0.jpg" 
alt="KITTI Dataset - Multiple Robots on 2 sequences" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/watch?v=bY81TniW3kQ" target="_blank"><img src="http://img.youtube.com/vi/bY81TniW3kQ/0.jpg" 
alt="TUM RGBD Dataset" width="240" height="180" border="10" /></a>


### ORBSLAMM vs ORB-SLAM
If there is no tracking failure (caused by corrupted frames, sensor's malfunction or a low number of features) the performance of ORBSLAMM and ORB-SLAM is similar except that ORBSLAMM initializes faster using the fundamental matrix only. However, the difference is visible when there is a tracking failure.
Below are two examples of the performance of ORBSLAMM in comparison to the state-of-the-art ORB-SLAM. 

The first scenario is the TUM RGBD _fr2-360-kidnap_ sequence, where the camera is moved in a circular trajectory and in the middle of its movement the camera is covered for some time (kidnapped). This causes the tracking loss and, subsequently, the loss of
all information until the camera returns to its starting point (closes the loop) where the relocalization retrieves the pose of the camera to continue mapping and tracking.

<a href="https://www.youtube.com/watch?v=UW1CeDllQrI" target="_blank"><img src="http://img.youtube.com/vi/UW1CeDllQrI/0.jpg" 
alt="TUM RGBD Dataset - Kidnap" width="240" height="180" border="10" /></a>

The second scenario shows the robustness of ORBSLAMM compared to mono-map SLAM systems such as ORB-SLAM when there is a wrong initialization, and when there is a tracking loss due to a low number of features (wall).

<a href="https://www.youtube.com/watch?v=wDnEj3KGwQM" target="_blank"><img src="http://img.youtube.com/vi/wDnEj3KGwQM/0.jpg" 
alt="TUM RGBD Dataset - Wrong Init" width="240" height="180" border="10" /></a>


### Related Publications:

[1] Hayyan Afeef Daoud, Aznul Qalid Md Sabri, Chu Kiong Loo and Ali Mohammed Mansoor (2018) **SLAMM: Visual Monocular SLAM with continuous mapping using Multiple Maps**. *PLoS ONE xx(xx): exxxxxxx* https://doi.org/10.1371/journal.pone.0195878.

[2] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics **Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[3] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORBSLAMM is released under a [GPLv3 license](https://github.com/HDaoud/ORBSLAMM/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/HDaoud/ORBSLAMM/blob/master/Dependencies.md).

For a closed-source version of ORBSLAMM for commercial purposes, please contact the authors: hayyan.d (at) gmail (dot) com.

If you use ORBSLAMM in an academic work, please cite:

    @article{daoud2018slamm,
    title={SLAMM: Visual monocular SLAM with continuous mapping using multiple maps.},
    author={Daoud, HA and Md, Sabri AQ and Loo, CK and Mansoor, AM},
    journal={PloS one},
    volume={13},
    number={4},
    pages={e0195878},
    year={2018}
    }

# 2. Prerequisites
Compiling ORBSLAMM is similar to [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). Below are the steps mentioned in the [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) GitHub repository.

We have tested the library in **14.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **Required at least 2.4.3. Tested with OpenCV 2.4.8 and 3.0.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are required by g2o (see below). On Ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## v4l2loopback (Included in Thirdparty folder)
v4l2loopback is a kernel module to create V4L2 loopback devices. We use the V4L2 loopback device to capture Bebop FIFO stream and make it available to the processing code in **mono_Bebop**
This is a modified version of the code in [v4l2loopback GitHub respository](https://github.com/umlaeute/v4l2loopback).

# 3. Building ORBSLAMM library and Datasets/Bebop examples

Clone the repository:
```
git clone https://github.com/HDaoud/ORBSLAMM.git ORBSLAMM
```

`build.sh` is provided to build the *Thirdparty* libraries and *ORBSLAMM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORBSLAMM/SingleRobotScenario
chmod +x build.sh
./build.sh
```

This will create **libORBSLAMM.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **mono_Bebop**, **mono_eth**, **mono_NewCollege**,**mono_AGZ** and **ConvertGTtoQuaternion** in *Examples* folder.

Or for multiple robots scenario
```
cd ORBSLAMM/MultipleRobotsScenario
chmod +x build.sh
./build.sh
```

This will create **libORBSLAMM.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **mono_kitti_dif-Seq**, **mono_Bebop**, **mono_eth** and **mono_NewCollege** in *Examples* folder.


# 4. Monocular Examples

To run an example navigate to the Examples folder and run the following command:
```
./mono_xxx path_to_vocabulary path_to_settings path_to_sequence [0|1]for_Multi_Maps_Usage
```
where: **mono_xxx** is related to the dataset (where xxx is tum or kitti or eth,..etc.). 
**path_to_vocabulary** is the directory of the ORBvoc.txt file extracted after running build.sh in the Vocabulary folder.
**path_to_settings** is a yaml file that contains camera's calibration and the number of features to be tracked settings.
**path_to_sequence** is the directory where the dataset is downloaded and extracted.
**[0|1]** this is to activate SLAMM by using 1 or running a normal mono map ORB-SLAM by using 0

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER 1
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 1
```

## Bebop Drone

1. Open the terminal and Create a virtual v4l2 loopback device:
```
modprobe v4l2loopback
```

2. Open another terminal and Run Bebop Decode Stream: Change PATH_TO_ARSDKBuildUtils to the directory where you downloaded Bebop SDK (refer to SettingUpBebop Folder)
```
cd PATH_TO_ARSDKBuildUtils/Samples/Unix/BebopDroneDecodeStream
make run
```

3. Run v4l2mpg:
```
cd PATH_TO_v4l2Loopback/examples
./rgb4mpeg_to_v4l2 /dev/video2 < /PATH_TO_ARSDKBuildUtils/Samples/Unix/BebopDroneDecodeStream/video_decoded_fifo_RGB
```
where PATH_TO_v4l2Loopback is the v4l2loopback folder inside the Thirdparty folder. and /dev/video2 is the created v4l2 loopback device (it could be /dev/video1. usually /dev/video0 is the webcam)

4. Run ORBSLAMM:
```
./mono_Bebop ../../Vocabulary/ORBvoc.txt ./BebopConf.yaml 2 1
```
where 2 is the V4L_Device_number (can be 1. 0 is usually for the webcam)

# 5. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORBSLAMM library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 6. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallel three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds a new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode, the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

# 7. Single or Multiple Maps Modes
You can change between the *Mono map* and *Multi-Mapping (SLAMM)* modes using the GUI of the map viewer.

### Multi-Mapping
This is the default mode when running the system using 1 at the end of the command. The system runs in parallel four threads: Tracking, Local Mapping, Loop Closing and Multi Mapping. The system localizes the camera, builds new maps when tracking is lost, and merge those maps when loops are closed.

### Mono map
This is the default mode when running the system with 0 at the end of the command. The system runs as SLAM mode in section 6.


