# Running SLAMM with Bebop Drone
Refer to the [ARDrone SDK website](http://developer.parrot.com/docs/SDK3/#go-deeper) to download the SDK3 code and run the example.

Add and Replace the files above with the ones in the **_PATH_TO_ARSDKBuildUtils/Samples/Unix/BebopDroneDecodeStream_**. Once everything is set:

1. Open terminal and Create a virtual v4l2 loopback device (This is included in the Thirdparty folder):
```
modprobe v4l2loopback
```

2. Open another terminal and Run Bebope Decode Stream: Change **_PATH_TO_ARSDKBuildUtils_** to the directory where you downloaded Bebop SDK
```
cd PATH_TO_ARSDKBuildUtils/Samples/Unix/BebopDroneDecodeStream
make run
```

3. Run v4l2mpg:
```
cd PATH_TO_v4l2Loopback/examples
./rgb4mpeg_to_v4l2 /dev/video2 < /PATH_TO_ARSDKBuildUtils/Samples/Unix/BebopDroneDecodeStream/video_decoded_fifo_RGB
```
where **PATH_TO_v4l2Loopback** is the v4l2loopback folder inside the Thirdparty folder. and **/dev/video2** is the created v4l2 loopback device _(it could be /dev/video1. usually /dev/video0 is the webcam)_

4. Run ORBSLAMM:
```
./mono_Bebop ../../Vocabulary/ORBvoc.txt ./BebopConf.yaml 2 1
```
where 2 is the V4L_Device_number _(can be 1. Usually 0 is used for the webcam)_

