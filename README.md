# VINS-Mobile-Android

### Warning: Potentially harmful code

During tests an anomaly occured where the smartphone has completely crashed and was only recoverable by connecting it to a computer/charger. **Use at your own risk!**

## Background

This Project was carried out as part of my [bachelor thesis (German)](https://epb.bibl.th-koeln.de/frontdoor/index/index/docId/1138). It is the android port of the [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile) project of the [HKUST Aerial Robotics Group](http://uav.ust.hk/).

**Videos:**\
[Outdoor](https://youtu.be/HV1XJ2WdGtQ)\
[Indoor](https://youtu.be/M987SGlABuc)\
[Checkerboard Pattern Printed](https://youtu.be/XEM3I9A3QV0)

## Build

### Dependencies
#### OpenCV
Download the OpenCV Android SDK and exctract it somewhere. 
Edit the {path} in the file 

*VINS_Mobile_AndroidPort\app\src\main\cpp\VINS_Android\CMakeLists.txt*

and 

*VINS_Mobile_AndroidPort\app\libs\VINS-Mobile-master\CMakeLists.txt*

according to the location:
```
SET(OpenCV_DIR {path}/OpenCV-android-sdk/sdk/native/jni)
```


#### Boost
Download [Boost](https://www.boost.org/users/download/) and set the {path} in the file

*VINS_Mobile_AndroidPort\app\libs\VINS-Mobile-master\VINS_ThirdPartyLib\CMakeLists.txt*

according to the location:
```
set(boost_include_dir {path}/boost_1_63_0/)
```
You might have a different boost version. The application was only tested with Version 1.63.0.


#### Ceres & Eigen
Ceres unfortunately uses the ndk-build System instead of cmake for android, so it has to be invoked manually.

**Before you start the build process make sure the right APP_ABI for your device is selected in the Application.mk file**

To start the build process use the console/terminal and go to the directory of the Android.mk and Application.mk file:
```
cd ...\VINS_Mobile_AndroidPort\app\libs\VINS-Mobile-master\VINS_ThirdPartyLib\ceres-solver\jni
```
and call the ndk-build programm located in the Android SDK installation directory:
```
...\Android\sdk\ndk-bundle\ndk-build
```
This may take some time. You can accelerate the process with the -jN parameter with N = Number of threads, though it might lead to errors later on.

As the result there should be generated a "obj/local" directory in the "ceres-solver" folder with one libceres.a file for each selected APP_ABI in it.


### Android Studio

Open the VINS_Mobile_AndroidPort folder as an Android Studio Project. Android Studio will generate all missing files and throw an error like this one:

*"Error:Could not find com.android.tools.build:gradle:3.0.1. ..."*

Choose *"Add Google Maven repository and sync project"* and you should be ready to deploy and run the application on your device.

## Device Parameters

The used intrinsic and extrinsic parameters are calibrated to a Samsung Galaxy S7. To have the programm work properly it is necessary to determine the correct values for your device.

For more details on the calibration procedure see my bachelor thesis.

The parameters are located in the file 

*VINS_Mobile_AndroidPort\app\libs\VINS-Mobile-master\VINS_ios\global_param.cpp*.
