# README #

This README would normally document whatever steps are necessary to get your application up and running.

### Performance Result ###

*YouTube Video*

https://youtu.be/FdMsZ2LuRbg

*GPX Result*

* KITTI Road 2011_09_26_drive_0029
![GroundTruth(blue) vs VisualOdom(red)](https://github.com/willSapgreen/stereo-vision/blob/master/result/KITTI_Road_2011_09_26_drive_0029.jpg)

GroundTruth(blue) AND VisualOdom(red)

### What is this repository for? ###

stereo-vision is a tool which contains the following computer vision and machine learning developments,

and will continue integrating more developments.

. stereomapper

  A Qt GUI to read KITTI dataset and visualize the disparity map/visual odometry.
  
  The original resource is from http://www.cvlibs.net/software/libviso/
  
  The version here only contains the playback function but with
  
  the memory leak fix.
  
. libviso2

  A library for computing the six DOF motion of a moving mono/stereo camera.
  
  The original resource is from http://www.cvlibs.net/software/libviso/
  
. libelas

  A library for computing disparity maps from rectified graylevel stereo pairs.

### How do I get set up? ###

. System setup:

  . git clone the repo.
  
  . git submodule update.

. Dependencies:

  . OpenCV 3.1.0
  
  . Qt Creator 4.2.1
  
. Database setup:

  . download the KITTI raw data
  
  http://www.cvlibs.net/datasets/kitti/
  
  . unzip the calibration zip file to "calib" folder.
  
  . unzip the sync zip file to "sync" folder.
  
  . unzip the tracklets zip file to "tracklets" folder.
  
. System executation:

  . Load stereomapper.pro in /stereomapper/
  
  . Run qmake
  
  . Build stereomapper project and run the project
  
  . Click "Scan" button and open the folder contains "calib", "sync" and "tracklets" folders
  
### Contribution guidelines ###

### Who do I talk to? ###

. Andreas Geiger ( geiger@kit.edu )
