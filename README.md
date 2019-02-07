# Indoor Navigation
Indoor navigation prototype developed during the practical course Advanced Topics in 3D Computer Vision at TUM (WS 18).


# Regarding the code - **READ THIS**

Depending on the system you run the code on, a runtime error might arise. On OS X there is no error, while it arises on Ubuntu. Even though the description the error gives is unrelated, it is caused by the shrinking of the channels of the png image. Not fully sure on the reason why, but probably due to the png creation on the same OS X machine, the image got 4 channels, while it seems to have already 3 on Ubuntu. This conversion is done at line 41 of FrameDrawer.cc inside ORB_SLAM2_solidMONO/src. So, in case this issue arises, comment this line out and rebuild it.
Also, there are a series of paths that need to be changed. Line 40 of the same file for example, or others in the Monocular directory or build file.


### This repository contains:

- **IMPORTANT**: the directory ORB_SLAM2_solidMONO: this contains src, include and CMakeLists.txt to replace the original files from ORB_SLAM2. Even though only these 3 are in this repository, all the other ORB_SLAM2 files are needed to build it, omitted since unchanged and large in size. What has been changed in src and include are only the System, Viewer and FrameDrawer .cc/.h files, the others are unchanged, added here only for completion of the two directories.

- the directory Monocular containing code and settings for the PennCOSYVIO dataset and the RealSense live data
- the directory Stereo containing code and settings for RealSense live data
- the directory utils containing utility functions and the code to generate the compass data for PennCOSYVIO
- the directory RGB-D containing code and settings for RealSense live data (not updatad)
- the build file CMakeLists.txt
- the arrow image file s_b_arrow_bg.png
- this README.md
