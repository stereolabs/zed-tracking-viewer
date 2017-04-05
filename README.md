### This sample is deprecated and no longer maintained for the ZED SDK >= 2.X, please use [this one instead](https://github.com/stereolabs/zed-examples/tree/master/positional%20tracking)

# Stereolabs ZED - Positional Tracking

This sample shows how to track the motion of the camera in 3D space with six degrees of freedom (6DoF). The code included in this example captures and displays a live point cloud of the scene along with 3D camera path.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Windows 7 64bits or later, Ubuntu 16.04
- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads), [OpenCV 3.1](http://opencv.org/downloads.html))
- [GLEW](http://glew.sourceforge.net/) and [FreeGLUT](http://freeglut.sourceforge.net/) for OpenGL visualization

## Build the program

Download the sample and follow these instructions:

#### Build for Windows

- Create a folder called "build" in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution and change the solution configuration to `Release`
- Build solution

#### Build for Linux

Open a terminal and run the following command to clone and build the sample:

    git clone https://github.com/stereolabs/zed-tracking-viewer.git
    cd zed-tracking-viewer
    mkdir build
    cd build
    cmake ..
    make


## Run the program

- Navigate to the build directory and launch the executable file
- Or open a terminal in the build directory and run the sample :

      ./ZED\ Tracking\ Viewer [path to SVO file]

You can optionally provide an SVO file path (recorded stereo video of the ZED)


## Features

This sample shows how to capture and display the current scene depth and 3D camera movement:
- Left image and depth map are captured and displayed
- Depth is displayed as a 3D point cloud
- 3D camera tracking is displayed as a continuous 3D path

![Tracking viewer](track.gif)

*NOTE:* Camera tracking is based on 3D vision only. Quick and sudden camera movements can be difficult to track if the image is blurry or there is no visual information in the scene. To improve tracking performance, we recommend using the ZED in HD720 mode at 60fps.


### Limitations

- For simplicity's sake, this sample uses GPU->CPU data transfer to retrieve and display the images. For optimal performance, use a direct device to device copy of the ZED SDK GPU buffer to OpenGL buffer, rather than using the CPU as an intermediate.
- To improve performance on Jetson TK1 and TX1, we recommend disabling OpenCV and OpenGL display windows and logging tracking data.
