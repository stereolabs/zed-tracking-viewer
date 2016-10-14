# zed-tracking-viewer

**This sample is designed to work with the ZED stereo camera only and requires the ZED SDK. For more information: https://www.stereolabs.com**

This sample demonstrates how to extract Left, Right images, print the 3D point cloud in OpenGL, and show the camera path on the OpenGL windows
This sample shows a good overview of Depth and Tracking functions.

**Warning:**
 - GPU->CPU readback is time consuming
 - This sample is not designed to perform efficiently on Jetson (many windows, OpenCV, ...).

Images are displayed with OpenCV and 3D Point Cloud is displayed with OpenGL

This sample needs OpenCV, GLEW, FreeGLUT (given as dependencies under windows)

## Build the program

#### Build the program (Linux)

Open a terminal in With OpenCV directory and execute the following command:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

#### Build the program (Windows)

  - Open cmake-gui and select both locations for source (CMakeLists folder) and build (create a build folder)
  - Configure then generate in the build directory,
  - Open the VS project change the build mode to release.
  - Build (Ctrl + Shift + B)

## Run the program

Open a terminal in build directory and execute the following command:

    $ ./ZED\ Tracking\ Viewer
