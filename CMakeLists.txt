SET(execName ZED_Tracking_Viewer)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

SET(VERSION_REQ_OCV "3.1")
SET(VERSION_REQ_CUDA "8.0")

IF(WIN32) # Windows
    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
    SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
    SET(OPENCV_DIR $ENV{OPENCV_DIR})

    SET(GLUT_INCLUDE_DIRS $ENV{GLUT_INCLUDE_DIRS})
    SET(GLUT_LIBRARY_DIRS $ENV{GLUT_LIBRARY_DIRS})
    SET(GLUT_LIBRARY $ENV{GLUT_LIBRARY})

    SET(GLEW_INCLUDE_DIRS $ENV{GLEW_INCLUDE_DIRS})
    SET(GLEW_LIBRARY_DIRS $ENV{GLEW_LIBRARY_DIRS})
    SET(GLEW_LIBRARY $ENV{GLEW_LIBRARY})
    SET(EIGEN3_INCLUDE_DIR $ENV{EIGEN_DIR})
ELSE() # Linux
    find_package(ZED REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(Eigen3 REQUIRED)

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
     exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
     if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
	SET(VERSION_REQ_OCV "2.4")
	SET(VERSION_REQ_CUDA "")
link_directories("/usr/lib/aarch64-linux-gnu/tegra") ## on Jetson TX1 64bits, the correct libGL.so is located here (the default one will lead to linking error (undef ref))
     endif()

    add_definitions(-Wno-write-strings -mavx)
    SET(LINUX_LIBRARIES "pthread" "X11")
ENDIF(WIN32)

# Mandatory
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

SET(HEADER_FOLDER include)
FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h")

ADD_EXECUTABLE(${execName} ${SRC_FILES} ${HEADER_FILES})
set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "ZED Tracking Viewer")
add_definitions(-std=c++0x -g -O3)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${execName}
			${ZED_LIBRARIES}
			${OPENGL_LIBRARIES}
			${GLUT_LIBRARY}
			${GLEW_LIBRARY}
			${OpenCV_LIBRARIES}
      ${LINUX_LIBRARIES}
			 ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
 )
