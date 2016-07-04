SET(execName ZED_Tracking_Viewer)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

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

    find_package(OpenCV 3.1 REQUIRED)
ELSE() # Linux
    find_package(ZED 1.0 REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(Eigen3 REQUIRED)

    ##For Jetson, OpenCV4Tegra is based on OpenCV2.4
    exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
    if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
       find_package(OpenCV 2.4 REQUIRED)
    else()
       find_package(OpenCV 3.1 REQUIRED)
    endif()

    add_definitions(-Wno-write-strings)
    SET(LINUX_LIBRARIES "pthread")
ENDIF(WIN32)

# Mandatory
find_package(CUDA REQUIRED)
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
                        ${LINUX_LIBRARIES}
			${OpenCV_LIBRARIES}
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
 )
