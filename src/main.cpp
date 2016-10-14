///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////




/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth map with the ZED SDK                    **
 ** and apply the result in a 3D view "point cloud style" with OpenGL /freeGLUT                   **
 ** Some of the functions of the ZED SDK are linked with a key press event		          **
 ***************************************************************************************************/


//standard Include
#include <stdio.h>
#include <string.h>
#include <chrono>

//ZED Include
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

//our point cloud generator and viewer.
#include "Viewer.hpp"
#include "PointCloud.hpp"
//#include "CloudViewer.hpp"

#include <opencv2/opencv.hpp>

using namespace sl::zed;
using namespace std;

#define OCV_DISPLAY 1
#define FPS_MODE 0

//main Loop

int main(int argc, char **argv) {

    std::string reloc_db_path = "";

    if (argc > 2) {
        std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
        return -1;
    }

    Camera* zed;

    if (argc == 1) // Use in Live Mode
        zed = new Camera(HD720);
    else // Use in SVO playback mode
        zed = new Camera(argv[1]);

    InitParams parameters;
    parameters.mode = MODE::PERFORMANCE;
    parameters.unit = UNIT::METER; // the openGl part is scaled for processing METERs
    parameters.verbose = true;
    parameters.coordinate = COORDINATE_SYSTEM::RIGHT_HANDED | COORDINATE_SYSTEM::APPLY_PATH; // OpenGL works in RIGHT_HAND coordinate system, the path is directly apply to the point cloud
    ERRCODE err = zed->init(parameters);

    // ERRCODE display
    cout << errcode2str(err) << endl;

    // Quit if an error occurred
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    Mat bufferXYZRGBA;

    PointCloud cloud(width, height, zed->getCUDAContext());
    Viewer viewer(cloud, FPS_MODE);

    sl::zed::TRACKING_STATE track_state;
    Eigen::Matrix4f pose;
    pose.setIdentity(4, 4);

    zed->enableTracking(pose, true, reloc_db_path);

    // OpenCV display init
#if OCV_DISPLAY
    cv::Mat SbS(height * 2, width, CV_8UC4);
    cv::Mat h_leftRGBA_fullsize(SbS, cv::Rect(0, 0, width, height));
    cv::Mat displayDepth(SbS, cv::Rect(0, height, width, height));
    cv::Size displaySize(720, 404 * 2);
    cv::Mat SbS_display(displaySize.height, displaySize.width, CV_8UC4);
    int waitKeyTime = 10;
#endif

    while (!viewer.isInitialized());

    while (!viewer.isEnded()) {
        // Get frames and launch the computation
        if (!zed->grab(SENSING_MODE::STANDARD, 1, 1, 1)) {
            bufferXYZRGBA = zed->retrieveMeasure_gpu(MEASURE::XYZRGBA);
            if (cloud.mutexData.try_lock()) {
                cloud.pushNewPC(bufferXYZRGBA);
                cloud.mutexData.unlock();
            }

#if OCV_DISPLAY
            slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(h_leftRGBA_fullsize);
            slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY, -40, -5)).copyTo(displayDepth);
#endif

            track_state = zed->getPosition(pose, MAT_TRACKING_TYPE::POSE);

            switch (track_state) {
                case TRACKING_GOOD:
                {
#if OCV_DISPLAY
                    cv::putText(SbS, "Tracking good (" + to_string(zed->getTrackingConfidence()) + ")", cv::Point2d(30, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);
#endif
                    viewer.setPose(pose);
                }
                    break;
                default:
#if OCV_DISPLAY
                    cv::putText(SbS, tracking_state2str(track_state), cv::Point2d(30, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);
#endif
                    break;
            }

#if OCV_DISPLAY
            cv::resize(SbS, SbS_display, displaySize);
            cv::imshow("ZED Images", SbS_display);
#endif
        }

#if OCV_DISPLAY
        cv::waitKey(waitKeyTime);
#endif
    };

    delete zed;

    viewer.destroy();

    return 0;
}
