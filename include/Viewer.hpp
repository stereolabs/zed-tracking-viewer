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

// Viewer.hpp

#ifndef __VIEWER_INCLUDE__
#define __VIEWER_INCLUDE__

#include <iostream>
#include <thread>

#include <zed/utils/GlobalDefine.hpp>
#include <GL/glew.h>
#include <GL/glut.h>   /* OpenGL Utility Toolkit header */
#include <Eigen/Core>
#include <Eigen/Eigen>

#include "utils.hpp"
#include "Shader.hpp"
#include "CameraGL.hpp"
#include "Simple3DObject.hpp"
#include "PointCloud.hpp"

#define MOUSE_R_SENSITIVITY 0.015f
#define MOUSE_UZ_SENSITIVITY 0.75f
#define MOUSE_DZ_SENSITIVITY 1.25f
#define MOUSE_T_SENSITIVITY 0.1f
#define KEY_T_SENSITIVITY 0.1f

/*
 * This class manages the window, input events and Opengl rendering pipeline
 */
class Viewer {
public:
    Viewer(PointCloud& pointCloud, bool fps_mode = false);
    ~Viewer();
    void destroy();
    bool isEnded();
    void setPose(Eigen::Matrix4f& pose);
    void toggleFPSView();
    bool isInitialized();
private:
    void initialize();
    void render();
    void update();
    void draw();
    void clearInputs();

    static Viewer* currentInstance_;

    //! OpenGL Functions CALLBACKs
    static void drawCallback();
    static void mouseButtonCallback(int button, int state, int x, int y);
    static void mouseMotionCallback(int x, int y);
    static void reshapeCallback(int width, int height);
    static void keyPressedCallback(unsigned char c, int x, int y);
    static void keyReleasedCallback(unsigned char c, int x, int y);
    static void idle();

    std::thread* mainLoopThread_;

    bool ended_;

    enum MOUSE_BUTTON {
        LEFT = 0,
        MIDDLE = 1,
        RIGHT = 2,
        WHEEL_UP = 3,
        WHEEL_DOWN = 4
    };

    enum KEY_STATE {
        UP = 'u',
        DOWN = 'd',
        FREE = 'f'
    };

    bool mouseButton_[3];
    int mouseWheelPosition_;
    int mouseCurrentPosition_[2];
    int mouseMotion_[2];
    int previousMouseMotion_[2];
    KEY_STATE keyStates_[256];

    Simple3DObject* axis_;
    Simple3DObject* path_;
    Simple3DObject* frustum_;
    PointCloud& pointCloud_;
    CameraGL* camera_;
    Shader* shader_;
    GLuint shMVPMatrixLoc_;

    bool new_path;
    Eigen::Matrix4f path, previous_path, pose;
    std::mutex cam_mtx;
    bool fps_mode;

    bool initialized_;
};

#endif /* __VIEWER_INCLUDE__ */