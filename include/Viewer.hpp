#ifndef __VIEWER_INCLUDE__
#define __VIEWER_INCLUDE__

#include <iostream>
#include <thread>

#include <zed/utils/GlobalDefine.hpp>
#include <GL/glew.h>
#include <GL/glut.h>   /* OpenGL Utility Toolkit header */
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <GL/freeglut.h>

#include "Shader.hpp"
#include "CameraGL.hpp"
#include "Simple3DObject.hpp"
#include "PointCloud.hpp"

#define MOUSE_R_SENSITIVITY 0.015f
#define MOUSE_UZ_SENSITIVITY 0.75f
#define MOUSE_DZ_SENSITIVITY 1.25f
#define MOUSE_T_SENSITIVITY 0.1f
#define KEY_T_SENSITIVITY 0.1f


// This class manages the window, input events and Opengl rendering pipeline

class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Viewer(PointCloud& pointCloud, int argc, char **argv);
    ~Viewer();
    void destroy();
    bool isEnded();
    void setPose(Eigen::Matrix4f& pose, sl::zed::TRACKING_STATE trackingState, int trackingError);
    bool isInitialized();
private:
    void initialize();
    void render();
    void update();
    void draw();
    void printText();
    void clearInputs();

    static Viewer* currentInstance_;

    // OpenGL functions callbacks
    static void drawCallback();
    static void mouseButtonCallback(int button, int state, int x, int y);
    static void mouseMotionCallback(int x, int y);
    static void reshapeCallback(int width, int height);
    static void keyPressedCallback(unsigned char c, int x, int y);
    static void keyReleasedCallback(unsigned char c, int x, int y);
    static void idleCallback();

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

    Simple3DObject* axis_X;
    Simple3DObject* axis_Y;
    Simple3DObject* axis_Z;

    Simple3DObject* path_;
    Simple3DObject* frustum_;
    PointCloud& pointCloud_;
    CameraGL* camera_;
    Shader* shader_;
    GLuint shMVPMatrixLoc_;

    float cr;
    float cg;
    float cb;
    int wnd_w;
    int wnd_h;
    sl::zed::TRACKING_STATE trackState;
    int trackError;
    bool new_path;
    Eigen::Matrix4f path, previous_path, pose;
    std::mutex cam_mtx;
    int _argc;
    char **_argv;
    bool initialized_;
};

#endif
