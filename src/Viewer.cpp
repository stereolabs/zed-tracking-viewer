#include "Viewer.hpp"

GLchar* VERTEX_SHADER =
"#version 330 core\n"
"layout(location = 0) in vec3 in_Vertex;\n"
"layout(location = 1) in vec3 in_Color;\n"
"uniform mat4 u_mvpMatrix;\n"
"out vec3 b_color;\n"
"void main() {\n"
"   b_color = in_Color;\n"
"	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
"}";

GLchar* FRAGMENT_SHADER =
"#version 330 core\n"
"in vec3 b_color;\n"
"layout(location = 0) out vec4 out_Color;\n"
"void main() {\n"
"   out_Color = vec4(b_color, 1);\n"
"}";

static void safe_glutBitmapString(void *font, const char *str) {
    for (size_t x = 0; x < strlen(str); ++x)
        glutBitmapCharacter(font, str[x]);
}

void getColor(int num_segments, int i, float &c1, float &c2, float &c3) {
    float r = fabs(1. - (float(i)*2.) / float(num_segments));
    c1 = (0.1 * r);
    c2 = (0.3 * r);
    c3 = (0.8 * r);
}

Viewer* Viewer::currentInstance_ = nullptr;

Viewer::Viewer(PointCloud& pointCloud, int argc, char **argv) :
    pointCloud_(pointCloud), initialized_(false) {
    if (currentInstance_ != nullptr) {
        delete currentInstance_;
    }
    wnd_w = 1000;
    wnd_h = 1000;
    currentInstance_ = this;
    mouseButton_[0] = mouseButton_[1] = mouseButton_[2] = false;
    clearInputs();
    previousMouseMotion_[0] = previousMouseMotion_[1] = 0;
    ended_ = false;
    _argc = argc;
    _argv = argv;

    cb = 0.847058f;
    cg = 0.596078f;
    cr = 0.203921f;

    mainLoopThread_ = new std::thread(&Viewer::initialize, this);
}

Viewer::~Viewer() {
    if (currentInstance_ != nullptr)
        destroy();
}

void Viewer::destroy() {
    mainLoopThread_->join();
    SAFE_DELETE(shader_);
    SAFE_DELETE(camera_);
    SAFE_DELETE(path_);
    SAFE_DELETE(axis_X);
    SAFE_DELETE(axis_Y);
    SAFE_DELETE(axis_Z);
    pointCloud_.close();
    SAFE_DELETE(mainLoopThread_);
    currentInstance_ = nullptr;
}

bool Viewer::isEnded() {
    return ended_;
}

void Viewer::setPose(Eigen::Matrix4f& pose, sl::zed::TRACKING_STATE track_state, int trackingError) {
    cam_mtx.lock();
    this->pose = pose;
    this->previous_path = this->path;
    this->path = this->previous_path * this->pose;
    new_path = true;

    trackState = track_state;
    trackError = trackingError;

    path_->addPoint(previous_path(0, 3), previous_path(1, 3), previous_path(2, 3), cr, cg, cb);
    path_->addPoint(path(0, 3), path(1, 3), path(2, 3), cr, cg, cb);

    cam_mtx.unlock();
}

void Viewer::initialize() {
    glutInit(&_argc, _argv);
    glutInitWindowSize(wnd_w, wnd_h);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("ZED 3D Viewer");

    GLenum err = glewInit();
    if (GLEW_OK != err)
        std::cout << "ERROR: glewInit failed with error: " << glewGetErrorString(err) << "\n";

    glEnable(GL_DEPTH_TEST);

    // Initialize OpenGL objects of the Point Cloud
    pointCloud_.initialize();

    // Compile and create the shader
    shader_ = new Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    shMVPMatrixLoc_ = glGetUniformLocation(shader_->getProgramId(), "u_mvpMatrix");

    // Create the camera
    camera_ = new CameraGL(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, -1));
    camera_->setOffsetFromPosition(Eigen::Vector3f(0, 0, 4));

    new_path = false;
    path.setIdentity(4, 4);

    // Create 3D axis
    Eigen::Matrix4f path_id;
    path_id.setIdentity();
    axis_X = new Simple3DObject(path_id.block<3, 1>(0, 3), true);
    axis_Y = new Simple3DObject(path_id.block<3, 1>(0, 3), true);
    axis_Z = new Simple3DObject(path_id.block<3, 1>(0, 3), true);

    int num_segments = 60;
    float rad = 0.10;
    float fade = 0.5f;

    for (int ii = 0; ii < num_segments; ii++) {
        float c1 = (cr * fade);
        float c2 = (cg * fade);
        float c3 = (cb * fade);

        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
        axis_X->addPoint(rad * cosf(theta), rad * sinf(theta), 0, c1, c2, c3);

        getColor(num_segments, ii, c1, c2, c3);
        axis_Y->addPoint(0, rad * sinf(theta), rad * cosf(theta), c3, c2, c2);

        theta = 2.0f * M_PI * (float(ii) + float(num_segments) / 4.) / float(num_segments);
        theta = theta > (2. * M_PI) ? theta - (2. * M_PI) : theta;
        getColor(num_segments, ii, c1, c2, c3);
        axis_Z->addPoint(rad * cosf(theta), 0, rad * sinf(theta), c2, c3, c1);
    }

    axis_X->setDrawingType(GL_LINE_LOOP);
    axis_X->pushToGPU();
    axis_Y->setDrawingType(GL_LINE_LOOP);
    axis_Y->pushToGPU();
    axis_Z->setDrawingType(GL_LINE_LOOP);
    axis_Z->pushToGPU();

    path_ = new Simple3DObject(path_id.block<3, 1>(0, 3), 0);
    path_->setDrawingType(GL_LINES);

    frustum_ = new Simple3DObject(path_id.block<3, 1>(0, 3), 1);
    frustum_->setDrawingType(GL_LINES);

    float size_w = 0.04;
    float size_h = 0.022;
    float p_ = -0.075;
    Eigen::Vector3f cam_0, cam_1, cam_2, cam_3, cam_4;
    cam_0 << 0, 0, 0;
    cam_1 << -size_w, size_h, p_;
    cam_2 << size_w, size_h, p_;
    cam_3 << size_w, -size_h, p_;
    cam_4 << -size_w, -size_h, p_;

    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), cb, cg, cr);
    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), cb, cg, cr);
    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), cb, cg, cr);
    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), cb, cg, cr);
    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), cb, cg, cr);
    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), cb, cg, cr);
    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), cb, cg, cr);
    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), cb, cg, cr);
    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), cb, cg, cr);
    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), cb, cg, cr);
    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), cb, cg, cr);
    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), cb, cg, cr);
    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), cb, cg, cr);
    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), cb, cg, cr);
    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), cb, cg, cr);
    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), cb, cg, cr);

    frustum_->pushToGPU();

    // Map glut function on this class methods
    glutDisplayFunc(Viewer::drawCallback);
    glutMouseFunc(Viewer::mouseButtonCallback);
    glutMotionFunc(Viewer::mouseMotionCallback);
    glutReshapeFunc(Viewer::reshapeCallback);
    glutKeyboardFunc(Viewer::keyPressedCallback);
    glutKeyboardUpFunc(Viewer::keyReleasedCallback);
    glutIdleFunc(Viewer::idleCallback);

    initialized_ = true;

    glutMainLoop();
}

void Viewer::render() {
    if (!ended_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glClearColor(0.12, 0.12, 0.12, 1.0f);
        glLineWidth(2);
        glPointSize(2);
        if (!ended_) {
            update();
            draw();
            printText();
        }
        glutSwapBuffers();
    }
}

bool Viewer::isInitialized() {
    return initialized_;
}

void Viewer::update() {
    if (keyStates_['q'] == KEY_STATE::UP || keyStates_['Q'] == KEY_STATE::UP || keyStates_[27] == KEY_STATE::UP) {
        ended_ = true;
    }

    if (new_path) {
        cam_mtx.lock();

        new_path = false;

        path_->pushToGPU();

        frustum_->setRT(path);
        frustum_->pushToGPU();

        cam_mtx.unlock();
    }

    // Rotation of the camera
    if (mouseButton_[MOUSE_BUTTON::LEFT]) {
        camera_->rotate(Eigen::Quaternionf(Eigen::AngleAxisf(mouseMotion_[1] * MOUSE_R_SENSITIVITY, camera_->getRight())));
        camera_->rotate(Eigen::Quaternionf(Eigen::AngleAxisf(mouseMotion_[0] * MOUSE_R_SENSITIVITY, -camera_->getVertical())));
    }

    // Translation of the camera on its plane
    if (mouseButton_[MOUSE_BUTTON::RIGHT]) {
        camera_->translate(camera_->getUp() * mouseMotion_[1] * MOUSE_T_SENSITIVITY);
        camera_->translate(camera_->getRight() * mouseMotion_[0] * MOUSE_T_SENSITIVITY);
    }

    // Zoom of the camera
    if (mouseWheelPosition_ != 0) {
        float distance = camera_->getOffsetFromPosition().norm();
        if (mouseWheelPosition_ > 0 && distance > camera_->getZNear()) { // zoom
            camera_->setOffsetFromPosition(camera_->getOffsetFromPosition() * MOUSE_UZ_SENSITIVITY);
        } else if (distance < camera_->getZFar()) {// unzoom
            camera_->setOffsetFromPosition(camera_->getOffsetFromPosition() * MOUSE_DZ_SENSITIVITY);
        }
    }

    // Translation of the camera on its axis
    if (keyStates_['u'] == KEY_STATE::DOWN) {
        camera_->translate(-camera_->getForward() * KEY_T_SENSITIVITY);
    }
    if (keyStates_['j'] == KEY_STATE::DOWN) {
        camera_->translate(camera_->getForward() * KEY_T_SENSITIVITY);
    }
    if (keyStates_['h'] == KEY_STATE::DOWN) {
        camera_->translate(camera_->getRight() * KEY_T_SENSITIVITY);
    }
    if (keyStates_['k'] == KEY_STATE::DOWN) {
        camera_->translate(-camera_->getRight() * KEY_T_SENSITIVITY);
    }

    // Update Point Cloud buffers
    pointCloud_.mutexData.lock();
    pointCloud_.update();
    pointCloud_.mutexData.unlock();

    // Set the axis following the camera center
    camera_->update();

    clearInputs();
}

void Viewer::draw() {
    const Eigen::Matrix4f& vpMatrix = camera_->getViewProjectionMatrix();
    // Simple 3D shader for simple 3D objects
    glUseProgram(shader_->getProgramId());
    // Axis
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, Eigen::Matrix4f(vpMatrix * axis_X->getModelMatrix()).data());
    axis_X->draw();
    axis_Y->draw();
    axis_Z->draw();
    glLineWidth(2);
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vpMatrix.data());
    path_->draw();
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, Eigen::Matrix4f(vpMatrix * frustum_->getModelMatrix()).data());
    frustum_->draw();
    glUseProgram(0);

    // Draw point cloud with its own shader
    pointCloud_.draw(vpMatrix);
}

void Viewer::printText() {
    bool trackIsGood = trackState == sl::zed::TRACKING_GOOD;
    glColor3f(0.92, 0.92, 0.92);
    float start_w = -0.97f;
    glRasterPos2f(start_w, -0.95f);

    std::string str = "Status : ";
    safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, str.c_str());

    if (trackIsGood)
        glColor3f(0.2, 0.85, 0.2);
    else
        glColor3f(0.85, 0.2, 0.2);

    glRasterPos2f((wnd_w *start_w + 120.f) / wnd_w, -0.95f);
    str = sl::zed::tracking_state2str(trackState);
    safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, str.c_str());

    glColor3f(0.92, 0.92, 0.92);
    start_w = (wnd_w - 280.f) / wnd_w;
    glRasterPos2f(start_w, -0.95f);
    str = "Confidence";
    safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, str.c_str());

    if (trackIsGood)
        glColor3f(0.2, 0.85, 0.2);
    else
        glColor3f(0.85, 0.2, 0.2);
    glRasterPos2f((wnd_w *start_w + 200.f) / wnd_w, -0.95f);
    str = trackIsGood ? std::to_string(trackError) : "Inf";
    safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, str.c_str());
}

void Viewer::clearInputs() {
    mouseMotion_[0] = mouseMotion_[1] = 0;
    mouseWheelPosition_ = 0;
    for (unsigned int i = 0; i < 256; ++i)
        if (keyStates_[i] != KEY_STATE::DOWN)
            keyStates_[i] = KEY_STATE::FREE;
}

void Viewer::drawCallback() {
    currentInstance_->render();
}

void Viewer::mouseButtonCallback(int button, int state, int x, int y) {
    if (button < 5) {
        if (button < 3) {
            currentInstance_->mouseButton_[button] = state == GLUT_DOWN;
        } else {
            currentInstance_->mouseWheelPosition_ += button == MOUSE_BUTTON::WHEEL_UP ? 1 : -1;
        }
        currentInstance_->mouseCurrentPosition_[0] = x;
        currentInstance_->mouseCurrentPosition_[1] = y;
        currentInstance_->previousMouseMotion_[0] = x;
        currentInstance_->previousMouseMotion_[1] = y;
    }
}

void Viewer::mouseMotionCallback(int x, int y) {
    currentInstance_->mouseMotion_[0] = x - currentInstance_->previousMouseMotion_[0];
    currentInstance_->mouseMotion_[1] = y - currentInstance_->previousMouseMotion_[1];
    currentInstance_->previousMouseMotion_[0] = x;
    currentInstance_->previousMouseMotion_[1] = y;
    glutPostRedisplay();
}

void Viewer::reshapeCallback(int width, int height) {
    currentInstance_->wnd_w = width;
    currentInstance_->wnd_h = height;
    glViewport(0, 0, width, height);
    float hfov = currentInstance_->camera_->getHorizontalFOV();
    currentInstance_->camera_->setProjection(hfov, hfov * (float)height / (float)width, currentInstance_->camera_->getZNear(), currentInstance_->camera_->getZFar());
}

void Viewer::keyPressedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void Viewer::keyReleasedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void Viewer::idleCallback() {
    if (currentInstance_->ended_)
        glutLeaveMainLoop();
    else
        glutPostRedisplay();
}

