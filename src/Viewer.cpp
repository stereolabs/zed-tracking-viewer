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

Viewer* Viewer::currentInstance_ = nullptr;

Viewer::Viewer(PointCloud& pointCloud, bool fps_mode) :
pointCloud_(pointCloud), initialized_(false) {
    if (currentInstance_ != nullptr) {
        delete currentInstance_;
    }
    this->fps_mode = fps_mode;
    currentInstance_ = this;
    mouseButton_[0] = mouseButton_[1] = mouseButton_[2] = false;
    clearInputs();
    previousMouseMotion_[0] = previousMouseMotion_[1] = 0;
    ended_ = false;
    mainLoopThread_ = new std::thread(&Viewer::initialize, this);
    mainLoopThread_->detach();
}

Viewer::~Viewer() {
    if (currentInstance_ != nullptr)
        destroy();
}

void Viewer::destroy() {
    SAFE_DELETE(shader_);
    SAFE_DELETE( camera_);
    SAFE_DELETE( path_);
    SAFE_DELETE( axis_);
    glutDestroyWindow(0);

    currentInstance_ = nullptr;
}

bool Viewer::isEnded() {
    return ended_;
}

void Viewer::setPose(Eigen::Matrix4f& pose) {
    cam_mtx.lock();
    this->pose = pose;
    this->previous_path = this->path;
    this->path = this->previous_path * this->pose;
    new_path = true;
	
	path_->addPoint(previous_path(0, 3), previous_path(1, 3), previous_path(2, 3), 0, 0.674, 0.925);
	path_->addPoint(path(0, 3), path(1, 3), path(2, 3), 0, 0.674, 0.925);

	if (fps_mode) {
		camera_->translate(pose.block<3, 1>(0, 3));
		camera_->rotate(pose.block<3, 3>(0, 0));
	}
    cam_mtx.unlock();
}

void Viewer::initialize() {
    char *argv[1];
    int argc = 1;
#if _WIN32
    argv[0] = _strdup("ZED View");
#else
    argv[0] = strdup("ZED View");
#endif
    glutInit(&argc, argv);

    glutInitWindowSize(1000, 1000);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("ZED 3D Viewer");

    GLenum err = glewInit();
    if (GLEW_OK != err) {
        std::cout << "ERROR: glewInit failed with error: " << glewGetErrorString(err) << "\n";
    }

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
	axis_ = new Simple3DObject(path_id.block<3, 1>(0, 3), true);
    float vertices[18] = {
        0, 0, 0,
        0, 0, 1,
        0, 0, 0,
        0, 1, 0,
        0, 0, 0,
        1, 0, 0
    };
    float colors[18] = {
        0, 0, 1,
        0, 0, 1,
        0, 1, 0,
        0, 1, 0,
        1, 0, 0,
        1, 0, 0
    };
    for (unsigned int i = 0; i < 6; i++) {
        axis_->addPoint(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2], colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
    }
    axis_->setDrawingType(GL_LINES);
    axis_->pushToGPU();

	path_ = new Simple3DObject(path_id.block<3, 1>(0, 3), 0);
    path_->setDrawingType(GL_LINES);

	frustum_ = new Simple3DObject(path_id.block<3, 1>(0, 3), 1);
    frustum_->setDrawingType(GL_LINES);

    float size_w = 0.04;
    float size_h = 0.022;
    float p_ = -0.075;
	float r = 0.172;
    float g = 0.243;
    float b = 0.313;
    Eigen::Vector3f cam_0, cam_1, cam_2, cam_3, cam_4;
    cam_0 << 0, 0, 0;
    cam_1 << -size_w, size_h, p_;
    cam_2 << size_w, size_h, p_;
    cam_3 << size_w, -size_h, p_;
    cam_4 << -size_w, -size_h, p_;

    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), r, g, b);
    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), r, g, b);

    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), r, g, b);
    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), r, g, b);

    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), r, g, b);
    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), r, g, b);

    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), r, g, b);
    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), r, g, b);

    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), r, g, b);
    frustum_->addPoint(cam_1(0), cam_1(1), cam_1(2), r, g, b);

    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), r, g, b);
    frustum_->addPoint(cam_2(0), cam_2(1), cam_2(2), r, g, b);

    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), r, g, b);
    frustum_->addPoint(cam_3(0), cam_3(1), cam_3(2), r, g, b);

    frustum_->addPoint(cam_0(0), cam_0(1), cam_0(2), r, g, b);
    frustum_->addPoint(cam_4(0), cam_4(1), cam_4(2), r, g, b);

    frustum_->pushToGPU();

    // Map glut function on this class methods
    glutDisplayFunc(Viewer::drawCallback);
    glutMouseFunc(Viewer::mouseButtonCallback);
    glutMotionFunc(Viewer::mouseMotionCallback);
    glutReshapeFunc(Viewer::reshapeCallback);
    glutKeyboardFunc(Viewer::keyPressedCallback);
    glutKeyboardUpFunc(Viewer::keyReleasedCallback);
    glutIdleFunc(Viewer::idle);

	initialized_ = true;

    glutMainLoop();
}

void Viewer::render() {
    if (!ended_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glClearColor(1, 1, 1, 1.0f);
        glLineWidth(2);
        glPointSize(2);
        if (!ended_)
        {
        update();
        draw();
        }
        glutSwapBuffers();
    }
}

void Viewer::toggleFPSView() {
    fps_mode = !fps_mode;
}

bool Viewer::isInitialized() {
	return initialized_;
}

void Viewer::update() {
	if (keyStates_['q'] == KEY_STATE::UP) {
        pointCloud_.close();
        ended_ = true;

        //exit(0);
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
	if (keyStates_['f'] == KEY_STATE::UP) {
		currentInstance_->toggleFPSView();
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
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, Eigen::Matrix4f(vpMatrix * axis_->getModelMatrix()).data());
    axis_->draw();
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vpMatrix.data());
    glLineWidth(1);
    path_->draw();
    glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, Eigen::Matrix4f(vpMatrix * frustum_->getModelMatrix()).data());
    frustum_->draw();
    glLineWidth(2);
    glUseProgram(0);

    // Draw point cloud with its own shader
    pointCloud_.draw(vpMatrix);
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
    glViewport(0, 0, width, height);
    float hfov = currentInstance_->camera_->getHorizontalFOV();
    currentInstance_->camera_->setProjection(hfov, hfov * (float) height / (float) width, currentInstance_->camera_->getZNear(), currentInstance_->camera_->getZFar());
}

void Viewer::keyPressedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void Viewer::keyReleasedCallback(unsigned char c, int x, int y) {
	currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void Viewer::idle() {
    glutPostRedisplay();
}

