#include "CameraGL.hpp"
#include <iostream>

const Eigen::Vector3f CameraGL::ORIGINAL_FORWARD = Eigen::Vector3f(0, 0, 1);
const Eigen::Vector3f CameraGL::ORIGINAL_UP = Eigen::Vector3f(0, 1, 0);
const Eigen::Vector3f CameraGL::ORIGINAL_RIGHT = Eigen::Vector3f(1, 0, 0);

CameraGL::CameraGL(Eigen::Vector3f position, Eigen::Vector3f direction, Eigen::Vector3f vertical) {
    this->position_ = position;
    setDirection(direction);

    offset_ = Eigen::Vector3f(0, 0, 0);
    view_.setIdentity();
    updateView();
    setProjection(60, 60, 0.01f, 100.f);
    updateVPMatrix();
}

CameraGL::~CameraGL() {

}

void CameraGL::update() {
    if (vertical_.dot(up_) < 0) {
        vertical_ = -vertical_;
    }
    updateView();
    updateVPMatrix();
}

void CameraGL::setProjection(float horizontalFOV, float verticalFOV, float znear, float zfar) {
    horizontalFieldOfView_ = horizontalFOV;
    verticalFieldOfView_ = verticalFOV;
    znear_ = znear;
    zfar_ = zfar;

    float fov_y = verticalFOV * M_PI / 180.f;
    float fov_x = horizontalFOV * M_PI / 180.f;

    projection_.setIdentity();
    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f);
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f);
    projection_(2, 2) = -(zfar + znear) / (zfar - znear);
    projection_(3, 2) = -1;
    projection_(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    projection_(3, 3) = 0;
}

const Eigen::Matrix4f& CameraGL::getViewProjectionMatrix() const {
    return vpMatrix_;
}

float CameraGL::getHorizontalFOV() const {
    return horizontalFieldOfView_;
}

float CameraGL::getVerticalFOV() const {
    return verticalFieldOfView_;
}

void CameraGL::setOffsetFromPosition(const Eigen::Vector3f& o) {
    offset_ = o;
}

const Eigen::Vector3f& CameraGL::getOffsetFromPosition() const {
    return offset_;
}

void CameraGL::setDirection(const Eigen::Vector3f& direction, const Eigen::Vector3f& vertical) {
    Eigen::Vector3f dirNormalized = direction;
    dirNormalized.normalize();
    this->rotation_ = Eigen::Quaternionf::FromTwoVectors(ORIGINAL_FORWARD, -dirNormalized);

    updateVectors();

    this->vertical_ = vertical;
    if (vertical_.dot(up_) < 0) {
        rotate(Eigen::Quaternionf(Eigen::AngleAxisf(M_PI, ORIGINAL_FORWARD)));
    }
}

void CameraGL::translate(const Eigen::Vector3f& t) {
    position_ += t;
}

void CameraGL::setPosition(const Eigen::Vector3f& p) {
    position_ = p;
}

void CameraGL::rotate(const Eigen::Quaternionf& rot) {
    rotation_ = rot * rotation_;
    updateVectors();
}

void CameraGL::rotate(const Eigen::Matrix3f& m) {
    this->rotate(Eigen::Quaternionf(m));
}

void CameraGL::setRotation(const Eigen::Quaternionf& rot) {
    rotation_ = rot;
    updateVectors();
}

void CameraGL::setRotation(const Eigen::Matrix3f& m) {
    this->setRotation(Eigen::Quaternionf(m));
}

const Eigen::Vector3f& CameraGL::getPosition() const {
    return position_;
}

const Eigen::Vector3f& CameraGL::getForward() const {
    return forward_;
}

const Eigen::Vector3f& CameraGL::getRight() const {
    return right_;
}

const Eigen::Vector3f& CameraGL::getUp() const {
    return up_;
}

const Eigen::Vector3f& CameraGL::getVertical() const {
    return vertical_;
}

float CameraGL::getZNear() const {
    return znear_;
}

float CameraGL::getZFar() const {
    return zfar_;
}

void CameraGL::updateVectors() {
    forward_ = rotation_ * ORIGINAL_FORWARD;
    up_ = rotation_ * ORIGINAL_UP;
    right_ = rotation_ * -ORIGINAL_RIGHT;
}

void CameraGL::updateView() {
    Eigen::Matrix4f transformation;
    transformation << rotation_.toRotationMatrix(), rotation_ * offset_ + position_, 0, 0, 0, 1;
    view_ = transformation.inverse();
}

void CameraGL::updateVPMatrix() {
    vpMatrix_ = projection_ * view_;
}