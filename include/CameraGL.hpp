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

// CameraGL.hpp

#ifndef __CAMERAGL_INCLUDE__
#define __CAMERAGL_INCLUDE__

#include <Eigen/Core>
#include <Eigen/Eigen>

#include "utils.hpp"

class CameraGL {
public:

    enum DIRECTION {
        UP, DOWN, LEFT, RIGHT, FORWARD, BACK
    };

    CameraGL(Eigen::Vector3f position, Eigen::Vector3f direction, Eigen::Vector3f vertical = Eigen::Vector3f(0, 1, 0));
    ~CameraGL();

    void update();
    void setProjection(float horizontalFOV, float verticalFOV, float znear, float zfar);
    const Eigen::Matrix4f& getViewProjectionMatrix() const;

    float getHorizontalFOV() const;
    float getVerticalFOV() const;

    /*
            Set an offset between the eye of the camera and its position. 
            Note: Useful to use the camera as a trackball camera with z>0 and x = 0, y = 0.
            Note: coordinates are in local space.
     */
    void setOffsetFromPosition(const Eigen::Vector3f& o);
    const Eigen::Vector3f& getOffsetFromPosition() const;

    void setDirection(const Eigen::Vector3f& direction, const Eigen::Vector3f& vertical = Eigen::Vector3f(0, 1, 0));
    void translate(const Eigen::Vector3f& t);
    void setPosition(const Eigen::Vector3f& p);
    void rotate(const Eigen::Quaternionf& rot);
    void rotate(const Eigen::Matrix3f& m);
    void setRotation(const Eigen::Quaternionf& rot);
    void setRotation(const Eigen::Matrix3f& m);

    const Eigen::Vector3f& getPosition() const;
    const Eigen::Vector3f& getForward() const;
    const Eigen::Vector3f& getRight() const;
    const Eigen::Vector3f& getUp() const;
    const Eigen::Vector3f& getVertical() const;
    float getZNear() const;
    float getZFar() const;

    static const Eigen::Vector3f ORIGINAL_FORWARD;
    static const Eigen::Vector3f ORIGINAL_UP;
    static const Eigen::Vector3f ORIGINAL_RIGHT;

    Eigen::Matrix4f projection_;
private:
    void updateVectors();
    void updateView();
    void updateVPMatrix();

    Eigen::Vector3f offset_;

    Eigen::Vector3f position_;
    Eigen::Vector3f forward_;
    Eigen::Vector3f up_;
    Eigen::Vector3f right_;
    Eigen::Vector3f vertical_;

    Eigen::Quaternionf rotation_;

    Eigen::Matrix4f view_;
    Eigen::Matrix4f vpMatrix_;
    float horizontalFieldOfView_;
    float verticalFieldOfView_;
    float znear_;
    float zfar_;
};

#endif /* __VIEWER_INCLUDE__ */