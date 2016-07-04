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

// Simple3DObject.hpp

#ifndef __SIMPLE3DOBJECT_INCLUDE__
#define __SIMPLE3DOBJECT_INCLUDE__

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <GL/glew.h>

#include <vector>

class Simple3DObject {
public:
	Simple3DObject(Eigen::Vector3f position, bool isStatic);
    ~Simple3DObject();

    void addPoint(float x, float y, float z, float r, float g, float b);
    void pushToGPU();
    void clear();

    void setDrawingType(GLenum type);

    void draw();

    void translate(const Eigen::Vector3f& t);
    void setPosition(const Eigen::Vector3f& p);

	void setRT(const Eigen::Matrix4f& mRT);

	void rotate(const Eigen::Quaternionf& rot);
	void rotate(const Eigen::Matrix3f& m);
	void setRotation(const Eigen::Quaternionf& rot);
	void setRotation(const Eigen::Matrix3f& m);
	
	const Eigen::Vector3f& getPosition() const;

    Eigen::Matrix4f getModelMatrix() const;
private:
    std::vector<float> vertices_;
    std::vector<float> colors_;
    std::vector<unsigned int> indices_;

    bool isStatic_;

    GLenum drawingType_;

    GLuint vaoID_;
    /*
            Vertex buffer IDs:
            - [0]: vertices coordinates;
            - [1]: RGB color values;
            - [2]: indices;
     */
    GLuint vboID_[3];

	Eigen::Vector3f position_;
	Eigen::Quaternionf rotation_;

};

#endif	/* __SIMPLE3DOBJECT_INCLUDE__ */