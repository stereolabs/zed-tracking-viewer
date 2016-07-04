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


#include "PointCloud.hpp"

#include <nppi.h>

GLchar* POINTCLOUD_VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec4 in_VertexRGBA;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec3 b_color;\n"
        "vec4 decomposeFloat(const in float value)\n"
        "{\n"
        "   uint rgbaInt = floatBitsToUint(value);\n"
        "	uint bIntValue = (rgbaInt / 256U / 256U) % 256U;\n"
        "	uint gIntValue = (rgbaInt / 256U) % 256U;\n"
        "	uint rIntValue = (rgbaInt) % 256U; \n"
        "	return vec4(rIntValue / 255.0f, gIntValue / 255.0f, bIntValue / 255.0f, 1.0); \n"
        "}\n"
        "void main() {\n"
        "   b_color = decomposeFloat(in_VertexRGBA.a).xyz;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_VertexRGBA.xyz, 1);\n"
        "}";

GLchar* POINTCLOUD_FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 1);\n"
        "}";

PointCloud::PointCloud(unsigned int width, unsigned int height, CUcontext ctx) :
hasNewPCL_(false), initialized_(false) {
    width_ = width;
    height_ = height;
    zed_cuda_ctx = ctx;
}

PointCloud::~PointCloud() {
    close();
}

void PointCloud::close() {
    initialized_ = false;

    SAFE_DELETE(shader_);
    matGPU_.deallocate();
    glDeleteBuffers(1, &bufferGLID_);
}

void PointCloud::initialize() {

    cuCtxSetCurrent(zed_cuda_ctx);


    glGenBuffers(1, &bufferGLID_);
    glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
    glBufferData(GL_ARRAY_BUFFER, width_ * height_ * 4 * sizeof (float), 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    cudaError_t err = cudaGraphicsGLRegisterBuffer(&bufferCudaID_, bufferGLID_, cudaGraphicsRegisterFlagsNone);
    if (err != cudaSuccess)
        std::cerr << "ERROR: CUDA - OpenGL Interop Failed with Err Code (" << err << ")" << std::endl;

    shader_ = new Shader(POINTCLOUD_VERTEX_SHADER, POINTCLOUD_FRAGMENT_SHADER);
    shMVPMatrixLoc_ = glGetUniformLocation(shader_->getProgramId(), "u_mvpMatrix");

    matGPU_.data = (sl::uchar*) nppiMalloc_32f_C4(width_, height_, &matGPU_.step);
    matGPU_.setUp(width_, height_, 4, sl::zed::FLOAT, sl::zed::GPU);
    initialized_ = true;
}

void PointCloud::pushNewPC(const sl::zed::Mat& matXYZRGBA) {
    if (initialized_) {
        cuCtxSetCurrent(zed_cuda_ctx);
        cudaError_t err = cudaMemcpy(matGPU_.data, matXYZRGBA.data, matXYZRGBA.getWidthByte() * height_, cudaMemcpyDeviceToDevice);
        if (err != cudaSuccess)
            std::cerr << "ERROR: CUDA - Cuda MemCpy Failed with Err Code (" << err << ")" << std::endl;

        hasNewPCL_ = true;
    }
}

void PointCloud::update() {
    if (hasNewPCL_ && initialized_) {
        cudaError_t err = cudaGraphicsMapResources(1, &bufferCudaID_, 0);
        if (err != cudaSuccess)
            std::cerr << "ERROR: CUDA MapResources Err Code (" << err << ")" << std::endl;

        err = cudaGraphicsResourceGetMappedPointer((void**) &xyzrgbaMappedBuf_, &numBytes_, bufferCudaID_);
        if (err != cudaSuccess)
            std::cerr << "ERROR: CUDA GetMappedPointer Err Code (" << err << ")" << std::endl;

        err = cudaMemcpy(xyzrgbaMappedBuf_, matGPU_.data, numBytes_, cudaMemcpyDeviceToDevice);
        if (err != cudaSuccess)
            std::cerr << "ERROR: CUDA MemCpy Err Code (" << err << ")" << std::endl;

        err = cudaGraphicsUnmapResources(1, &bufferCudaID_, 0);
        if (err != cudaSuccess)
            std::cerr << "ERROR: CUDA UnmapResources Err Code (" << err << ")" << std::endl;

        hasNewPCL_ = false;
    }
}

void PointCloud::draw(const Eigen::Matrix4f& vp) {
    if (initialized_) {
        glUseProgram(shader_->getProgramId());

        glUniformMatrix4fv(shMVPMatrixLoc_, 1, GL_FALSE, vp.data());

        glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glDrawArrays(GL_POINTS, 0, width_ * height_);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }
}

unsigned int PointCloud::getWidth() {
    return width_;
}

unsigned int PointCloud::getHeight() {
    return height_;
}
