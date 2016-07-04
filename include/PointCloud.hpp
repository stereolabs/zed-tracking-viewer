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

// PointCloud.hpp
#ifndef __POINT_CLOUD_INCLUDE__
#define __POINT_CLOUD_INCLUDE__

#include <zed/utils/GlobalDefine.hpp>
#include <zed/Mat.hpp>

#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <mutex>

#include "Shader.hpp"


#define SAFE_DELETE( res ) if( res!=NULL )  \
                                {                \
                                delete res;  \
                                res = NULL;  \
                                }

class PointCloud {
public:
    PointCloud(unsigned int width, unsigned int height, CUcontext ctx);
    ~PointCloud();
    void initialize();
    void pushNewPC(const sl::zed::Mat& matXYZRGBA);
    void update();
    void draw(const Eigen::Matrix4f& vp);
    void close();

    unsigned int getWidth();
    unsigned int getHeight();
    std::mutex mutexData;
    CUcontext zed_cuda_ctx;

private:
    unsigned int width_;
    unsigned int height_;

    sl::zed::Mat matGPU_;
    bool hasNewPCL_;
    bool initialized_;
    Shader* shader_;
    GLuint shMVPMatrixLoc_;
    size_t numBytes_;
    float* xyzrgbaMappedBuf_;
    GLuint bufferGLID_;
    cudaGraphicsResource* bufferCudaID_;
};
#endif /* __POINT_CLOUD_INCLUDE__ */
