#ifndef __POINT_CLOUD_INCLUDE__
#define __POINT_CLOUD_INCLUDE__

#include <mutex>

#include <zed/utils/GlobalDefine.hpp>
#include <zed/Mat.hpp>

#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


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
#endif
