#ifndef PLANEESTIMATION_H
#define PLANEESTIMATION_H

#include <vector>
#include <stdint.h>
#include <math.h>
#include "../libviso2/src/matrix.h"

class PlaneEstimation {

public:

    PlaneEstimation();
    void computeTransformationFromDisparityMap(float* D,int32_t width,int32_t height,int32_t step,float f_,float cu_,float cv_,float base_);
    Matrix getTransformation() { return H; }
    Matrix getPlaneDsi() { return plane_d; }
    Matrix getPlaneEuclidean() { return plane_e; }
    float  getPitch() { return pitch; }

private:

    struct disp {
      float u,v,d;
      disp(float u,float v,float d) : u(u),v(v),d(d) {}
    };

    std::vector<disp> sparseDisparityGrid (float* D,int32_t width,int32_t height,int32_t step,int32_t* roi,int32_t step_size);
    void drawRandomPlaneSample (std::vector<PlaneEstimation::disp> &d_list);
    void leastSquarePlane(std::vector<disp> &d_list,std::vector<int32_t> &ind);
    void planeDsiTo3d();

    float  f,cu,cv,base;

    Matrix A;
    Matrix b;
    Matrix plane_d;
    Matrix plane_e;
    Matrix H;
    float  pitch;
};

#endif // PLANEESTIMATION_H
