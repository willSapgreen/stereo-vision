#ifndef PLANEESTIMATION_H
#define PLANEESTIMATION_H

#include <vector>
#include <stdint.h>
#include <math.h>
#include "../libviso2/src/matrix.h"

class PlaneEstimation {

public:

    PlaneEstimation();
    void computeTransformationFromDisparityMap(float* D,int32_t width,int32_t height,int32_t
                                               step,float f,float cu,float cv,float base);
    Matrix getTransformation() { return _H; }
    Matrix getPlaneDsi() { return _plane_d; }
    Matrix getPlaneEuclidean() { return _plane_e; }
    float  getPitch() { return _pitch; }

private:

    struct disp
    {
        float u;
        float v;
        float d;
        disp(float u,float v,float d) : u(u),v(v),d(d) {}
    };

    std::vector<disp> sparseDisparityGrid (float* D,int32_t width,int32_t height,int32_t step,int32_t* roi,int32_t step_size);
    void drawRandomPlaneSample (std::vector<PlaneEstimation::disp> &d_list);
    void leastSquarePlane(std::vector<disp> &d_list,std::vector<int32_t> &ind);
    void planeDsiTo3d();

    float  _f;
    float _cu;
    float _cv;
    float _base;

    Matrix _A;
    Matrix _b;
    Matrix _plane_d;
    Matrix _plane_e;
    Matrix _H;
    float  _pitch;
};

#endif // PLANEESTIMATION_H
