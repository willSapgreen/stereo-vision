#include "planeestimation.h"

using namespace std;

PlaneEstimation::PlaneEstimation()
{
    _A = Matrix(3,3);
    _b = Matrix(3,1);
    _plane_d = Matrix(3,1);
    _plane_e = Matrix(3,1);
    _H = Matrix(4,4);
    _pitch = 0;
}

//==============================================================================//

void PlaneEstimation::computeTransformationFromDisparityMap(float* D,int32_t width,
                                                            int32_t height,int32_t step,
                                                            float f,float cu,float cv,float base)
{
    // copy intrinsics
    _f    = f;
    _cu   = cu;
    _cv   = cv;
    _base = base;

    _plane_d.zero();
    _plane_e.zero();
    _H = Matrix::eye(4);

    // region of interest
    int32_t roi[4] = {0,height/3,width-1,height-1};
    int32_t num_samples = 5000;
    FLOAT d_threshold = 5;

    // random seed
    srand( time(NULL));

    // get list with disparities
    vector<disp> d_list = sparseDisparityGrid(D,width,height,step,roi,5);

    // loop variables
    vector<int32_t> curr_inlier;
    vector<int32_t> best_inlier;

    for (int32_t i=0; i<num_samples; i++)
    {
        // draw random samples and compute plane
        drawRandomPlaneSample(d_list);

        // find inlier
        curr_inlier.clear();
        for (int32_t i=0; i<(int32_t)d_list.size(); i++)
        {
            float result = _plane_d._val[0][0] * d_list[i].u +
                           _plane_d._val[1][0] * d_list[i].v +
                           _plane_d._val[2][0] - d_list[i].d;
            if (fabs(result) < d_threshold)
            {
                curr_inlier.push_back(i);
            }
        }

        // is this a better solution? (=more inlier)
        if (curr_inlier.size()>best_inlier.size())
        {
            best_inlier = curr_inlier;
        }
    }

    // if enough inlier
    if (best_inlier.size()>3)
    {
        // reoptimize plane with inliers only
        leastSquarePlane(d_list,best_inlier);

        // compute 3d representation and transformation
        planeDsiTo3d();
    }
}

//==============================================================================//

void PlaneEstimation::planeDsiTo3d()
{
    // 3d plane parameters
    FLOAT a = _plane_d._val[0][0];
    FLOAT b = _plane_d._val[1][0];
    FLOAT c = _plane_d._val[2][0];
    _plane_e._val[0][0] = a/_base;
    _plane_e._val[1][0] = b/_base;
    _plane_e._val[2][0] = (a*_cu+b*_cv+c)/(_f*_base);

    // road
    if (fabs(_plane_d._val[0][1])>0.1)
    {
        // r2 (facing bottom) = normal vector
        Matrix r2 = _plane_e/_plane_e.l2norm();

        // r1 (facing right) is defined by the following constraints:
        // - z=0
        // - r2'*r1=0
        // - norm(r1)=1
        Matrix r1(3,1);
        r1._val[0][0] = +sqrt(r2._val[1][0]*r2._val[1][0]/(r2._val[0][0]*r2._val[0][0]+r2._val[1][0]*r2._val[1][0]));
        r1._val[1][0] = -r1._val[0][0]*r2._val[0][0]/r2._val[1][0];
        r1._val[2][0] = 0;

        // cross product (facing forward)
        Matrix r3 = Matrix::cross(r1,r2);
        _pitch = atan2(r3._val[1][0],r3._val[2][0]);

        // create 3d homography
        _H.eye();
        _H.setMat(r1,0,0);
        _H.setMat(r2,0,1);
        _H.setMat(r3,0,2);

        // portrait (TODO)
    }
    else
    {
        // create 3d homography
        _H.eye();
    }
}

//==============================================================================//

vector<PlaneEstimation::disp> PlaneEstimation::sparseDisparityGrid (float* D,int32_t width,
                                                                    int32_t height,int32_t step,
                                                                    int32_t* roi,int32_t step_size)
{
    // init list
    vector<disp> d_list;

    // loop through disparity image
    for (int32_t u=max(roi[0],0); u<=min(roi[2],width-1); u+=step_size)
    {
        for (int32_t v=max(roi[1],0); v<=min(roi[3],height-1); v+=step_size)
        {
          float d = D[v*step+u];
          if (d>=1)
          {
            d_list.push_back(disp(u,v,d));
          }
        }
    }

    // return list
    return d_list;
}

//==============================================================================//

void PlaneEstimation::drawRandomPlaneSample (vector<PlaneEstimation::disp> &d_list)
{
    int32_t num_data = d_list.size();
    vector<int32_t> ind;

    // draw 3 measurements
    int32_t k=0;
    while (ind.size()<3 && k<1000)
    {
        // draw random measurement
        int32_t curr_ind = rand()%num_data;

        // first observation
        if (ind.size()==0)
        {

            // simply add
            ind.push_back(curr_ind);


        }
        // second observation
        else if(ind.size() == 1)
        {
            // check distance to first point
            float diff_u = d_list[curr_ind].u - d_list[ind[0]].u;
            float diff_v = d_list[curr_ind].v - d_list[ind[0]].v;
            if (sqrt(diff_u * diff_u + diff_v * diff_v)>50)
            {
                ind.push_back(curr_ind);
            }

        }
        // third observation
        else
        {
            // check distance to line between first and second point
            float vu   = d_list[ind[1]].u - d_list[ind[0]].u;
            float vv   = d_list[ind[1]].v - d_list[ind[0]].v;
            float norm = sqrt(vu*vu+vv*vv);
            float nu   = +vv/norm;
            float nv   = -vu/norm;
            float ru   = d_list[curr_ind].u - d_list[ind[0]].u;
            float rv   = d_list[curr_ind].v - d_list[ind[0]].v;
            if (fabs(nu * ru + nv * rv)>50)
            {
                ind.push_back(curr_ind);
            }
        }

        k++;
    }

    // return zero plane on error
    if (ind.size() == 0)
    {
        _plane_d.zero();
        return;
    }

    // find least squares solution
    leastSquarePlane(d_list,ind);
}

//==============================================================================//

void PlaneEstimation::leastSquarePlane(vector<disp> &d_list,vector<int32_t> &ind)
{
    // clear matrices
    _A.zero();
    _b.zero();

    // find parameters
    for (vector<int32_t>::iterator it=ind.begin(); it!=ind.end(); it++)
    {
        float u = d_list[*it].u;
        float v = d_list[*it].v;
        float d = d_list[*it].d;
        _A._val[0][0] += u*u;
        _A._val[0][1] += u*v;
        _A._val[0][2] += u;
        _A._val[1][1] += v*v;
        _A._val[1][2] += v;
        _A._val[2][2] += 1;
        _b._val[0][0] += u*d;
        _b._val[1][0] += v*d;
        _b._val[2][0] += d;
    }
    _A._val[1][0] = _A._val[0][1];
    _A._val[2][0] = _A._val[0][2];
    _A._val[2][1] = _A._val[1][2];

    if (_b.solve(_A))
    {
        _plane_d = _b;
    }
    else
    {
        _plane_d.zero();
    }
}
