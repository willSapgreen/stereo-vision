#include "planeestimation.h"

using namespace std;

PlaneEstimation::PlaneEstimation() {
  A = Matrix(3,3);
  b = Matrix(3,1);
  plane_d = Matrix(3,1);
  plane_e = Matrix(3,1);
  H = Matrix(4,4);
  pitch = 0;
}

void PlaneEstimation::computeTransformationFromDisparityMap(float* D,int32_t width,int32_t height,int32_t step,float f_,float cu_,float cv_,float base_) {

  // copy intrinsics
  f    = f_;
  cu   = cu_;
  cv   = cv_;
  base = base_;

  plane_d.zero();
  plane_e.zero();
  H = Matrix::eye(4);

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

  for (int32_t i=0; i<num_samples; i++) {

    // draw random samples and compute plane
    drawRandomPlaneSample(d_list);

    // find inlier
    curr_inlier.clear();
    for (int32_t i=0; i<(int32_t)d_list.size(); i++)
      if (fabs(plane_d.val[0][0]*d_list[i].u+plane_d.val[1][0]*d_list[i].v+plane_d.val[2][0]-d_list[i].d)<d_threshold)
        curr_inlier.push_back(i);

    // is this a better solution? (=more inlier)
    if (curr_inlier.size()>best_inlier.size()) {
      best_inlier = curr_inlier;
    }
  }

  // if enough inlier
  if (best_inlier.size()>3) {

    // reoptimize plane with inliers only
    leastSquarePlane(d_list,best_inlier);

    // compute 3d representation and transformation
    planeDsiTo3d();
  }
}

void PlaneEstimation::planeDsiTo3d() {

  // 3d plane parameters
  FLOAT a = plane_d.val[0][0];
  FLOAT b = plane_d.val[1][0];
  FLOAT c = plane_d.val[2][0];
  plane_e.val[0][0] = a/base;
  plane_e.val[1][0] = b/base;
  plane_e.val[2][0] = (a*cu+b*cv+c)/(f*base);

  // road
  if (fabs(plane_d.val[0][1])>0.1) {

    // r2 (facing bottom) = normal vector
    Matrix r2 = plane_e/plane_e.l2norm();

    // r1 (facing right) is defined by the following constraints:
    // - z=0
    // - r2'*r1=0
    // - norm(r1)=1
    Matrix r1(3,1);
    r1.val[0][0] = +sqrt(r2.val[1][0]*r2.val[1][0]/(r2.val[0][0]*r2.val[0][0]+r2.val[1][0]*r2.val[1][0]));
    r1.val[1][0] = -r1.val[0][0]*r2.val[0][0]/r2.val[1][0];
    r1.val[2][0] = 0;

    // cross product (facing forward)
    Matrix r3 = Matrix::cross(r1,r2);
    pitch = atan2(r3.val[1][0],r3.val[2][0]);

    // create 3d homography
    H.eye();
    H.setMat(r1,0,0);
    H.setMat(r2,0,1);
    H.setMat(r3,0,2);

  // portrait (TODO)
  } else {

    // create 3d homography
    H.eye();


  }
}

vector<PlaneEstimation::disp> PlaneEstimation::sparseDisparityGrid (float* D,int32_t width,int32_t height,int32_t step,int32_t* roi,int32_t step_size) {

  // init list
  vector<disp> d_list;

  // loop through disparity image
  for (int32_t u=max(roi[0],0); u<=min(roi[2],width-1); u+=step_size) {
    for (int32_t v=max(roi[1],0); v<=min(roi[3],height-1); v+=step_size) {
      float d = D[v*step+u];
      if (d>=1)
        d_list.push_back(disp(u,v,d));
    }
  }

  // return list
  return d_list;
}

void PlaneEstimation::drawRandomPlaneSample (vector<PlaneEstimation::disp> &d_list) {

  int32_t num_data = d_list.size();
  vector<int32_t> ind;

  // draw 3 measurements
  int32_t k=0;
  while (ind.size()<3 && k<1000) {

    // draw random measurement
    int32_t curr_ind = rand()%num_data;

    // first observation
    if (ind.size()==0) {

      // simply add
      ind.push_back(curr_ind);

    // second observation
    } else if(ind.size()==1) {

      // check distance to first point
      float diff_u = d_list[curr_ind].u-d_list[ind[0]].u;
      float diff_v = d_list[curr_ind].v-d_list[ind[0]].v;
      if (sqrt(diff_u*diff_u+diff_v*diff_v)>50)
        ind.push_back(curr_ind);

    // third observation
    } else {

      // check distance to line between first and second point
      float vu   = d_list[ind[1]].u-d_list[ind[0]].u;
      float vv   = d_list[ind[1]].v-d_list[ind[0]].v;
      float norm = sqrt(vu*vu+vv*vv);
      float nu   = +vv/norm;
      float nv   = -vu/norm;
      float ru   = d_list[curr_ind].u-d_list[ind[0]].u;
      float rv   = d_list[curr_ind].v-d_list[ind[0]].v;
      if (fabs(nu*ru+nv*rv)>50)
        ind.push_back(curr_ind);
    }

    k++;
  }

  // return zero plane on error
  if (ind.size()==0) {
    plane_d.zero();
    return;
  }

  // find least squares solution
  leastSquarePlane(d_list,ind);
}

void PlaneEstimation::leastSquarePlane(vector<disp> &d_list,vector<int32_t> &ind) {

  // clear matrices
  A.zero();
  b.zero();

  // find parameters
  for (vector<int32_t>::iterator it=ind.begin(); it!=ind.end(); it++) {
    float u = d_list[*it].u;
    float v = d_list[*it].v;
    float d = d_list[*it].d;
    A.val[0][0] += u*u;
    A.val[0][1] += u*v;
    A.val[0][2] += u;
    A.val[1][1] += v*v;
    A.val[1][2] += v;
    A.val[2][2] += 1;
    b.val[0][0] += u*d;
    b.val[1][0] += v*d;
    b.val[2][0] += d;
  }
  A.val[1][0] = A.val[0][1];
  A.val[2][0] = A.val[0][2];
  A.val[2][1] = A.val[1][2];

  if (b.solve(A)) plane_d = b;
  else            plane_d.zero();
}
