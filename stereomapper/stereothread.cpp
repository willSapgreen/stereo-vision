#include "stereothread.h"
#include "../libviso2/src/timer.h"

using namespace std;

StereoThread::StereoThread(CalibIO *calib,View3D *modelView,QObject *parent) :
    QThread(parent),
    calib(calib),
    modelView(modelView) {

  plane = new PlaneEstimation();
  simg = 0;
  D_color = 0;
  max_dist = 20;
}

StereoThread::~StereoThread() {
  delete plane;
  if (simg!=0) {
    delete simg;
    simg = 0;
  }
  if (D_color!=0)
    free (D_color);
}

void StereoThread::pushBack(StereoImage::simage &s,Matrix H,float gain_) {
  if (simg!=0) {
    delete simg;
    simg = 0;
  }
  simg = new StereoImage::simage(s);
  H_total = H;
  gain = gain_;
  mode = 0;
  subsampling = 0;
}

void StereoThread::pushBack(StereoImage::simage &s,bool subsampling_) {
  if (simg!=0) {
    delete simg;
    simg = 0;
  }
  simg = new StereoImage::simage(s);
  mode = 1;
  subsampling = subsampling_;
}

void StereoThread::run() {

  // get intrinsics from calibration file
  getIntrinsics();

  Timer t;
  t.start("elas");

  Elas::parameters param(Elas::ROBOTICS);
  param.postprocess_only_left = true;
  param.filter_adaptive_mean = true; ///////////////////
  //param.support_threshold = 0.85;
  param.support_texture = 30;
  //param.incon_threshold = 5;
  //param.incon_min_support = 10;
  param.filter_slanted_faces = 1;
  param.calib_cu   = cu;
  param.calib_cv   = cv;
  param.calib_f    = f;
  param.calib_base = base;

  int32_t d_width  = simg->width;
  int32_t d_height = simg->height;
  if (mode==1 && subsampling) {
    d_width  = simg->width/2;
    d_height = simg->height/2;
    param.subsampling = 1;
  }
  simg->D1 = (float*)malloc(d_width*d_height*sizeof(float));
  simg->D2 = (float*)malloc(d_width*d_height*sizeof(float));
  const int32_t dims[3] = {simg->width,simg->height,simg->step};

  Elas elas(param);
  elas.process(simg->I1,simg->I2,simg->D1,simg->D2,dims);

  // create color map
  float d_max = 200;
  if (D_color!=0)
    free (D_color);
  D_color = (float*)malloc(3*d_width*d_height*sizeof(float));
  for (int32_t u=0; u<d_width; u++) {
    for (int32_t v=0; v<d_height; v++) {
      int32_t addr1 = v*d_width+u;
      int32_t addr2 = 3*(v*d_width+u);
      float val = min(simg->D1[addr1]/d_max,(float)1.0);
      if (val<=0) {
        D_color[addr2+0] = 0; D_color[addr2+1] = 0; D_color[addr2+2] = 0;
      } else {
        float h2 = 6.0*(1.0-val);
        float x  = 1.0*(1.0-fabs(fmod(h2,(float)2.0)-1.0));
        if      (0<=h2&&h2<1)  { D_color[addr2+0] = 1; D_color[addr2+1] = x; D_color[addr2+2] = 0; }
        else if (1<=h2&&h2<2)  { D_color[addr2+0] = x; D_color[addr2+1] = 1; D_color[addr2+2] = 0; }
        else if (2<=h2&&h2<3)  { D_color[addr2+0] = 0; D_color[addr2+1] = 1; D_color[addr2+2] = x; }
        else if (3<=h2&&h2<4)  { D_color[addr2+0] = 0; D_color[addr2+1] = x; D_color[addr2+2] = 1; }
        else if (4<=h2&&h2<5)  { D_color[addr2+0] = x; D_color[addr2+1] = 0; D_color[addr2+2] = 1; }
        else if (5<=h2&&h2<=6) { D_color[addr2+0] = 1; D_color[addr2+1] = 0; D_color[addr2+2] = x; }
      }
    }
  }

  if (mode==0) {

    // adjust H_total
    // THIS IS BUGGY! (09_08_0023)
    if (0) {
      t.start("plane estimation");
      if (!plane_estimated) {
        plane->computeTransformationFromDisparityMap(simg->D1,simg->width,simg->height,simg->width,f,cu,cv,base);
        H_init = Matrix::inv(plane->getTransformation());
        plane_estimated = true;
      }
      H_total = H_init*H_total;
    }

    t.start("reconstruction");
    if (calib->calibrated())
      addDisparityMapToReconstruction();
  }

  picked = false;
  emit newDisparityMapArrived();
  while (!picked) usleep(1000);
}

StereoThread::map3d StereoThread::createCurrentMap() {

  map3d m;
  m.I      = (float*)malloc(simg->width*simg->height*sizeof(float));
  m.D      = (float*)malloc(simg->width*simg->height*sizeof(float));
  m.X      = (float*)malloc(simg->width*simg->height*sizeof(float));
  m.Y      = (float*)malloc(simg->width*simg->height*sizeof(float));
  m.Z      = (float*)malloc(simg->width*simg->height*sizeof(float));
  m.H      = H_total;
  m.width  = simg->width;
  m.height = simg->height;

  // copy disparities and intensities
  memcpy(m.D,simg->D1,simg->width*simg->height*sizeof(float));
  for (int32_t v=0; v<m.height; v++)
    for (int32_t u=0; u<m.width; u++)
      m.I[v*m.width+u] = ((float)simg->I1[v*simg->step+u])/255.0;

  // compute 3d coordinates in first coordinate system
  float hcf00=0,hcf01=0,hcf02=0,hcf03=0,hcf10=0,hcf11=0,hcf12=0,hcf13=0,hcf20=0,hcf21=0,hcf22=0,hcf23=0;
  hcf00 = m.H.val[0][0]; hcf01 = m.H.val[0][1]; hcf02 = m.H.val[0][2]; hcf03 = m.H.val[0][3];
  hcf10 = m.H.val[1][0]; hcf11 = m.H.val[1][1]; hcf12 = m.H.val[1][2]; hcf13 = m.H.val[1][3];
  hcf20 = m.H.val[2][0]; hcf21 = m.H.val[2][1]; hcf22 = m.H.val[2][2]; hcf23 = m.H.val[2][3];
  for (int32_t u=0; u<m.width; u++) {
    for (int32_t v=0; v<m.height; v++) {
      int32_t addr = v*m.width+u;
      float   d    = m.D[addr];
      if (d>0) {
        float z = f*base/d;
        if (z>0.1 && z<max_dist) {
          float x = ((float)u-cu)*base/d;
          float y = ((float)v-cv)*base/d;
          m.X[addr] = hcf00*x+hcf01*y+hcf02*z+hcf03;
          m.Y[addr] = hcf10*x+hcf11*y+hcf12*z+hcf13;
          m.Z[addr] = hcf20*x+hcf21*y+hcf22*z+hcf23;
        } else
          m.D[addr] = -1;
      }
    }
  }

  // adjust gain
  int32_t margin = min(min(200,m.width/2),m.height/2);
  float gain_inv = 1;
  if (gain)
    gain_inv = 1.0/gain;
  for (int32_t i=0; i<margin; i++) {
    float g = ((float)(margin-i)*gain_inv+(float)i*1.0)/(float)margin;
    for (int32_t u=margin; u<m.width-margin;  u++) {
      m.I[i*m.width+u]              = min(max(g*m.I[i*m.width+u],(float)0),(float)1);
      m.I[(m.height-i-1)*m.width+u] = min(max(g*m.I[(m.height-i-1)*m.width+u],(float)0),(float)1);
    }
    for (int32_t v=margin; v<simg->height-margin; v++) {
      m.I[v*m.width+i]           = min(max(g*m.I[v*m.width+i],(float)0),(float)1);
      m.I[v*m.width+m.width-i-1] = min(max(g*m.I[v*m.width+m.width-i-1],(float)0),(float)1);
    }
  }

  return m;
}

void StereoThread::releaseMap(map3d m) {
  free(m.I);
  free(m.D);
  free(m.X);
  free(m.Y);
  free(m.Z);
}

void StereoThread::addDisparityMapToReconstruction() {

  // current map
  map3d m_curr = createCurrentMap();

  // if previous map exists => try to associate with current map
  if (maps3d.size()>0) {

    // previous map
    map3d m_prev = maps3d.back();

    // projection from first coordinate system to current coordinate system
    Matrix H = Matrix::inv(m_curr.H);
    float hfc20 = H.val[2][0]; float hfc21 = H.val[2][1]; float hfc22 = H.val[2][2]; float hfc23 = H.val[2][3];

    // projection from first coordinate system to current image plane
    Matrix P = K*H.getMat(0,0,2,3);
    float pfc00 = P.val[0][0]; float pfc01 = P.val[0][1]; float pfc02 = P.val[0][2]; float pfc03 = P.val[0][3];
    float pfc10 = P.val[1][0]; float pfc11 = P.val[1][1]; float pfc12 = P.val[1][2]; float pfc13 = P.val[1][3];
    float pfc20 = P.val[2][0]; float pfc21 = P.val[2][1]; float pfc22 = P.val[2][2]; float pfc23 = P.val[2][3];

    std::vector<View3D::point_3d> points_prev;

    // for all pixels in previous image do
    for (int32_t u=0; u<m_prev.width; u++) {
      for (int32_t v=0; v<m_prev.height; v++) {

        // get pixel address and disparity
        int32_t addr = v*m_prev.width+u;
        float   d    = m_prev.D[addr];

        // if disparity is valid
        if (d>0) {

          // grab 3d coordinates in first coordinate system
          float x  = m_prev.X[addr];
          float y  = m_prev.Y[addr];
          float z  = m_prev.Z[addr];

          // map to current coordinate system
          float z2 = hfc20*x+hfc21*y+hfc22*z+hfc23;

          // if in range
          if (z2>0.1 && z2<max_dist) {

            // compute image coordinates in current image plane
            float   w2 =            pfc20*x+pfc21*y+pfc22*z+pfc23;
            int32_t u2 = (int32_t)((pfc00*x+pfc01*y+pfc02*z+pfc03)/w2);
            int32_t v2 = (int32_t)((pfc10*x+pfc11*y+pfc12*z+pfc13)/w2);

            // if within image
            if (u2>=0 && u2<m_curr.width && v2>=0 && v2<m_curr.height) {

              // get pixel address and disparity
              int32_t addr2 = v2*m_curr.width+u2;
              float   d2    = m_curr.D[addr2];

              bool added = false;

              // fuse with current image
              if (d2>0) {

                // TODO: THIS VALUE DEPENDS ON THE SCENARIO!!
                // if the points are close enough: compute
                // new position as average from previous and current
                if (fabs(x-m_curr.X[addr2])+fabs(y-m_curr.Y[addr2])+fabs(z-m_curr.Z[addr2])<0.2) {

                  m_curr.X[addr2] = (m_curr.X[addr2]+x)/2.0;
                  m_curr.Y[addr2] = (m_curr.Y[addr2]+y)/2.0;
                  m_curr.Z[addr2] = (m_curr.Z[addr2]+z)/2.0;
                  m_curr.I[addr2] = (m_curr.I[addr2]+m_prev.I[addr])/2.0;
                  //m_curr.I[addr2] = m_prev.I[addr];
                  added = true;
                }

              // copy to current image (create new coordinates)
              } else {

                m_curr.X[addr2] = x;
                m_curr.Y[addr2] = y;
                m_curr.Z[addr2] = z;
                m_curr.I[addr2] = m_prev.I[addr];
                m_curr.D[addr2] = 1; // set valid
                added = true;
              }

              // invalidate current pixel in previous image
              if (added) m_prev.D[addr] = -1;
              else       points_prev.push_back( View3D::point_3d(x,y,z,m_prev.I[addr]) );

            // add to previous points
            } else {
              points_prev.push_back( View3D::point_3d(x,y,z,m_prev.I[addr]) );
            }
          } else {
            points_prev.push_back( View3D::point_3d(x,y,z,m_prev.I[addr]) );
          }
        }
      }
    }

    // replace previous points
    points.pop_back();
    points.push_back(points_prev);
  }

  std::vector<View3D::point_3d> points_curr;

  // for all pixels do
  for (int32_t u=0; u<m_curr.width; u++) {
    for (int32_t v=0; v<m_curr.height; v++) {

      // get pixel address and disparity
      int32_t addr = v*m_curr.width+u;
      float   d    = m_curr.D[addr];

      // if disparity is valid: add 3d coordinates to point cloud
      if (d>0)
        points_curr.push_back( View3D::point_3d(m_curr.X[addr],m_curr.Y[addr],m_curr.Z[addr],m_curr.I[addr]) );
    }
  }

  // set current points
  points.push_back(points_curr);

  // add current map
  maps3d.push_back(m_curr);
}

void StereoThread::getIntrinsics() {

  // intrinsics
  f    = cvmGet(calib->P1_roi,0,0);
  cu   = cvmGet(calib->P1_roi,0,2);
  cv   = cvmGet(calib->P1_roi,1,2);
  base = -cvmGet(calib->P2_roi,0,3)/cvmGet(calib->P2_roi,0,0);

  // calibration matrix
  K = Matrix(3,3);
  K.val[0][0] = f;
  K.val[1][1] = f;
  K.val[0][2] = cu;
  K.val[1][2] = cv;
  K.val[2][2] = 1;
}

void StereoThread::clearReconstruction() {
  for (vector<map3d>::iterator it=maps3d.begin(); it!=maps3d.end(); it++)
    releaseMap(*it);
  maps3d.clear();
  points.clear();
  plane_estimated = false;
}
