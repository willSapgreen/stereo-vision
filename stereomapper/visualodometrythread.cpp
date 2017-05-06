#include "visualodometrythread.h"

using namespace std;

VisualOdometryThread::VisualOdometryThread(CalibIO *calib,QObject *parent) :
    QThread(parent),
    calib(calib) {

  matcher = new Matcher(3,50,50,300,2,5,5,1,1,2);
  vo      = new VisualOdometry();
  simg              = 0;
  time_prev.tv_sec  = 0;
  time_prev.tv_usec = 0;
  time_curr.tv_sec  = 0;
  time_curr.tv_usec = 0;
  record_raw_odometry = false;
  H_total = Matrix(4,4);
  H_total.eye();
}

VisualOdometryThread::~VisualOdometryThread() {
  delete matcher;
  delete vo;
  if (simg!=0) {
    delete simg;
    simg = 0;
  }
}

void VisualOdometryThread::pushBack(StereoImage::simage &s,bool record_raw_odometry_) {
  if (simg!=0) {
    delete simg;
    simg = 0;
  }
  simg = new StereoImage::simage(s);
  record_raw_odometry = record_raw_odometry_;
}

void VisualOdometryThread::run() {

  if (simg!=0 && simg->width>0 && simg->height>0) {

    // get time
    time_prev = time_curr;
    time_curr = simg->time;

    // read calibration
    float f  = 1;
    float cu = 0;
    float cv = 0;
    float b  = 1;
    if (calib->calibrated()) {
      f  =  cvmGet(calib->P1_roi,0,0);
      cu =  cvmGet(calib->P1_roi,0,2);
      cv =  cvmGet(calib->P1_roi,1,2);
      b  = -cvmGet(calib->P2_roi,0,3)/cvmGet(calib->P2_roi,0,0);
    }

    Timer t;
    t.start("push");
    matcher->pushBack(simg->I1,simg->I2,simg->width,simg->height,simg->step,false);

    t.start("match");
    matcher->matchFeatures(2);
    matcher->bucketFeatures(5,50,50);

    // grab matches
    matches = matcher->getMatches();

    t.start("vo");

    // update: H_total = H_total * H^-1
    if (matches.size()>0) {

      // visual odometry
      vo->setCalibration(f,cu,cv,b);
      vo->update(matches,timeDiff(time_curr,time_prev),true,record_raw_odometry);
      Matrix H_inv = Matrix::eye(4);
      Matrix H = vo->getTransformation();

      // get inliers
      vector<int32_t> inliers_ = vo->getInliers();
      inliers.clear();
      for (int32_t i=0; i<(int32_t)matches.size(); i++)
        inliers.push_back(false);
      for (std::vector<int32_t>::iterator it=inliers_.begin(); it!=inliers_.end(); it++)
        inliers[*it] = true;

      // compute gain
      gain = matcher->getGain(inliers_);

      if (H_inv.solve(H)) {
        H_total = H_total*H_inv;
        picked = false;
        emit newHomographyArrived();
        while (!picked) usleep(1000);
      }
    }
  }
}
