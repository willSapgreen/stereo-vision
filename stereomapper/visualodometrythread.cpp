#include "visualodometrythread.h"

using namespace std;

VisualOdometryThread::VisualOdometryThread(CalibIOKITTI *calib,QObject *parent)
    : QThread(parent)
    , calib(calib)
{
    //Matcher::parameters matcherParam( 3,50,50,300,2,5,5,1,1,2 );
    //Matcher::parameters matcherParam;
    //matcherParam.match_radius = 200;
    //matcherParam.refinement = 2;
    //matcher = new Matcher( matcherParam );

    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    // Now we use the 1st and 2nd cameras in KITTI data set for visual odometry
    // TODO: Let the user have the choice.
    VisualOdometryStereo::parameters voParam;
    if (calib->calibrated())
    {
        voParam.calib.f = calib->m_cam_to_cam_P_rect[0].at<float>(0,0); // x focal length in pixels.
        voParam.calib.cu = calib->m_cam_to_cam_P_rect[0].at<float>(0,2); // principal point (u-coordinate) in pixels
        voParam.calib.cv = calib->m_cam_to_cam_P_rect[0].at<float>(1,2); // principal point (v-coordinate) in pixels
        voParam.base  = -(calib->m_cam_to_cam_P_rect[1].at<float>(0,3))/(calib->m_cam_to_cam_P_rect[1].at<float>(0,0)); // baseline in meters
    }
    vo      = new VisualOdometryStereo( voParam );

    simg              = 0;
    time_prev.tv_sec  = 0;
    time_prev.tv_usec = 0;
    time_curr.tv_sec  = 0;
    time_curr.tv_usec = 0;
    record_raw_odometry = false;
    H_total = Matrix(4,4);
    H_total.eye();
}

VisualOdometryThread::~VisualOdometryThread()
{
    delete vo;
    if (simg!=0)
    {
        delete simg;
        simg = 0;
    }
}

//==============================================================================//

void VisualOdometryThread::pushBack(StereoImage::simage &s,bool record_raw_odometry_)
{
    if (simg!=0)
    {
        delete simg;
        simg = 0;
    }
    simg = new StereoImage::simage(s);
    record_raw_odometry = record_raw_odometry_;
}

//==============================================================================//

void VisualOdometryThread::run()
{
    if (simg!=0 && simg->width>0 && simg->height>0)
    {
        // get time
        time_prev = time_curr;
        time_curr = simg->time;

        // read calibration
        //float f  = 1;
        //float cu = 0;
        //float cv = 0;
        //float b  = 1;
        //if (calib->calibrated())
        //{
            //f  =  cvmGet(calib->P1_roi,0,0);
            //cu =  cvmGet(calib->P1_roi,0,2);
            //cv =  cvmGet(calib->P1_roi,1,2);
            //b  = -cvmGet(calib->P2_roi,0,3)/cvmGet(calib->P2_roi,0,0);
        //}

        //Timer t;
        //t.start("push");
        int32_t dim[3] = {0};
        dim[0] = simg->width;
        dim[1] = simg->height;
        dim[2] = simg->step;
        //matcher->pushBack( simg->I1,simg->I2,dim,false );

        //t.start("match");
        //matcher->matchFeatures(2);
        //matcher->bucketFeatures(5,50,50);

        // grab matches
        //matches = matcher->getMatches();

        //t.start("vo");

        // update: H_total = H_total * H^-1
        //if (matches.size()>0)
        //{

          // visual odometry
          vo->process( simg->I1,simg->I2,dim,false );
          Matrix H_inv = Matrix::eye(4);
          Matrix H = vo->getMotion();

          // get inliers
          vector<int32_t> inliers_ = vo->getInlierIndices();
          inliers.clear();
          for (int32_t i=0; i<(int32_t)vo->getMatches().size(); i++)
          {
            inliers.push_back(false);
          }
          for (std::vector<int32_t>::iterator it=inliers_.begin(); it!=inliers_.end(); it++)
          {
              inliers[*it] = true;
          }

          // compute gain
          gain = vo->getGain( inliers_ );

          if (H_inv.solve(H))
          {
            H_total = H_total*H_inv;
            picked = false;
            emit newHomographyArrived();
            while (!picked) usleep(1000);
          }
        //}
    }
}
