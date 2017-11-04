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
    VisualOdometryStereo::parameters visualOdomStereoParam;
    if (calib->calibrated())
    {
        visualOdomStereoParam.calib.f = calib->m_cam_to_cam_P_rect[0].at<float>(0,0); // x focal length in pixels.
        visualOdomStereoParam.calib.cu = calib->m_cam_to_cam_P_rect[0].at<float>(0,2); // principal point (u-coordinate) in pixels
        visualOdomStereoParam.calib.cv = calib->m_cam_to_cam_P_rect[0].at<float>(1,2); // principal point (v-coordinate) in pixels
        visualOdomStereoParam.base  = -(calib->m_cam_to_cam_P_rect[1].at<float>(0,3))/(calib->m_cam_to_cam_P_rect[1].at<float>(0,0)); // baseline in meters
    }
    std::cout << "visual odom stereo param(f, cu, cv, base): " << visualOdomStereoParam.calib.f
                                                               << ","
                                                               << visualOdomStereoParam.calib.cu
                                                               << ","
                                                               << visualOdomStereoParam.calib.cv
                                                               << ","
                                                               << visualOdomStereoParam.base << std::endl;
    visualOdomStereo      = new VisualOdometryStereo( visualOdomStereoParam );

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
    delete visualOdomStereo;
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

        int32_t dim[3] = {0};
        dim[0] = simg->width;
        dim[1] = simg->height;
        dim[2] = simg->step;

        visualOdomStereo->process(simg->I1,simg->I2,dim,false);
        Matrix H_inv = Matrix::eye(4);
        Matrix H = visualOdomStereo->getMotion();

        // get inliers
        vector<int32_t> inliers_ = visualOdomStereo->getInlierIndices();
        inliers.clear();
        for (int32_t i=0; i<(int32_t)visualOdomStereo->getMatches().size(); i++)
        {
        inliers.push_back(false);
        }
        for (std::vector<int32_t>::iterator it=inliers_.begin(); it!=inliers_.end(); it++)
        {
          inliers[*it] = true;
        }

        // compute gain
        gain = visualOdomStereo->getGain( inliers_ );

        if (H_inv.solve(H))
        {
            H_total = H_total*H_inv;
            picked = false;
            emit newHomographyArrived();
            while (!picked) usleep(1000);
        }
    }
}
