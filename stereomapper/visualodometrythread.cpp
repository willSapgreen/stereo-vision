#include "visualodometrythread.h"

using namespace std;

VisualOdometryThread::VisualOdometryThread(CalibIOKITTI *calib,QObject *parent)
    : QThread(parent)
    , _calib(calib)
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
    if (_calib->calibrated())
    {
        visualOdomStereoParam.calib.f = _calib->_cam_to_cam_P_rect[0].at<float>(0,0); // x focal length in pixels.
        visualOdomStereoParam.calib.cu = _calib->_cam_to_cam_P_rect[0].at<float>(0,2); // principal point (u-coordinate) in pixels
        visualOdomStereoParam.calib.cv = _calib->_cam_to_cam_P_rect[0].at<float>(1,2); // principal point (v-coordinate) in pixels

        /*
        Projection matrix is defined as K[I|t]. 3x4
        K: intrinsic matrix
        fx 0  cv
        0  fy cu
        0  0  1

        I: rotation matrix
        r1 r2 r3
        r4 r5 r6
        r7 r8 r9

        t: translation matrix
        t1
        t2
        t3

        so m_cam_to_cam_P_rect[1].at<float>(0,3) is fx*t1
        To get the t1( the distance between two cameras), t1 = (fx*t1/fx)
        */
        visualOdomStereoParam.base  = -(_calib->_cam_to_cam_P_rect[1].at<float>(0,3))/(_calib->_cam_to_cam_P_rect[1].at<float>(0,0)); // baseline in meters
    }

    _visualOdomStereo      = new VisualOdometryStereo( visualOdomStereoParam );

    _simg              = 0;
    _time_prev.tv_sec  = 0;
    _time_prev.tv_usec = 0;
    _time_curr.tv_sec  = 0;
    _time_curr.tv_usec = 0;
    _record_raw_odometry = false;
    _H_total = Matrix(4,4);
    _H_total.eye();
}

VisualOdometryThread::~VisualOdometryThread()
{
    delete _visualOdomStereo;
    if (_simg!=0)
    {
        delete _simg;
        _simg = 0;
    }
}

//==============================================================================//

void VisualOdometryThread::pushBack(StereoImage::simage& s,bool record_raw_odometry)
{
    if (_simg!=0)
    {
        delete _simg;
        _simg = 0;
    }
    _simg = new StereoImage::simage(s);
    _record_raw_odometry = record_raw_odometry;
}

//==============================================================================//

void VisualOdometryThread::run()
{
    if (_simg!=0 && _simg->width>0 && _simg->height>0)
    {
        // get time
        _time_prev = _time_curr;
        _time_curr = _simg->time;

        int32_t dim[3] = {0};
        dim[0] = _simg->width;
        dim[1] = _simg->height;
        dim[2] = _simg->step;

        _visualOdomStereo->process(_simg->I1,_simg->I2,dim,false);
        Matrix H_inv = Matrix::eye(4);
        Matrix H = _visualOdomStereo->getMotion();

        // get inliers
        vector<int32_t> inliers = _visualOdomStereo->getInlierIndices();
        _inliers.clear();
        for (int32_t i=0; i<(int32_t)_visualOdomStereo->getMatches().size(); i++)
        {
            _inliers.push_back(false);
        }
        for (std::vector<int32_t>::iterator it=inliers.begin(); it!=inliers.end(); it++)
        {
            _inliers[*it] = true;
        }

        // compute gain
        _gain = _visualOdomStereo->getGain( inliers );

        if (H_inv.solve(H))
        {
            _H_total = _H_total*H_inv;
            _picked = false;
            emit newHomographyArrived();
            while (!_picked) usleep(1000);
        }
    }
}
