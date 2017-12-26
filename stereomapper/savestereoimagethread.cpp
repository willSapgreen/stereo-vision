#include "savestereoimagethread.h"

using namespace std;

SaveStereoImageThread::SaveStereoImageThread(StereoImage::simage& simg,string output_dir,int frame_number,QObject *parent) :
    QThread(parent)
{
    _frame_number = frame_number;
    _output_dir = output_dir;

    if(_simg != 0)
    {
        delete _simg;
        _simg = 0;
    }

    _simg = new StereoImage::simage(simg);
}

//==============================================================================//

SaveStereoImageThread::~SaveStereoImageThread()
{
    if (_simg != NULL)
    {
        delete _simg;
    }
}

//==============================================================================//

void SaveStereoImageThread::run()
{
    // init header
    char file_name[1024];
    IplImage *I = cvCreateImageHeader(cvSize(_simg->width,_simg->height),IPL_DEPTH_8U,1);

    // save left image
    cvSetData(I,_simg->I1,_simg->step);
    sprintf(file_name,"%sI1_%06d.png",_output_dir.c_str(),_frame_number);

    //cvSaveImage(file_name,I);
    cv::Mat matI = cv::cvarrToMat( I );
    cv::imwrite( file_name, matI );

    // save right image
    cvSetData(I,_simg->I2,_simg->step);
    sprintf(file_name,"%sI2_%06d.png",_output_dir.c_str(),_frame_number);

    //cvSaveImage(file_name,I);
    cv::imwrite( file_name, matI );

    // release header
    cvReleaseImageHeader(&I);
}
