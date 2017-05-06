#include "savestereoimagethread.h"
#include <opencv/highgui.h>

using namespace std;

SaveStereoImageThread::SaveStereoImageThread(StereoImage::simage simg_,string output_dir_,int frame_number_,QObject *parent) :
    QThread(parent) {
  frame_number = frame_number_;
  output_dir = output_dir_;
  simg = new StereoImage::simage(simg_);
}

SaveStereoImageThread::~SaveStereoImageThread(){
  delete simg;
}

void SaveStereoImageThread::run() {

  // init header
  char file_name[1024];
  IplImage *I = cvCreateImageHeader(cvSize(simg->width,simg->height),IPL_DEPTH_8U,1);

  // save left image
  cvSetData(I,simg->I1,simg->step);
  sprintf(file_name,"%sI1_%06d.png",output_dir.c_str(),frame_number);
  cvSaveImage(file_name,I);

  // save right image
  cvSetData(I,simg->I2,simg->step);
  sprintf(file_name,"%sI2_%06d.png",output_dir.c_str(),frame_number);
  cvSaveImage(file_name,I);

  // release header
  cvReleaseImageHeader(&I);
}
