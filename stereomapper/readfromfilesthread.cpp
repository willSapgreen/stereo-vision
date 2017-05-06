#include "readfromfilesthread.h"

#include "QFileDialog"

using namespace std;

ReadFromFilesThread::ReadFromFilesThread(StereoImage *stereo_image,CalibIO *calib,QObject *parent) :
    QThread(parent),
    calib(calib),
    stereo_image(stereo_image) {
}

ReadFromFilesThread::~ReadFromFilesThread() {
}

void ReadFromFilesThread::run() {

  // release old images
  for (int32_t i=0; i<(int32_t)I1.size(); i++) {
    cvReleaseImage(&I1[i]);
    cvReleaseImage(&I2[i]);
  }
  I1.clear();
  I2.clear();

  // read new calibration file
  if (calib->readCalibFromFile((input_dir+"/calib.txt").toStdString())) {

    calib->showCalibrationParameters();

    char fn1[1024];
    char fn2[1024];

    FILE *f1;
    FILE *f2;

    // read images
    for (int32_t i=0; i<5000; i++) {

      string input_dir_str = input_dir.toStdString();
      sprintf(fn1,"%s/I1_%06d.png",input_dir_str.c_str(),i);
      sprintf(fn2,"%s/I2_%06d.png",input_dir_str.c_str(),i);

      f1 = fopen (fn1,"r");
      f2 = fopen (fn2,"r");

      if (f1!=NULL && f2!=NULL) {
        printf("Reading: I1_%06d.png, I2_%06d.png",i,i);
        cout << endl;
        IplImage* I1_curr = cvLoadImage(fn1,CV_LOAD_IMAGE_GRAYSCALE);
        IplImage* I2_curr = cvLoadImage(fn2,CV_LOAD_IMAGE_GRAYSCALE);
        I1.push_back(I1_curr);
        I2.push_back(I2_curr);
      }

      if (f1!=NULL) fclose(f1);
      if (f2!=NULL) fclose(f2);
    }

    // start output loop
    float fps = 10;
    unsigned char *I1_data;
    unsigned char *I2_data;
    int step1,step2;
    for (int32_t i=0; i<(int32_t)I1.size(); i++) {
      cvGetRawData(I1[i],&I1_data,&step1);
      cvGetRawData(I2[i],&I2_data,&step2);
      stereo_image->setImage(I1_data,I1[i]->width,I1[i]->height,step1,true,true);
      stereo_image->setImage(I2_data,I2[i]->width,I2[i]->height,step2,false,true);
      usleep(1e6/fps);
    }

    // release images
    for (int32_t i=0; i<(int32_t)I1.size(); i++) {
      cvReleaseImage(&I1[i]);
      cvReleaseImage(&I2[i]);
    }
    I1.clear();
    I2.clear();

  } else {
    cout << "No calibration file found => No files read." << endl;
  }
}
