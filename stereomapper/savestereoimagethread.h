#ifndef SAVESTEREOIMAGETHREAD_H
#define SAVESTEREOIMAGETHREAD_H

#include <QThread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "stereoimage.h"

class SaveStereoImageThread : public QThread
{
  Q_OBJECT

public:

  SaveStereoImageThread(StereoImage::simage& simg,std::string output_dir,int frame_number,QObject *parent = 0);
  ~SaveStereoImageThread();

protected:

  void run();

private:

  StereoImage::simage* _simg;
  int _frame_number;
  std::string _output_dir;

};

#endif // SAVESTEREOIMAGETHREAD_H
