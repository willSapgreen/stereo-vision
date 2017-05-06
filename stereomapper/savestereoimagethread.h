#ifndef SAVESTEREOIMAGETHREAD_H
#define SAVESTEREOIMAGETHREAD_H

#include <QThread>
#include "stereoimage.h"

class SaveStereoImageThread : public QThread
{
  Q_OBJECT

public:

  SaveStereoImageThread(StereoImage::simage simg,std::string output_dir,int frame_number_,QObject *parent = 0);
  ~SaveStereoImageThread();

protected:

  void run();

private:

  StereoImage::simage *simg;
  int frame_number;
  std::string output_dir;

};

#endif // SAVESTEREOIMAGETHREAD_H
