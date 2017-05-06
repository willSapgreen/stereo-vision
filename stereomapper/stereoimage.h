#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <QObject>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

class StereoImage : public QObject
{
    Q_OBJECT

public:

    StereoImage(QObject *parent=0);
    ~StereoImage();

    struct simage {
      unsigned char* I1;
      unsigned char* I2;
      float*         D1;
      float*         D2;
      int            width;
      int            height;
      int            step;
      timeval        time;
      bool           rectified;
      simage () {
        I1           = 0;
        I2           = 0;
        D1           = 0;
        D2           = 0;
        width        = 0;
        height       = 0;
        step         = 0;
        time.tv_sec  = 0;
        time.tv_usec = 0;
      }
      simage (const StereoImage::simage &src) {
        I1 = 0;
        I2 = 0;
        D1 = 0;
        D2 = 0;
        if (src.I1!=0) {
          I1 = (unsigned char*)malloc(src.step*src.height*sizeof(unsigned char));
          memcpy(I1,src.I1,src.step*src.height*sizeof(unsigned char));
        }
        if (src.I2!=0) {
          I2 = (unsigned char*)malloc(src.step*src.height*sizeof(unsigned char));
          memcpy(I2,src.I2,src.step*src.height*sizeof(unsigned char));
        }
        if (src.D1!=0) {
          D1 = (float*)malloc(src.step*src.height*sizeof(float));
          memcpy(D1,src.D1,src.step*src.height*sizeof(float));
        }
        if (src.D2!=0) {
          D2 = (float*)malloc(src.step*src.height*sizeof(float));
          memcpy(D2,src.D2,src.step*src.height*sizeof(float));
        }
        width     = src.width;
        height    = src.height;
        step      = src.step;
        time      = src.time;
        rectified = src.rectified;
      }
      ~simage () {
        if (I1!=0) { free(I1); I1 = 0; }
        if (I2!=0) { free(I2); I2 = 0; }
        if (D1!=0) { free(D1); D1 = 0; }
        if (D2!=0) { free(D2); D2 = 0; }
      }
    };

    void setImage(unsigned char* data,int width,int height,int step,bool cam_left,bool rectified);
    bool isRectified() { return 0; }
    simage* getStereoImage() { return simg; }
    void pickedUp() { picked = true; }
    void histogramNormalization(unsigned char* I,const int &width,const int &height,const int &step);

    float timeDiff(timeval a,timeval b) {
      return ((float)(a.tv_sec -b.tv_sec ))*1e+3 +
             ((float)(a.tv_usec-b.tv_usec))*1e-3;
    }

private:

    struct image {
      unsigned char* data;
      int            width;
      int            height;
      int            step;
      timeval        time;
      image () {
        data         = 0;
        width        = 0;
        height       = 0;
        time.tv_sec  = 0;
        time.tv_usec = 0;
      }
      ~image () {
        if (data!=0) { free(data); data = 0; }
      }
    };

    image  *img_left;
    image  *img_right;
    simage *simg;
    bool    picked;

signals:

    void newStereoImageArrived();

};

#endif // STEREOIMAGE_H
