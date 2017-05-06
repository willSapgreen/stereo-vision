#include "stereoimage.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>

using namespace std;

StereoImage::StereoImage(QObject *parent) :
    QObject(parent) {
  img_left  = new image();
  img_right = new image();
  simg      = new simage();
}

StereoImage::~StereoImage()
{
  if (img_left!=0)   { delete img_left;  img_left  = 0; }
  if (img_right!=0)  { delete img_right; img_right = 0; }
  if (simg!=0)       { delete simg;      simg      = 0; }
}

void StereoImage::setImage(unsigned char* data,int width,int height,int step,bool cam_left,bool rectified) {

  // get pointer to data
  image *img = img_right;
  if (cam_left)
    img = img_left;

  // if width or height has changed => free old & allocate new memory block
  if (width!=img->width || height!=img->height || step!=img->step) {
    if (img->data!=0) {
      free(img->data);
      img->data = 0;
    }
    img->data   = (unsigned char*)malloc(step*height*sizeof(unsigned char));
    img->width  = width;
    img->height = height;
    img->step   = step;
  }

  // set timestamp
  gettimeofday(&img->time,0);

  // copy data
  memcpy(img->data,data,step*height*sizeof(unsigned char));

  // do we have a valid stereo frame (conservative: 10ms)? => copy it!
  if (fabs(timeDiff(img_left->time,img_right->time))<10) {

    // do both frames have same size?
    if (img_left->width==img_right->width && img_left->height==img_right->height && img_left->step==img_right->step) {

      // if width or height has changed => free old & allocate new memory block
      if (img_left->width!=simg->width || img_left->height!=simg->height || img_left->step!=simg->step) {
        if (simg->I1!=0) { free(simg->I1); simg->I1 = 0; }
        if (simg->I2!=0) { free(simg->I2); simg->I2 = 0; }
        simg->I1     = (unsigned char*)malloc(img_left->step*img_left->height*sizeof(unsigned char));
        simg->I2     = (unsigned char*)malloc(img_left->step*img_left->height*sizeof(unsigned char));
        simg->width  = img_left->width;
        simg->height = img_left->height;
        simg->step   = img_left->step;
      }

      // copy timestamp and set rectification flag
      simg->time      = img_left->time;
      simg->rectified = rectified;

      // copy data from single images to stereo images
      memcpy(simg->I1,img_left->data,simg->step*simg->height*sizeof(unsigned char));
      memcpy(simg->I2,img_right->data,simg->step*simg->height*sizeof(unsigned char));

      // signal to main dialog that we have new data
      picked = false;
      emit newStereoImageArrived();
      while (!picked) usleep(1000);
    }
  }
}

void StereoImage::histogramNormalization(unsigned char* I,const int &width,const int &height,const int &step) {

  // compute probability histogram
  float fraction = 1.0/(float)(width*height);
  float p[256];
  for (int32_t i=0; i<256; i++)
    p[i] = 0;
  for (int32_t v=0; v<height; v++)
    for (int32_t u=0; u<width; u++)
      p[*(I+v*step+u)] += fraction;

  // compute cumulative distribution
  float cdf[256];
  cdf[0] = p[0];
  for (int32_t i=1; i<256; i++)
    cdf[i] = cdf[i-1]+p[i];

  // compute mapping
  uint8_t map[256];
  for (int32_t i=0; i<256; i++)
    map[i] = (uint8_t)(cdf[i]*255.0);

  // map pixels
  for (int32_t v=0; v<height; v++)
    for (int32_t u=0; u<width; u++)
      *(I+v*step+u) = map[*(I+v*step+u)];
}

