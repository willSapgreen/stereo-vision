#include "stereoimage.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>

using namespace std;

StereoImage::StereoImage(QObject *parent):QObject(parent)
{
    _img_left  = new image();
    _img_right = new image();
    _simg      = new simage();
    _picked = false;
}

//==============================================================================//

StereoImage::~StereoImage()
{
    if (_img_left!=0)   { delete _img_left;  _img_left  = 0; }
    if (_img_right!=0)  { delete _img_right; _img_right = 0; }
    if (_simg!=0)       { delete _simg;      _simg      = 0; }
}

//==============================================================================//

void StereoImage::setImage(unsigned char* data,int width,int height,int step,bool cam_left,bool rectified,timeval captured_time )
{
    // get pointer to data
    image *img = _img_right;
    if (cam_left)
    {
        img = _img_left;
    }

    // if width or height has changed => free old & allocate new memory block
    if (width!=img->width || height!=img->height || step!=img->step)
    {
        if (img->data!=0)
        {
            free(img->data);
            img->data = 0;
        }
        img->data   = (unsigned char*)malloc(step*height*sizeof(unsigned char));
        img->width  = width;
        img->height = height;
        img->step   = step;
    }

    // set timestamp
    img->time = captured_time;

    // copy data
    memcpy(img->data,data,step*height*sizeof(unsigned char));

    // do we have a valid stereo frame (conservative: 10ms)? => copy it!
    if( fabs(timeDiff(_img_left->time,_img_right->time)) < 10 )
    {
        // do both frames have same size?
        if (_img_left->width==_img_right->width && _img_left->height==_img_right->height && _img_left->step==_img_right->step)
        {
            // if width or height has changed => free old & allocate new memory block
            if (_img_left->width!=_simg->width || _img_left->height!=_simg->height || _img_left->step!=_simg->step)
            {
                if (_simg->I1!=0) { free(_simg->I1); _simg->I1 = 0; }
                if (_simg->I2!=0) { free(_simg->I2); _simg->I2 = 0; }
                _simg->I1     = (unsigned char*)malloc(_img_left->step*_img_left->height*sizeof(unsigned char));
                _simg->I2     = (unsigned char*)malloc(_img_left->step*_img_left->height*sizeof(unsigned char));
                _simg->width  = _img_left->width;
                _simg->height = _img_left->height;
                _simg->step   = _img_left->step;
            }

            // copy timestamp and set rectification flag
            _simg->time      = _img_left->time;
            _simg->rectified = rectified;

            // copy data from single images to stereo images
            memcpy(_simg->I1,_img_left->data,_simg->step*_simg->height*sizeof(unsigned char));
            memcpy(_simg->I2,_img_right->data,_simg->step*_simg->height*sizeof(unsigned char));

            // signal to main dialog that we have new data
            _picked = false;
            emit newStereoImageArrived();
            while (!_picked) usleep(1000);
        }
    }
}

//==============================================================================//

void StereoImage::histogramNormalization(unsigned char* I,const int &width,const int &height,const int &step)
{
    // compute probability histogram
    float fraction = 1.0/(float)(width*height);
    float p[256];
    for (int32_t i=0; i<256; i++)
    {
        p[i] = 0;
    }
    for (int32_t v=0; v<height; v++)
    {
        for (int32_t u=0; u<width; u++)
        {
            p[*(I+v*step+u)] += fraction;
        }
    }

    // compute cumulative distribution
    float cdf[256];
    cdf[0] = p[0];
    for (int32_t i=1; i<256; i++)
    {
        cdf[i] = cdf[i-1]+p[i];
    }

    // compute mapping
    uint8_t map[256];
    for (int32_t i=0; i<256; i++)
    {
        map[i] = (uint8_t)(cdf[i]*255.0);
    }

    // map pixels
    for (int32_t v=0; v<height; v++)
    {
        for (int32_t u=0; u<width; u++)
        {
            *(I+v*step+u) = map[*(I+v*step+u)];
        }
    }
}
