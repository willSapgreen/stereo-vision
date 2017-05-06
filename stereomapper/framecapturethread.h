#ifndef FRAMECAPTURETHREAD_H
#define FRAMECAPTURETHREAD_H

#include <QThread>
#include <QMutex>
#include <vector>
#include <string>
#include <dc1394/video.h>

#include "stereoimage.h"
#include "calibio.h"

class FrameCaptureThread : public QThread {

    Q_OBJECT

public:

    FrameCaptureThread(StereoImage *stereo_image,CalibIO *calib,bool cam_left,QMutex *capture_mutex,QObject *parent = 0);
    ~FrameCaptureThread();
    std::vector<std::string> queryDevices();
    void setCamInd(int ind,int sht) { cam_ind=ind; shutter = sht; }
    dc1394video_frame_t* getFrame() { return dc_frame; }
    void stopRecording() { recording = false; }

    void resetBus() {

      // create device
      dc_dev = dc1394_new();
      if (!dc_dev)
        return;

      // enumerate cameras
      dc_err = dc1394_camera_enumerate (dc_dev, &dc_list);
      if (dc_err!=0 || dc_list->num==0)
        return;

      // create camera
      dc_cam = dc1394_camera_new(dc_dev,dc_list->ids[0].guid);
      if (!dc_cam)
          return;

      // release camera list
      dc1394_camera_free_list(dc_list);

      // reset bus
      if (dc1394_reset_bus(dc_cam)!=DC1394_SUCCESS)
        return;

      // release camera and device
      dc1394_camera_free(dc_cam);
      dc1394_free(dc_dev);
    }

protected:

    void run();

private:

    void closeCamera();

    CvMat               *Mx,*My;
    IplImage            *I_rect;
    StereoImage         *stereo_image;
    QMutex              *capture_mutex;
    dc1394_t            *dc_dev;
    dc1394camera_list_t *dc_list;
    dc1394camera_t      *dc_cam;
    dc1394error_t        dc_err;
    dc1394video_frame_t *dc_frame;
    int                  cam_ind;
    CalibIO             *calib;
    bool                 cam_left;
    int                  shutter;
    bool                 recording;

signals:

public slots:

};

#endif // FRAMECAPTURETHREAD_H
