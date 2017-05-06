#include "framecapturethread.h"

#include <iostream>
#include <sstream>
#include <unistd.h>

using namespace std;

FrameCaptureThread::FrameCaptureThread(StereoImage *stereo_image,CalibIO *calib,bool cam_left,QMutex *capture_mutex,QObject *parent) :
    QThread(parent),
    stereo_image(stereo_image),
    capture_mutex(capture_mutex),
    calib(calib),
    cam_left(cam_left)
{
  cam_ind  = 0;
  shutter  = 100;
  dc_frame = NULL; 
}

FrameCaptureThread::~FrameCaptureThread() {
  if (Mx)     cvReleaseMat(&Mx);
  if (My)     cvReleaseMat(&My);
  if (I_rect) cvReleaseImage(&I_rect);
  terminate();
  closeCamera();
}

vector<string> FrameCaptureThread::queryDevices() {

  vector<string> device_names;

  // create dc1394 instance
  dc_dev = dc1394_new();
  if (!dc_dev) {
    cerr << "Could not create dc1394 instance." << endl;
    return device_names;
  }

  // enumerate cameras
  if (dc1394_camera_enumerate(dc_dev, &dc_list)!=0) {
    cerr << "Failed to enumerate cameras. Do you have access rights to /dev/video1394-x and /dev/raw1394?" << endl;
    return device_names;
  }

  // check if 2 cameras found
  if (dc_list->num <= 1) {
    cerr << "We need at least two cameras!" << endl;
    return device_names;
  }

  // return device names
  for (uint32_t i=0; i<dc_list->num; i++) {
    stringstream ss(stringstream::in | stringstream::out);
    ss << std::hex << dc_list->ids[i].guid;
    device_names.push_back(ss.str());
  }
  return device_names;
}

void FrameCaptureThread::closeCamera() {
  cout << endl << "Closing device ... ";
  dc1394_capture_stop(dc_cam);
  dc1394_video_set_transmission(dc_cam, DC1394_OFF);
  dc1394_camera_free(dc_cam);
  dc1394_free(dc_dev);
  cout << "done." << endl;
}

void FrameCaptureThread::run() {

  // TODO: This has to be released when thread is killed!
  if (calib->calibrated()) {
    Mx = cvCreateMat(calib->height(),calib->width(),CV_32F);
    My = cvCreateMat(calib->height(),calib->width(),CV_32F);
    if (cam_left) cvInitUndistortRectifyMap(calib->K1,calib->D1,calib->R1,calib->P1_roi,Mx,My);
    else          cvInitUndistortRectifyMap(calib->K2,calib->D2,calib->R2,calib->P2_roi,Mx,My);
    I_rect = cvCreateImage(cvSize(calib->width(),calib->height()),IPL_DEPTH_8U,1);
  }

  // initialize camera
  dc_cam = dc1394_camera_new(dc_dev,dc_list->ids[cam_ind].guid);
  if (!dc_cam) {
    cerr << "Failed to initialize camera. Exiting ..." << endl;
    return;
  }

  // set mode to firewire b
  if (dc1394_video_set_operation_mode(dc_cam,DC1394_OPERATION_MODE_1394B)!=0) {
    cerr << "Could not set operation mode to 1394B. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // set iso speed
  if (dc1394_video_set_iso_speed(dc_cam,DC1394_ISO_SPEED_800)!=0) {
    cerr << "Could not set iso speed. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // set video mode
  if (dc1394_video_set_mode(dc_cam,DC1394_VIDEO_MODE_FORMAT7_0)!=0) {
    cerr << "Could not set video mode." << endl;
    closeCamera();
    return;
  }

  // set frame rate
  if (dc1394_video_set_framerate(dc_cam,DC1394_FRAMERATE_15)!=0) {
    cerr << "Could not set framerate. Exiting ..." << endl;
    closeCamera();
    return;
  }

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  // set format7
//  if (dc1394_format7_set_roi(dc_cam,DC1394_VIDEO_MODE_FORMAT7_0,DC1394_COLOR_CODING_MONO8,2000,
//                                    0,260,1392,512)!=0) {
  if (dc1394_format7_set_roi(dc_cam,DC1394_VIDEO_MODE_FORMAT7_0,DC1394_COLOR_CODING_MONO8,2000,
                                    184,4,1024,1024)!=0) {
    cerr << "Could not set format 7. Exiting ..." << endl;
    closeCamera();
    return;
  }

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  // setting up capture device
  if (dc1394_capture_setup(dc_cam,8, DC1394_CAPTURE_FLAGS_DEFAULT)!=0) {
    cerr << "Error setting up capture device. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // start transmission
  if (dc1394_video_set_transmission(dc_cam, DC1394_ON)!=0) {
    cerr << "Could not start camera iso transmission. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // automatic brightness mode
  if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_BRIGHTNESS,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
    cout << "Could not set automatic mode for brightness. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // automatic gamma mode
  if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_GAMMA,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
    cout << "Could not set automatic mode for gamma. Exiting ..." << endl;
    closeCamera();
    return;
  }

  if (1) {
    // manual sharpness mode
    if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_SHARPNESS,DC1394_FEATURE_MODE_MANUAL)!=DC1394_SUCCESS){
      cout << "Could not set manual mode for sharpness. Exiting ..." << endl;
      closeCamera();
      return;
    }
    // set sharpness (0=unscharf,4096=scharf)
    if (dc1394_feature_set_value(dc_cam,DC1394_FEATURE_SHARPNESS,0)!=DC1394_SUCCESS){
      cerr << "Error: Could not set sharpness. Exiting ..." << endl;
      closeCamera();
      return;
    }
  } else {
    // automatic gamma mode
    if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_SHARPNESS,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
      cout << "Could not set automatic mode for sharpness. Exiting ..." << endl;
      closeCamera();
      return;
    }
  }



  // automatic exposure mode
  if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_EXPOSURE,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
    cout << "Could not set automatic exposure mode. Exiting ..." << endl;
    closeCamera();
    return;
  }

  /*
  // manual exposure mode
  if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_EXPOSURE,DC1394_FEATURE_MODE_MANUAL)!=DC1394_SUCCESS){
    cout << "Could not set manual exposure mode. Exiting ..." << endl;
    closeCamera();
    return;
  }
  */

  if (0) {
    // manual shutter mode
    if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_SHUTTER,DC1394_FEATURE_MODE_MANUAL)!=DC1394_SUCCESS){
      cout << "Could not set manual mode for shutter speed. Exiting ..." << endl;
      closeCamera();
      return;
    }

    // set shutter
    if (dc1394_feature_set_value(dc_cam,DC1394_FEATURE_SHUTTER,shutter)!=DC1394_SUCCESS){
      cerr << "Error: Could not set exposure. Exiting ..." << endl;
      closeCamera();
      return;
    }
  } else {
    // automatic shutter mode
    if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_SHUTTER,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
      cout << "Could not set manual mode for shutter speed. Exiting ..." << endl;
      closeCamera();
      return;
    }
  }

  // automatic gain mode
  if (dc1394_feature_set_mode(dc_cam,DC1394_FEATURE_GAIN,DC1394_FEATURE_MODE_AUTO)!=DC1394_SUCCESS){
    cout << "Could not set automatic mode for gain. Exiting ..." << endl;
    closeCamera();
    return;
  }

  // software trigger
  if (dc1394_external_trigger_set_power(dc_cam,DC1394_OFF)!=DC1394_SUCCESS){
    cout << "Could not set external trigger power. Exiting ..." << endl;
    return;
  }
  if (dc1394_software_trigger_set_power(dc_cam,DC1394_ON)!=DC1394_SUCCESS){
    cout << "Could not set software trigger power. Exiting ..." << endl;
    closeCamera();
    return;
  }

  recording = true;
  int frame = 0;
  do {

    // capture frame
    if (dc1394_capture_dequeue(dc_cam, DC1394_CAPTURE_POLICY_WAIT, &dc_frame)!=0) {
      cerr << "Could not capture a frame. Exiting ..." << endl;
      closeCamera();
      return;
    }

    // rectified image
    if (calib->calibrated()) {

      IplImage *I = cvCreateImageHeader(cvSize(dc_frame->size[0],dc_frame->size[1]),IPL_DEPTH_8U,1);
      cvSetData(I,dc_frame->image,dc_frame->size[0]);
      cvRemap(I,I_rect,Mx,My,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(0));
      int step; // OpenCV allocates aligned memory
      unsigned char *I_data;
      cvGetRawData(I_rect,&I_data,&step);
      capture_mutex->lock();
      stereo_image->setImage(I_data,calib->width(),calib->height(),step,cam_left,true);
      capture_mutex->unlock();
      cvReleaseImageHeader(&I);

    // raw image
    } else {
      if (frame==0)
        cout << dc_frame->size[0] << " x " << dc_frame->size[1] << endl;
      capture_mutex->lock();
      stereo_image->setImage(dc_frame->image,dc_frame->size[0],dc_frame->size[1],dc_frame->size[0],cam_left,false);
      capture_mutex->unlock();
    }

    // release dc frame buffer
    if (dc1394_capture_enqueue(dc_cam,dc_frame)!=DC1394_SUCCESS) {
      cerr << "Failed to enqueue Buffer. Exiting ..." << endl;
      return;
    }

    frame++;
  } while(recording);

  // cleanup
  closeCamera();
}
