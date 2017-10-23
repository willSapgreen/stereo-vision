#ifndef READFROMFILESTHREAD_H
#define READFROMFILESTHREAD_H

// Qt
#include <QThread>
#include <QString>

// opencv
#include <opencv2/highgui/highgui.hpp>

// stereo vision
#include "calibiokitti.h"
#include "stereoimage.h"
#include "stereoimageiokitti.h"

/**
    Read the camera calibration file and the stereo RECTIFIED images.
*/

// const variable definition.
const static std::string DEFAULT_CAM_TO_CAM_TXT_PATH = "/calib/calib_cam_to_cam.txt";
const static std::string DEFAULT_IMU_TO_VELO_TXT_PATH = "/calib/calib_imu_to_velo.txt";
const static std::string DEFAULT_VELO_TO_CAM_TXT_PATH = "/calib/calib_velo_to_cam.txt";
const static std::string DEFAULT_IMAGE00_DATA_PATH = "/sync/image_00/data/";
const static std::string DEFAULT_IMAGE01_DATA_PATH = "/sync/image_01/data/";
const static std::string DEFAULT_IMAGE00_TIMESTAMP_TXT_PATH = "/sync/image_00/timestamps.txt";
const static std::string DEFAULT_IMAGE01_TIMESTAMP_TXT_PATH = "/sync/image_01/timestamps.txt";

class ReadFromFilesThread : public QThread
{
    Q_OBJECT

public:

    ReadFromFilesThread(StereoImage *stereo_image, CalibIOKITTI *calib, StereoImageIOKITTI* stereo_image_io, QObject *parent = 0);
    ~ReadFromFilesThread();
    inline void setInputDir(QString input_dir) {_input_dir = input_dir;}

protected:

    void run();

private:

    CalibIOKITTI   *_calib;
    StereoImageIOKITTI *_stereo_image_io;
    StereoImage     *_stereo_image;
    QString _input_dir;

signals:

public slots:

};

#endif // READFROMFILESTHREAD_H
