#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QDialog>
#include <QMutex>
#include "QSettings"
#include <sys/time.h>
#include <opencv/highgui.h>

#include "visualodometrythread.h"
#include "stereothread.h"
#include "calibiokitti.h"
#include "savestereoimagethread.h"
#include "readfromfilesthread.h"
#include "visualizethread.h"
#include "gpxgenerator.h"
#include "stereoimage.h"
#include "gpsinertialdata.h"

#include "headingfilter.h"
#include "positionfilter.h"

namespace Ui
{
    class MainDialog;
}

class MainDialog : public QDialog
{
    Q_OBJECT

public:

    MainDialog(QWidget *parent = 0);
    ~MainDialog();

private:

    void                 keyPressEvent(QKeyEvent * event);

    Ui::MainDialog*       _ui;
    StereoImage*          _stereo_image;
    GPSInertialData*      _gps_inertial_data;
    VisualOdometryThread* _visual_odom_thread;
    HeadingFilter*        _heading_filter;
    PositionFilter*       _position_filter;
    StereoThread*         _stereo_thread;
    ReadFromFilesThread*  _read_thread;
    VisualizeThread*      _visualize_thread;
    QMutex*               _capture_mutex;
    timeval               _time_of_last_frame;
    CalibIOKITTI*         _calib;
    CvMat*                _M1x;
    CvMat*                _M1y;
    CvMat*                _M2x;
    CvMat*                _M2y;
    IplImage*             _I1_rect;
    IplImage*             _I2_rect;
    QSettings*            _settings;
    int                   _frame_index;
    timeval               _last_frame_time;
    bool                  _stereo_scan;
    float                 _gain_total;
    bool                  _save_single_frame;
    std::string           _output_dir;
    std::vector<SaveStereoImageThread*> _save_stereo_threads;
    StereoImageIOKITTI* _stereo_image_io;
    GPSInertialDataIOKITTI* _gps_inertial_data_io;
    GpxGenerator* _ground_truth_gpx_generator;
    GpxGenerator* _visual_odom_gpx_generator;
    Matrix _cam_to_imu_trans;
    bool _is_cam_to_imu_transformation_ready;
    bool _is_first_gnss_ins_ready;

private slots:

    void on_resizeSmallPushButton_clicked();
    void on_whiteCheckBox_clicked();
    void on_gridCheckBox_clicked();
    void on_recordPosesPushButton_clicked();
    void on_playPosesPushButton_clicked();
    void on_deletePosePushButton_clicked();
    void on_addPosePushButton_clicked();
    void on_resizePushButton_clicked();
    void on_showCamerasCheckBox_clicked();
    void on_backgroundWallCheckBox_clicked();
    void on_backgroundWallSlider_sliderMoved(int position);
    void on_stereoScanButton_clicked();
    void on_exitButton_clicked();

    void onNewStereoImageArrived();

    /*
     * Handle the homography transformation from visual odom processing.
     */
    void onNewHomographyArrived();

    void onNewDisparityMapArrived();
    void onNewGPSInertialDataArrived();
    void onNewCalibrationData();
    void onPlaybackDataFinished();

};

#endif // MAINDIALOG_H
