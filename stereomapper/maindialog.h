#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QDialog>
#include <QMutex>
#include "QSettings"
#include <sys/time.h>
#include <opencv/highgui.h>

#include "framecapturethread.h"
#include "visualodometrythread.h"
#include "stereothread.h"
#include "stereoimage.h"
#include "calibio.h"
#include "savestereoimagethread.h"
#include "readfromfilesthread.h"
#include "visualizethread.h"

namespace Ui {
    class MainDialog;
}

class MainDialog : public QDialog {

    Q_OBJECT

public:

    MainDialog(QWidget *parent = 0);
    ~MainDialog();

private:

    std::string          createNewOutputDirectory();
    void                 keyPressEvent(QKeyEvent * event);

    Ui::MainDialog       *ui;
    StereoImage          *stereo_image;
    FrameCaptureThread   *cam_left;
    FrameCaptureThread   *cam_right;
    VisualOdometryThread *vo_thread;
    StereoThread         *stereo_thread;
    ReadFromFilesThread  *read_thread;
    VisualizeThread      *visualize_thread;
    QMutex               *capture_mutex;
    timeval              time_of_last_frame;
    CalibIO              *calib;
    CvMat                *M1x,*M1y,*M2x,*M2y;
    IplImage             *I1_rect,*I2_rect;
    QSettings            *settings;
    int                  frame_number;
    timeval              last_frame_time;
    bool                 stereo_scan;
    float                gain_total;
    bool                 save_single_frame;
    std::string          output_dir;
    std::vector<SaveStereoImageThread*> save_stereo_threads;

private slots:

    void on_shutterSpinBox_valueChanged(int );
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
    void on_resetBusButton_clicked();
    void on_readFromFilesCheckBox_clicked();
    void on_stereoScanButton_clicked();
    void on_exitButton_clicked();
    void on_stopCapturingButton_clicked();
    void on_captureFromFirewireButton_clicked();
    void newStereoImageArrived();
    void newHomographyArrived();
    void newDisparityMapArrived();

};

#endif // MAINDIALOG_H
