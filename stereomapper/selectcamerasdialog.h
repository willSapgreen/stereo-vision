#ifndef SELECTCAMERASDIALOG_H
#define SELECTCAMERASDIALOG_H

#include <QDialog>

#include "framecapturethread.h"
#include "calibiokitti.h"
#include "QSettings"

namespace Ui
{
    class SelectCamerasDialog;
}

class SelectCamerasDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SelectCamerasDialog(FrameCaptureThread *cam_left,FrameCaptureThread *cam_right,
                                 int shutter,CalibIOKITTI *calib,QWidget *parent = 0);
    ~SelectCamerasDialog();

private:
    Ui::SelectCamerasDialog *ui;
    FrameCaptureThread *cam_left;
    FrameCaptureThread *cam_right;
    int shutter;
    QSettings *settings;
    CalibIOKITTI *calib;

private slots:

    /*
     * Handle when clicking the select file button and then the window will pop up to allow
     * users to select the cameras calibration file.
     */
    void on_selectFileButton_clicked();

    void on_buttonBox_accepted();
};

#endif // SELECTCAMERASDIALOG_H
