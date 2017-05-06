#ifndef SELECTCAMERASDIALOG_H
#define SELECTCAMERASDIALOG_H

#include <QDialog>

#include "framecapturethread.h"
#include "calibio.h"
#include "QSettings"

namespace Ui {
    class SelectCamerasDialog;
}

class SelectCamerasDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SelectCamerasDialog(FrameCaptureThread *cam_left,FrameCaptureThread *cam_right,int shutter,CalibIO *calib,QWidget *parent = 0);
    ~SelectCamerasDialog();

private:
    Ui::SelectCamerasDialog *ui;
    FrameCaptureThread *cam_left;
    FrameCaptureThread *cam_right;
    int shutter;
    QSettings *settings;
    CalibIO *calib;

private slots:
    void on_selectFileButton_clicked();
    void on_buttonBox_accepted();
};

#endif // SELECTCAMERASDIALOG_H
