#ifndef READFROMFILESTHREAD_H
#define READFROMFILESTHREAD_H

#include <QThread>
#include <QString>
#include <opencv/highgui.h>

#include "calibio.h"
#include "stereoimage.h"

class ReadFromFilesThread : public QThread {

    Q_OBJECT

public:

    ReadFromFilesThread(StereoImage *stereo_image,CalibIO *calib,QObject *parent = 0);
    ~ReadFromFilesThread();
    void setInputDir(QString input_dir_) {input_dir = input_dir_;}

protected:

    void run();

private:

    CalibIO     *calib;
    StereoImage *stereo_image;

    std::vector<IplImage*> I1;
    std::vector<IplImage*> I2;
    QString input_dir;

signals:

public slots:

};

#endif // READFROMFILESTHREAD_H
