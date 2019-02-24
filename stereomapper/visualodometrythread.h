#ifndef VISUALODOMETRYTHREAD_H
#define VISUALODOMETRYTHREAD_H

#include <QThread>

#include <list>
#include "calibiokitti.h"
#include "stereoimage.h"
#include "../libviso2/src/viso_stereo.h"
#include "../libviso2/src/timer.h"

class VisualOdometryThread : public QThread
{
    Q_OBJECT

public:

    VisualOdometryThread(CalibIOKITTI *calib,QObject *parent = 0);
    ~VisualOdometryThread();
    void pushBack(StereoImage::simage& s,bool record_raw_odometry=false);
    std::vector<Matcher::p_match> getMatches() { return _visualOdomStereo->getMatches(); }
    std::vector<bool> getInliers() { return _inliers; }
    StereoImage::simage* getStereoImage() { return _simg; }
    Matrix getHomographyTotal() { return _H_total; }
    float getGain() { return _gain; }
    void resetHomographyTotal() { _H_total.eye(); }
    void pickedUp() { _picked = true; }

    // The latest( accmulated ) transformation matrix.
    // current_transformation = previous_transformation * new_delta_transformation
    Matrix _H_total;

protected:

    void run();

private:

    float timeDiff(timeval a,timeval b)
    {
        return ((float)(a.tv_sec -b.tv_sec )) +
               ((float)(a.tv_usec-b.tv_usec))*1e-6;
    }

    VisualOdometryStereo* _visualOdomStereo;
    StereoImage::simage*  _simg;
    CalibIOKITTI*         _calib;
    //Matcher                      *matcher;

    //std::vector<Matcher::p_match>   matches;
    std::vector<bool>               _inliers;
    timeval                         _time_prev;
    timeval                         _time_curr;
    bool                            _record_raw_odometry;
    float                           _gain;
    bool                            _picked;

signals:

    void newHomographyArrived();

public slots:

};

#endif // VISUALODOMETRYTHREAD_H
