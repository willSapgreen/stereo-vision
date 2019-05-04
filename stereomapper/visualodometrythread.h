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
    std::vector<bool> getInliers() const { return _inliers; }
    StereoImage::simage* getStereoImage() { return _simg; }
    Matrix getHomographyTotal() const { return _H_total; }
    Matrix getHomographyDelta() const { return _H_Delta; }
    float getGain() { return _gain; }
    void resetHomographyTotal() { _H_total.eye(); }
    void resetHomographyDelta() { _H_Delta.eye(); }

    void getVisualOdomStatus( double& roll, double& pitch, double& yaw, double& vel, double& alt )
    {
        roll = _delta_roll; pitch = _delta_pitch; yaw = _delta_yaw; vel = _delta_vel; alt = _delta_alt;
    }

    void pickedUp() { _picked = true; }

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

    std::vector<bool>               _inliers;
    timeval                         _time_prev;
    timeval                         _time_curr;
    bool                            _record_raw_odometry;
    float                           _gain;
    bool                            _picked;


    /**
     * @brief _H_total
     * The transformation which project the point in the current coordinate
     * to the first coordinate.
     *
     * Ex.
     * P_1 = _H_total * P_cur
     * P_cur is the position of the 3D point wrt. the current coordinate.
     * P_1 is the position of the same 3D point wrt. the first coordinate.
     */
    Matrix _H_total;

    // The latest vehicle delta transformation matrix.
    Matrix _H_Delta;

    // Statuses in the NED coordinate extracted from the latest delta transformation.
    double _delta_yaw;
    double _delta_pitch;
    double _delta_roll;
    double _delta_vel;
    double _delta_alt;

signals:

    void newHomographyArrived();

public slots:

};

#endif // VISUALODOMETRYTHREAD_H
