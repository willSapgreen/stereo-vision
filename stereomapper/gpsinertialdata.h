#ifndef GPS_INERTIAL_DATA_H
#define GPS_INERTIAL_DATA_H

#include <QObject>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

// TODO:
// Need mutex for read/write when being used in the multi-thread situation.

struct GPSInertialDataFormat
{
    double lat; //latitude (deg)
    double lon; //longitude (deg)
    double alt; //altitude (m)
    double roll; //roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    double pitch; //pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    double yaw; //heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    double vn; //velocity towards north (m/s)
    double ve; //velocity towards east (m/s)
    double vf; //forward velocity, i.e. parallel to earth-surface (m/s)
    double vl; //leftward velocity, i.e. parallel to earth-surface (m/s)
    double vu; //upward velocity, i.e. perpendicular to earth-surface (m/s)
    double ax; //acceleration in x, i.e. in direction of vehicle front (m/s^2)
    double ay; //acceleration in y, i.e. in direction of vehicle left (m/s^2)
    double az; //acceleration in z, i.e. in direction of vehicle top (m/s^2)
    double af; //forward acceleration (m/s^2)
    double al; //leftward acceleration (m/s^2)
    double au; //upward acceleration (m/s^2)
    double wx; //angular rate around x (rad/s)
    double wy; //angular rate around y (rad/s)
    double wz; //angular rate around z (rad/s)
    double wf; //angular rate around forward axis (rad/s)
    double wl; //angular rate around leftward axis (rad/s)
    double wu; //angular rate around upward axis (rad/s)
    double pos_accuracy; //velocity accuracy (north/east in m)
    double vel_accuracy; //velocity accuracy (north/east in m/s)
    int32_t navstat; //navigation status (see navstat_to_string)
    int32_t numsats; //number of satellites tracked by primary GPS receiver
    int32_t posmode; //position mode of primary GPS receiver (see gps_mode_to_string)
    int32_t velmode; //velocity mode of primary GPS receiver (see gps_mode_to_string)
    int32_t orimode; //orientation mode of primary GPS receiver (see gps_mode_to_string)
    timeval saved_time; // when data is saved.
};

class GPSInertialData : public QObject
{
    Q_OBJECT

public:

    GPSInertialData(QObject *parent=0);

    virtual ~GPSInertialData();

    void setData(const GPSInertialDataFormat& gps_inertial_data);

    inline GPSInertialDataFormat getData() { return _data; }

    void pickedUp() { _picked = true; }

private:

    bool _picked;

    GPSInertialDataFormat _data;

signals:

    void newGPSInertialDataArrived();
};

#endif // GPS_INERTIAL_DATA_H
