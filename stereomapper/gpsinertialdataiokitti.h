/*
 * Name:        gpsinertialdataiokitti.h
 * Description: Interface to read OxTS GPS/IMU processed data in KITTI.
 * Will Huang [willsapgreen@gmail.com]
 */

#ifndef GPS_INERTIAL_DATAKITTI_H
#define GPS_INERTIAL_DATAKITTI_H

// std
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <memory>
#include <vector>
#include <algorithm>

#include "gpsinertialdata.h"

struct OxSTFormat
{
    double lat; //latitude of the oxts-unit (deg)
    double lon; //longitude of the oxts-unit (deg)
    double alt; //altitude of the oxts-unit (m)
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
};

struct OxSTData
{
    OxSTFormat oxst;
    timeval saved_time;

    void toGPSInertialDataFormat(GPSInertialDataFormat& data)
    {
        data.lat = oxst.lat;
        data.lon = oxst.lon;
        data.alt = oxst.alt;
        data.roll = oxst.roll;
        data.pitch = oxst.pitch;
        data.yaw = oxst.yaw;
        data.vn = oxst.vn;
        data.ve = oxst.ve;
        data.vf = oxst.vf;
        data.vl = oxst.vl;
        data.vu = oxst.vu;
        data.ax = oxst.ax;
        data.ay = oxst.ay;
        data.az = oxst.az;
        data.af = oxst.af;
        data.al = oxst.al;
        data.au = oxst.au;
        data.wx = oxst.wx;
        data.wy = oxst.wy;
        data.wz = oxst.wz;
        data.wf = oxst.wf;
        data.wl = oxst.wl;
        data.wu = oxst.wu;
        data.pos_accuracy = oxst.pos_accuracy;
        data.vel_accuracy = oxst.vel_accuracy;
        data.navstat = oxst.navstat;
        data.numsats = oxst.numsats;
        data.posmode = oxst.posmode;
        data.velmode = oxst.velmode;
        data.orimode = oxst.orimode;
        data.saved_time = saved_time;
    }
};

class GPSInertialDataIOKITTI
{
public:
    /*
     * Default constructor.
     */
    GPSInertialDataIOKITTI();

    /*
     * Default destructor.
     */
    virtual ~GPSInertialDataIOKITTI();

    /*
     * Set up the GPS-Inertial data and timestamp path.
     */
    bool setUpDataPath(const std::string& oxts_data_directory,
                           const std::string& oxts_timestamp);

    /*
     * Get the size of the storage.
     */
    inline int32_t getOxTSDataSize() const
    {
        return _data_size;
    }

    /*
     * Get the next GPS-Inertial data.
     */
    bool getNextGPSInertialData(OxSTData& gps_inertial_data);

private:

    int32_t _data_size;

    int32_t _data_index;

    std::unique_ptr<OxSTData[]> _oxts_data_set;

    /*
     * Read in GPS/IMU processed data.
     */
    bool readInOxTSData(const std::string& oxts_file, OxSTData& oxta_data);

    /*
     * Get the nth OxTS data.
     * 0 <= nth < the size of the storage.
     */
    bool getOxTSData(int nth, OxSTData& oxts_data);
};

#endif // OXTSIOKITTI_H
