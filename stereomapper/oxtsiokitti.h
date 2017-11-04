/*
 * Name:        oxtsiokitti.h
 * Description: Interface to read OxTS GPS/IMU processed data.
 * Will Huang [willsapgreen@gmail.com]
 */

#ifndef OXTSIOKITTI_H
#define OXTSIOKITTI_H


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

struct OxST
{
    double _lat; //latitude of the oxts-unit (deg)
    double _lon; //longitude of the oxts-unit (deg)
    double _alt; //altitude of the oxts-unit (m)
    double _roll; //roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    double _pitch; //pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    double _yaw; //heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    double _vn; //velocity towards north (m/s)
    double _ve; //velocity towards east (m/s)
    double _vf; //forward velocity, i.e. parallel to earth-surface (m/s)
    double _vl; //leftward velocity, i.e. parallel to earth-surface (m/s)
    double _vu; //upward velocity, i.e. perpendicular to earth-surface (m/s)
    double _ax; //acceleration in x, i.e. in direction of vehicle front (m/s^2)
    double _ay; //acceleration in y, i.e. in direction of vehicle left (m/s^2)
    double _az; //acceleration in z, i.e. in direction of vehicle top (m/s^2)
    double _af; //forward acceleration (m/s^2)
    double _al; //leftward acceleration (m/s^2)
    double _au; //upward acceleration (m/s^2)
    double _wx; //angular rate around x (rad/s)
    double _wy; //angular rate around y (rad/s)
    double _wz; //angular rate around z (rad/s)
    double _wf; //angular rate around forward axis (rad/s)
    double _wl; //angular rate around leftward axis (rad/s)
    double _wu; //angular rate around upward axis (rad/s)
    double _pos_accuracy; //velocity accuracy (north/east in m)
    double _vel_accuracy; //velocity accuracy (north/east in m/s)
    int32_t _navstat; //navigation status (see navstat_to_string)
    int32_t _numsats; //number of satellites tracked by primary GPS receiver
    int32_t _posmode; //position mode of primary GPS receiver (see gps_mode_to_string)
    int32_t _velmode; //velocity mode of primary GPS receiver (see gps_mode_to_string)
    int32_t _orimode; //orientation mode of primary GPS receiver (see gps_mode_to_string)
};

struct OxSTData
{
    OxST _oxst;
    timeval _captured_time;
};

class OxTSIOKITTI
{
public:
    /*
     * Default constructor.
     */
    OxTSIOKITTI();

    /*
     * Default destructor.
     */
    virtual ~OxTSIOKITTI();

    /*
     * Process the OxTS data and timestamp.
     */
    bool fetchGrayOxTSData(const std::string& oxts_data_directory,
                           const std::string& oxts_timestamp);

    /*
     * Get the size of the storage.
     */
    inline int32_t getOxTSDataSize() const
    {
        return _data_size;
    }

    /*
     * Get the nth OxTS data.
     * 0 <= nth < the size of the storage.
     */
    bool getOxTSData(int nth, OxSTData& oxts_data);

private:
    int32_t _data_size;
    std::unique_ptr<OxSTData[]> _oxts_data_set;

    /*
     * Read in GPS/IMU processed data.
     */
    bool readInOxTSData(const std::string& oxts_file, OxSTData& oxta_data);
};

#endif // OXTSIOKITTI_H
