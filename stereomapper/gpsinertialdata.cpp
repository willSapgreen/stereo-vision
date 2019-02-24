#include "gpsinertialdata.h"

GPSInertialData::GPSInertialData(QObject *parent):QObject(parent)
{
    _picked = false;
}

//==============================================================================//

GPSInertialData::~GPSInertialData()
{
    //Do nothing for now.
}

//==============================================================================//

void GPSInertialData::setData(const GPSInertialDataFormat& gps_inertial_data)
{
    // Just copy the data for the current design.
    _data = gps_inertial_data;

    _picked = false;
    emit newGPSInertialDataArrived();
    while (!_picked) usleep(1000);
}
