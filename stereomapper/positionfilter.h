#ifndef POSITIONFILTER_H
#define POSITIONFILTER_H

#include "coordinateconverter.h"

class PositionFilter
{
public:

    /*
    enum GeographicPos
    {
        LAT = 0, // latitude( degree )
        LON,     // longitude( degree )
        ALT,     // altitude( meter )
        GEOGRAPHIC_POS_NUM_COORDS
    };

    enum NEDPos
    {
        NORTH = 0, // meter
        EAST,      // meter
        DOWN,      // meter
        NED_POS_NUM_COORDS
    };
    */

    PositionFilter();

    virtual ~PositionFilter();

    inline Coordinate::GEOD_LLA GetPosition() const { return _position; }

    void SetPosition( const Coordinate::GEOD_LLA& position );

    void UpdatePosition( const Coordinate::GEOD_LLA& delta_position );

private:

    Coordinate::GEOD_LLA _position;
};

#endif // POSITIONFILTER_H
