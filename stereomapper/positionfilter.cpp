#include "positionfilter.h"

PositionFilter::PositionFilter()
    : _position(0,0,0)
{

}

//==============================================================================//

PositionFilter::~PositionFilter()
{

}

//==============================================================================//

void PositionFilter::SetPosition( const Coordinate::GEOD_LLA& position )
{
    _position = position;
}

//==============================================================================//

void PositionFilter::UpdatePosition( const Coordinate::GEOD_LLA& delta_position )
{
    _position._alt += delta_position._alt;
    _position._lat += delta_position._lat;
    _position._lon += delta_position._lon;
}
