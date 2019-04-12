#include "headingfilter.h"



HeadingFilter::HeadingFilter()
{

}

//==============================================================================//

HeadingFilter::~HeadingFilter()
{

}

//==============================================================================//

void HeadingFilter::SetHeading( double heading )
{
    _heading = heading;

    // check the boundary condition
    _heading = _heading > M_PI ? -M_PI + ( _heading - M_PI ) : _heading;
    _heading = _heading < -M_PI ? M_PI + ( _heading - (-M_PI) ) : _heading;
}

//==============================================================================//

void HeadingFilter::UpdateHeading( double delta_heading )
{
    // update the previous heading.
    _heading += delta_heading;

    // check the boundary condition
    _heading = _heading > M_PI ? -M_PI + ( _heading - M_PI ) : _heading;
    _heading = _heading < -M_PI ? M_PI + ( _heading - (-M_PI) ) : _heading;
}
