#ifndef HEADINGFILTER_H
#define HEADINGFILTER_H

#include <math.h>

class HeadingFilter
{
public:
    HeadingFilter();

    virtual ~HeadingFilter();

    inline double GetHeading() const { return _heading; }

    void SetHeading( double heading );

    void UpdateHeading( double delta_heading );

private:

    /*
     * The current heading(yaw)
     * unit: rad
     * 0: rad east
     * positive: counter clockwise
     * range: from -pi to +pi
     */
    double _heading;
};

#endif // HEADINGFILTER_H
