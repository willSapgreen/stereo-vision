#ifndef COORDINATECONVERTER_H
#define COORDINATECONVERTER_H

#include <math.h>
#include <../libviso2/src/matrix.h>

namespace Coordinate
{
    const double WGS84_A = 6378137.0; // unit: meter
    const double WGS84_B = 6356752.31424; // unit: meter
    const double WGS84_ONE_F_TH = 298.257223563;
    const double WGS84_F = 1 / WGS84_ONE_F_TH;
    const double WGS84_E = 1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A); // WGS84 first eccentricity squared
    const double R2D = 180.0 / M_PI ;
    const double D2R = M_PI / 180.0;

    //==============================================================================//

    /*
     * Geodetic coordinate:
     * Latitude (radian)
     * Longitude (radian)
     * Altitude (meter)
     */
    struct GEOD_LLA
    {
        GEOD_LLA(double lat, double lon, double alt)
            : _lat(lat)
            , _lon(lon)
            , _alt(alt)
        {}

        double _lat;
        double _lon;
        double _alt;
    };

    //==============================================================================//

    struct ECEF_XYZ
    {
        ECEF_XYZ(double x, double y, double z)
            : _x(x)
            , _y(y)
            , _z(z)
        {}

        double _x;
        double _y;
        double _z;
    };

    //==============================================================================//

    struct LOCAL_NED
    {
        LOCAL_NED(double north, double east, double down)
            : _north(north)
            , _east(east)
            , _down(down)
        {}

        double _north;
        double _east;
        double _down;
    };

    //==============================================================================//

    struct BODY_XYZ
    {
        BODY_XYZ(double x, double y, double z)
            : _x(x)
            , _y(y)
            , _z(z)
        {}

        double _x;
        double _y;
        double _z;
    };

    //==============================================================================//

    /*
     * Convert GEOD_LLA to ECEF_XYZ
     */
    inline void convertWGS84ToECEF( const GEOD_LLA& lla, ECEF_XYZ& xyz )
    {
        double val = WGS84_E * lla._lat;
        double radius_of_curvature = WGS84_A / sqrt( 1.0 - ( val ) * ( val ) );
        xyz._x = ( radius_of_curvature + lla._alt ) * cos( lla._lat ) * cos( lla._lon );
        xyz._y = ( radius_of_curvature + lla._alt ) * cos( lla._lat ) * sin( lla._lon );
        xyz._z = ( ( 1 - WGS84_E * WGS84_E ) * radius_of_curvature + lla._alt ) * sin( lla._lat );
    }

    //==============================================================================//

    /*
     * Convert ECEF_XYZ to GEOD_LLA
     * Algorithm I : "Transformation from Cartesian to Geodetic Coordinates Accelerated by
     * Halley’s Method", T. Fukushima (2006), Journal of Geodesy.
     * Reference:
     * "A comparison of methods used in rectangular to Geodetic Coordinates
     * Transformations", Burtch R. R. (2006), American Congress for Surveying
     * and Mapping Annual Conference. Orlando, Florida.
     */
    /*
    void convertECEFToWGS84_T_Fukushima( const ECEF_XYZ& xyz, GEOD_LLA& lla )
    {
        double dist_from_polar_axis = sqrt( xyz._x * xyz._x + xyz._y * xyz._y );
        const double MINIMUM_DISTANCE_THRESHOLD = 1e-8;
        const int ITERATION_TIME = 10;

        lla._lon = 0;
        if( ( dist_from_polar_axis - 0 ) <= MINIMUM_DISTANCE_THRESHOLD )
        {
            lla._lon = atan2(xyz._y, xyz._x);
        }

        // Special Case:
        // Convergence is very slow if the position is close to the pole.
        if( dist_from_polar_axis < ( WGS84_A * 1e-16 ) )
        {
            lla._lat = copysign( M_PI_2, xyz._z );
            lla._alt = fabs( xyz._z ) - WGS84_B;
            return;
        }

        double normalized_p = dist_from_polar_axis / WGS84_A;
        double normalized_z = fabs( xyz._z ) * e_c / WGS84_A;
        double e_c = sqrt(1.0 - WGS84_E * WGS84_E);
        double z_c = e_c * fabs( normalized_z );
        double c = WGS84_A * WGS84_E;

        // initialize s and c to a zero height solution.
        // where tan(psi) = s/c, the tangent of the reduced latitude.
        // psi is the reduced latitude defined in the paper.
        double cur_s = normalized_z;
        double cur_c = e_c * normalized_p;

        double a = 0;
        double b = 0;
        for( int i = 0; i < ITERATION_TIME; ++i )
        {
            a = sqrt( cur_s * cur_s + cur_c * cur_c );
            b = 1.5 * c * cur_s * cur_c * ( ( normalized_p * cur_s - normalized_z * cur_c ) * a - c * cur_s * cur_c );

            // update
            double pow_a_3 = pow( a, 3.0 );
            cur_s = ( normalized_z * pow_a_3 + c * pow( cur_s, 3.0 ) ) * pow_a_3 - b * cur_s;
            cur_c = ( normalized_p * pow_a_3 + c * pow( cur_c, 3.0 ) ) * pow_a_3 - b * cur_c;

            // In https://github.com/swift-nav/libswiftnav/blob/master/src/coord_system.c
            // the author proposes an approach to resolve the over/underflow issue in T. Fukushima's algorithm.
            // TODO: Over/underflow issue implementation
        }


        lla._lat = copysign( 1.0, xyz._z * atan( cur_s / ( e_c * cur_c ) ) );
        double e_c_cur_c = e_c * cur_c;
        lla._alt = ( normalized_p * e_c_cur_c + fabs( xyz._z ) * cur_s - WGS84_A * sqrt( e_c * e_c * cur_s * cur_s + e_c_cur_c * e_c_cur_c ) )\
                / sqrt( e_c_cur_c * e_c_cur_c + cur_s * cur_s );
    }
    */

    /*
     * Populate a 3x3 ECEF to NED coordinates matrix, given the LLA reference vector
     */
    /*
    void populateECEFToNEDMatrix( const GEOD_LLA& lla, Matrix& transformation_matrix )
    {
        double sin_lat = sin(llh[0]);
        double cos_lat = cos(llh[0]);
        double sin_lon = sin(llh[1]);
        double cos_lon = cos(llh[1]);
        transformation_matrix._val[0][0] = -sin_lat * cos_lon;
        transformation_matrix._val[0][1] = -sin_lat * sin_lon;
        transformation_matrix._val[0][2] = cos_lat;
        transformation_matrix._val[1][0] = -sin_lon;
        transformation_matrix._val[1][1] = cos_lon;
        transformation_matrix._val[1][2] = 0.0;
        transformation_matrix._val[2][0] = -cos_lat * cos_lon;
        transformation_matrix._val[2][1] = -cos_lat * sin_lon;
        transformation_matrix._val[2][2] = -sin_lat;
    }
    */

    /**
     * @brief calculateRadiusCurvature
     *        Calculate the meridional radius of curvature
     *        and the radius of curvature in the prime vertical, respectively.
     *        Ref: http://www.edwilliams.org/avform.htm#flat
     *             Local, flat earth approximation
     * @param lla: [in] the given fixed point
     * @param r1: [in/out] the meridional radius of curvature
     * @param r2: [in/out] the radus of curvature in the prime vertical
     */
    inline void calculateRadiusCurvature( const GEOD_LLA& lla, double& r1, double& r2 )
    {
        const double E2 = WGS84_F * ( 2 - WGS84_F );
        const double DENOMINATOR = sqrt( ( 1 - E2 * ( sin(lla._lat) * sin(lla._lat) ) ) );
        r1 = ( WGS84_A * ( 1 - E2 ) ) / ( pow( DENOMINATOR, 3 ) );
        r2 = WGS84_A / DENOMINATOR;
    }

    /**
     * @brief calculateScaleLLToNE:
     *        Calculate the scales which convert delta latitude and delta longitude to
     *        delta north distance and delta east distance
     * @param lla: [in] the given fixed point
     * @param lat_scale: [in/out] the latitude to north scale
     * @param lon_scale: [in/out] the longitude to east scale
     */
    inline void calculateScaleLLToNE( const GEOD_LLA& lla, double& lat_scale, double& lon_scale )
    {
        double r1;
        double r2;
        calculateRadiusCurvature( lla, r1, r2 );
        lat_scale = r1;
        lon_scale = r2 * cos(lla._lat);
    }

}

#endif // COORDINATECONVERTER_H
