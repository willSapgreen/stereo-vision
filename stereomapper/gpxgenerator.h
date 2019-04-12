/*
 * Name:        GpxGenerator.h
 * Description: Declaration of GPXGenerator which generates the GPX file.
 * Author(s):   Will Sapgreen [willsapgreen@gmail.com]
 */

#ifndef GPXGENERATOR_H
#define GPXGENERATOR_H

// STL files
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <math.h>

// gpxlib files
#include "gpx/Node.h"
#include "gpx/GPX.h"
#include "gpx/Writer.h"
#include "gpx/ReportCerr.h"
#include "gpx/TRK.h"
#include "gpx/TRKSeg.h"

// TODO:
//     1. namespace.
//     2. C++ 11+ smart pointer.
//     3. buffer for the new position.
class GpxGenerator
{
public:

    const static int EARTH_RADIUS = 6378137;

    /*
     * Default constructor.
     */
    GpxGenerator();

    /*
     * Destructor.
     */
    ~GpxGenerator();

    /*
     * Open a GPX file.
     * [in]  a_GpsFilePath:    the path of the GPS file.
     * [out] true if opening the file works.
     */
    bool open( std::string gpxFilePath );

    /*
     * Check if the file is opened.
     * [out] true if the file is opened.
     */
    bool is_open() const;

    /*
     * Write the result and close a GPS file.
     * [out] true if writing and closing the file works.
     */
    bool close();

    /*
     * Reset the GPX generator.
     * [out] true if reset works.
     */
    bool reset();

    /*
     * Add the new position( latitude and longitude ).
     * [in]  a_Latitude:    the latitude of the new position. Unit: radian.
     * [in]  a_Longtitude:  the longtitude of the new position. Unit: radian.
     * [out] true if adding the new position works.
     */
    bool AddNewPosition( const std::string latitude, const std::string longitude );

    /*
     * Convert Lat/Lon in WGS84 Datum to XY in Spherical Mercrator ESPG:900913
     * [in] Latitude.
     * [in] Longitude.
     * [in/out] x.
     * [in/out] y.
     * [out] flag to indicate if the output xy is valid.
     * Ref: http://www.maptiler.org/google-maps-coordinates-tile-bounds-projection/
     */
    static bool LatLonToMeters(float lat, float lon, float& x, float& y );

    /*
     * Convert XY in Spherical Mercrator ESPG:900913 to Lat/Lon in WGS84 Datum.
     * [in] x.
     * [in] y.
     * [in/out] latitude.
     * [in/out] longitude.
     * [out] flag to indicate if the output lat/lon is valid.
     * Ref: http://www.maptiler.org/google-maps-coordinates-tile-bounds-projection/
     */
    static bool MetersToLatLon(float x, float y, float& lat, float& lon);

private:

    /*
     * The root node of a GPX document.
     */
    gpx::GPX* _GPXRoot;

    /*
     * The report on cerr class for reporting warnings.
     */
    gpx::ReportCerr* _GPXReport;

    /*
     * The track class.
     */
    gpx::TRK* _GPXTrk;

    /*
     * The track segment class.
     */
    gpx::TRKSeg* _GPXTrkSeg;

    /*
     * The output file stream.
     */
    std::ofstream _FileStream;

    /*
     * The written GPX file path.
     */
    std::string _FilePath;

    /*
     * Initialize the GPX root.
     * [out] true if initialization works.
     */
    bool initialize();

    /*
     * Write the stored positions in m_GPXRoot to the file.
     * [out] true if writing to the file works.
     */
    bool write();
};

#endif // GPXGENERATOR_H
