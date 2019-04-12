/*
 * Name:        GpxGenerator.h
 * Description: Implementation of GPXGenerator which generates the GPX file.
 * Author(s):   Will Sapgreen [willsapgreen@gmail.com]
 */

#include "gpxgenerator.h"

GpxGenerator::GpxGenerator()
{
    //reset();
    initialize();
}

//==============================================================================//

GpxGenerator::~GpxGenerator()
{
    reset();
}

//==============================================================================//

bool GpxGenerator::open( std::string gpxFilePath )
{
    // When the generator is on processing.
    if( _FileStream.is_open() )
    {
        close();
    }

    initialize();
    _FilePath = gpxFilePath;
    _FileStream.open( _FilePath.c_str() );

    _GPXTrk = dynamic_cast<gpx::TRK*>( _GPXRoot->trks().add( _GPXReport ) );
    _GPXTrk->name().add( _GPXReport )->setValue( _FilePath.c_str() );
    _GPXTrkSeg = dynamic_cast<gpx::TRKSeg*>( _GPXTrk->trksegs().add( _GPXReport ) );
    return true;
}

//==============================================================================//

bool GpxGenerator::is_open() const
{
    return _FileStream.is_open();
}

//==============================================================================//

bool GpxGenerator::close()
{
    return write() && reset();
}

//==============================================================================//

bool GpxGenerator::AddNewPosition(const std::string latitude, const std::string longitude)
{
    gpx::WPT *trkpt = dynamic_cast<gpx::WPT*>( _GPXTrkSeg->trkpts().add( _GPXReport ) );
    trkpt->lat().add( _GPXReport )->setValue( latitude );
    trkpt->lon().add( _GPXReport )->setValue( longitude );
    return true;
}

//==============================================================================//

bool GpxGenerator::initialize()
{
    _GPXRoot = new gpx::GPX();
    _GPXReport = new gpx::ReportCerr();
    _GPXRoot->add( "xmlns", gpx::Node::ATTRIBUTE )->setValue( "http://www.topografix.com/GPX/1/1" ); // Some tools need this
    _GPXRoot->version().add( _GPXReport )->setValue( "1.1") ;
    _GPXRoot->creator().add( _GPXReport )->setValue( "GPX" );
    return true;
}

//==============================================================================//

bool GpxGenerator::reset()
{
    if( _FileStream.is_open() )
    {
        _FileStream.close();
    }

    if( _GPXRoot != NULL )
    {
        delete _GPXRoot;
        _GPXRoot = NULL;
    }

    if( _GPXReport != NULL )
    {
        delete _GPXReport;
        _GPXReport = NULL;
    }

    /*
    if( _GPXTrk != NULL )
    {
        delete _GPXTrk;
        _GPXTrk = NULL;
    }

    if( _GPXTrkSeg != NULL )
    {
        delete _GPXTrkSeg;
        _GPXTrkSeg = NULL;
    }
    */

    _FilePath = "";

    return true;
}

//==============================================================================//

bool GpxGenerator::write()
{
    gpx::Writer writer;
    if( _FileStream.is_open() )
    {
        writer.write( _FileStream, _GPXRoot, true );
    }
    return true;
}

//==============================================================================//

bool GpxGenerator::LatLonToMeters(float lat, float lon, float& x, float& y )
{
    bool is_return_valid = true;

    x = lon * M_PI * EARTH_RADIUS / 180.0F;
    y = static_cast<float>(log(tan((90.0 + lat) * M_PI / 360.0)) / (M_PI / 180.0));
    y = y * M_PI * EARTH_RADIUS / 180.0F;

    return is_return_valid;
}

//==============================================================================//

bool GpxGenerator::MetersToLatLon(float x, float y, float& lat, float& lon)
{
    bool is_return_valid = true;

    lon = (x / (M_PI * EARTH_RADIUS)) * 180.0F;
    lat = (y / (M_PI * EARTH_RADIUS)) * 180.0F;
    lat = 180.0F / M_PI * (2 * atan(exp(lat * M_PI / 180)) - M_PI / 2.0F);

    return is_return_valid;
}
