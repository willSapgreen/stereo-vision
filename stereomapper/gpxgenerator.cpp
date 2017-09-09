/*
 * Name:        GpxGenerator.h
 * Description: Implementation of GPXGenerator which generates the GPX file.
 * Author(s):   Will Sapgreen [willsapgreen@gmail.com]
 */

#include "gpxgenerator.h"

GpxGenerator::GpxGenerator()
{
    reset();
    initialize();
}

//==============================================================================//

GpxGenerator::~GpxGenerator()
{
    reset();
}

//==============================================================================//

bool GpxGenerator::open( std::string a_GpxFilePath )
{
    // When the generator is on processing.
    if( m_FileStream.is_open() )
    {
        close();
    }

    initialize();
    m_FilePath = a_GpxFilePath;
    m_FileStream.open( m_FilePath.c_str() );

    m_GPXTrk = dynamic_cast<gpx::TRK*>( m_GPXRoot->trks().add( m_GPXReport ) );
    m_GPXTrk->name().add( m_GPXReport )->setValue( m_FilePath.c_str() );
    m_GPXTrkSeg = dynamic_cast<gpx::TRKSeg*>( m_GPXTrk->trksegs().add( m_GPXReport ) );
    return true;
}

//==============================================================================//

bool GpxGenerator::is_open() const
{
    return m_FileStream.is_open();
}

//==============================================================================//

bool GpxGenerator::close()
{
    return write() && reset();
}

//==============================================================================//

bool GpxGenerator::AddNewPosition(const std::string a_Latitude, const std::string a_Longitude)
{
    gpx::WPT *trkpt = dynamic_cast<gpx::WPT*>( m_GPXTrkSeg->trkpts().add( m_GPXReport ) );
    trkpt->lat().add( m_GPXReport )->setValue( a_Latitude );
    trkpt->lon().add( m_GPXReport )->setValue( a_Longitude );
    return true;
}

//==============================================================================//

bool GpxGenerator::initialize()
{
    m_GPXRoot = new gpx::GPX();
    m_GPXReport = new gpx::ReportCerr();
    m_GPXRoot->add( "xmlns", gpx::Node::ATTRIBUTE )->setValue( "http://www.topografix.com/GPX/1/1" ); // Some tools need this
    m_GPXRoot->version().add( m_GPXReport )->setValue( "1.1") ;
    m_GPXRoot->creator().add( m_GPXReport )->setValue( "GPX" );
    return true;
}

//==============================================================================//

bool GpxGenerator::reset()
{
    if( m_FileStream.is_open() )
    {
        m_FileStream.close();
    }

    if( m_GPXRoot != NULL )
    {
        delete m_GPXRoot;
        m_GPXRoot = NULL;
    }

    if( m_GPXReport != NULL )
    {
        delete m_GPXReport;
        m_GPXReport = NULL;
    }

    if( m_GPXTrk != NULL )
    {
        delete m_GPXTrk;
        m_GPXTrk = NULL;
    }

    if( m_GPXTrkSeg != NULL )
    {
        delete m_GPXTrkSeg;
        m_GPXTrkSeg = NULL;
    }

    m_FilePath = "";

    return true;
}

//==============================================================================//

bool GpxGenerator::write()
{
    gpx::Writer writer;
    if( m_FileStream.is_open() )
    {
        writer.write( m_FileStream, m_GPXRoot, true );
    }
    return true;
}
