#include "gpsinertialdataiokitti.h"

#define OXTSIO_KITTI_DEBUG 0

GPSInertialDataIOKITTI::GPSInertialDataIOKITTI()
    : _data_size(0)
    , _data_index(-1)
    , _oxts_data_set(nullptr)
{

}

//==============================================================================//

GPSInertialDataIOKITTI::~GPSInertialDataIOKITTI()
{
    _oxts_data_set.release();
}

//==============================================================================//

bool GPSInertialDataIOKITTI::setUpDataPath(const std::string& oxts_data_directory,
                                    const std::string& oxts_timestamp)
{
    // Read the timestamp files.
    std::ifstream time_stamp_stream;
    time_stamp_stream.open(oxts_timestamp);
    if (!time_stamp_stream.is_open())
    {
        std::cout << "ERROR: cannot open oxts timestamp text file" << std::endl;
        return false;
    }


    // Count the lines number in both timestamp files.
    int32_t line_count = std::count( std::istreambuf_iterator<char>(time_stamp_stream),
                                     std::istreambuf_iterator<char>(),
                                     '\n');
    _data_size = line_count;
    time_stamp_stream.clear();
    time_stamp_stream.seekg(0, std::ios::beg);

    // Initialize the oxts data arrays.
    _oxts_data_set = std::unique_ptr<OxSTData[]>(new OxSTData[_data_size]);

    // Read in timestamp.
    std::string lines;
    int count = 0;
    if (time_stamp_stream.is_open())
    {
        while (std::getline(time_stamp_stream, lines))
        {
            //int year = std::stoi(lines[GRAY_LEFT].substr(0,4));
            //int month = std::stoi(lines[GRAY_LEFT].substr(5,2));
            //int day = std::stoi(lines[GRAY_LEFT].substr(8,2));
            int hour = std::stoi(lines.substr(11,2));
            int minute = std::stoi(lines.substr(14,2));
            int second = std::stoi(lines.substr(17,2));
            int narosecond = std::stoi(lines.substr(20,9));

            _oxts_data_set[count].saved_time.tv_sec = hour * 3600 + minute * 60 + second;
            _oxts_data_set[count].saved_time.tv_usec = narosecond / 1000;

            std::ostringstream ss;
            ss << std::setw(10) << std::setfill('0') << count << ".txt";
            std::string oxts_file_path = oxts_data_directory + "//" + ss.str();

#if OXTSIO_KITTI_DEBUG
            std::cout << "Reading in " << ss.str() << std::endl;
#endif

            readInOxTSData(oxts_file_path, _oxts_data_set[count]);
            ++count;
        }
        time_stamp_stream.close();
    }
    else
    {
        std::cout << "ERROR: cannot open OxST timestamp text file" << std::endl;
        return false;
    }

    return true;
}

//==============================================================================//

bool GPSInertialDataIOKITTI::getNextGPSInertialData(OxSTData& gps_inertial_data)
{
    _data_index++;

    return getOxTSData( _data_index, gps_inertial_data );
}

//==============================================================================//

bool GPSInertialDataIOKITTI::getOxTSData(int nth, OxSTData& oxts_data)
{
    // Check if nth is valid.
    if (nth < 0 || nth >= _data_size)
    {
        return false;
    }

    oxts_data = _oxts_data_set[nth];

    return true;
}

//==============================================================================//

bool GPSInertialDataIOKITTI::readInOxTSData(const std::string& oxts_file, OxSTData& oxta_data)
{
    std::ifstream oxta_data_stream;
    oxta_data_stream.open(oxts_file);
    if (!oxta_data_stream.is_open())
    {
        return false;
    }

    std::string lines;
    std::vector<std::string> tokens; // Create vector to hold our words
    while (std::getline(oxta_data_stream, lines))
    {
        std::string buf; // Have a buffer string
        std::stringstream ss(lines); // Insert the string into a stream
        while (ss >> buf)
        {
            tokens.push_back(buf);
        }

        if (30 != tokens.size())
        {
            std::cout << "ERROR: incorrect oxts data" << std::endl;
            break;
        }
    }

    int index = 0;
    oxta_data.oxst.lat = std::stod(tokens[index]); ++index;
    oxta_data.oxst.lon = std::stod(tokens[index]); ++index;
    oxta_data.oxst.alt = std::stod(tokens[index]); ++index;
    oxta_data.oxst.roll = std::stod(tokens[index]); ++index;
    oxta_data.oxst.pitch = std::stod(tokens[index]); ++index;
    oxta_data.oxst.yaw = std::stod(tokens[index]); ++index;
    oxta_data.oxst.vn = std::stod(tokens[index]); ++index;
    oxta_data.oxst.ve = std::stod(tokens[index]); ++index;
    oxta_data.oxst.vf = std::stod(tokens[index]); ++index;
    oxta_data.oxst.vl = std::stod(tokens[index]); ++index;
    oxta_data.oxst.vu = std::stod(tokens[index]); ++index;
    oxta_data.oxst.ax = std::stod(tokens[index]); ++index;
    oxta_data.oxst.ay = std::stod(tokens[index]); ++index;
    oxta_data.oxst.ay = std::stod(tokens[index]); ++index;
    oxta_data.oxst.af = std::stod(tokens[index]); ++index;
    oxta_data.oxst.al = std::stod(tokens[index]); ++index;
    oxta_data.oxst.au = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wx = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wy = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wz = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wf = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wl = std::stod(tokens[index]); ++index;
    oxta_data.oxst.wu = std::stod(tokens[index]); ++index;
    oxta_data.oxst.pos_accuracy = std::stod(tokens[index]); ++index;
    oxta_data.oxst.vel_accuracy = std::stod(tokens[index]); ++index;
    oxta_data.oxst.navstat = std::stoi(tokens[index]); ++index;
    oxta_data.oxst.numsats = std::stoi(tokens[index]); ++index;
    oxta_data.oxst.posmode = std::stoi(tokens[index]); ++index;
    oxta_data.oxst.velmode = std::stoi(tokens[index]); ++index;
    oxta_data.oxst.orimode = std::stoi(tokens[index]); ++index;

    oxta_data_stream.close();
    return true;
}
