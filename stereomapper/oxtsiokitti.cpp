#include "oxtsiokitti.h"

OxTSIOKITTI::OxTSIOKITTI()
    : _data_size(0)
    , _oxts_data_set(nullptr)
{

}

//==============================================================================//

OxTSIOKITTI::~OxTSIOKITTI()
{
    _oxts_data_set.release();
}

//==============================================================================//

bool OxTSIOKITTI::fetchGrayOxTSData(const std::string& oxts_data_directory,
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

            _oxts_data_set[count]._captured_time.tv_sec = hour * 3600 + minute * 60 + second;
            _oxts_data_set[count]._captured_time.tv_usec = narosecond / 1000;

            std::ostringstream ss;
            ss << std::setw(10) << std::setfill('0') << count << ".txt";
            std::string oxts_file_path = oxts_data_directory + "//" + ss.str();

            std::cout << "Reading in " << ss.str() << std::endl;

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

bool OxTSIOKITTI::getOxTSData(int nth, OxSTData& oxts_data)
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

bool OxTSIOKITTI::readInOxTSData(const std::string& oxts_file, OxSTData& oxta_data)
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
    oxta_data._oxst._lat = std::stod(tokens[index]); ++index;
    oxta_data._oxst._lon = std::stod(tokens[index]); ++index;
    oxta_data._oxst._alt = std::stod(tokens[index]); ++index;
    oxta_data._oxst._roll = std::stod(tokens[index]); ++index;
    oxta_data._oxst._pitch = std::stod(tokens[index]); ++index;
    oxta_data._oxst._yaw = std::stod(tokens[index]); ++index;
    oxta_data._oxst._vn = std::stod(tokens[index]); ++index;
    oxta_data._oxst._ve = std::stod(tokens[index]); ++index;
    oxta_data._oxst._vf = std::stod(tokens[index]); ++index;
    oxta_data._oxst._vl = std::stod(tokens[index]); ++index;
    oxta_data._oxst._vu = std::stod(tokens[index]); ++index;
    oxta_data._oxst._ax = std::stod(tokens[index]); ++index;
    oxta_data._oxst._ay = std::stod(tokens[index]); ++index;
    oxta_data._oxst._ay = std::stod(tokens[index]); ++index;
    oxta_data._oxst._af = std::stod(tokens[index]); ++index;
    oxta_data._oxst._al = std::stod(tokens[index]); ++index;
    oxta_data._oxst._au = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wx = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wy = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wz = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wf = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wl = std::stod(tokens[index]); ++index;
    oxta_data._oxst._wu = std::stod(tokens[index]); ++index;
    oxta_data._oxst._pos_accuracy = std::stod(tokens[index]); ++index;
    oxta_data._oxst._vel_accuracy = std::stod(tokens[index]); ++index;
    oxta_data._oxst._navstat = std::stoi(tokens[index]); ++index;
    oxta_data._oxst._numsats = std::stoi(tokens[index]); ++index;
    oxta_data._oxst._posmode = std::stoi(tokens[index]); ++index;
    oxta_data._oxst._velmode = std::stoi(tokens[index]); ++index;
    oxta_data._oxst._orimode = std::stoi(tokens[index]); ++index;

    oxta_data_stream.close();
    return true;
}
