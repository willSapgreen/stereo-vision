#include "stereoimageiokitti.h"

#define STEREO_IMAGE_IO_KITTI_DEBUG 1

StereoImageIOKITTI::StereoImageIOKITTI()
    : _images_number(0)
    , _image_index(-1)
{
}

//==============================================================================//

StereoImageIOKITTI::~StereoImageIOKITTI()
{
    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        _time_stamp_stream[i].close();
    }
}

//==============================================================================//

bool StereoImageIOKITTI::setUpDataPath(const std::string images_directory[],
                                       const std::string images_timestamp[])
{
    // Read the timestamp files.
    bool is_open = true;
    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        _time_stamp_stream[i].open(images_timestamp[i]);
        is_open = (is_open && _time_stamp_stream[i].is_open());

        _images_directory[i] = images_directory[i];
    }

    if(false == is_open)
    {
        std::cout << "ERROR: cannot open timestamp text file" << std::endl;
        return false;
    }

    // Count the lines number in each timestamp files.
    int32_t line_count_in_each_source[IMAGE_INPUT_SOURCE_COUNT] = {0};
    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        line_count_in_each_source[i] = std::count( std::istreambuf_iterator<char>( _time_stamp_stream[i] ),
                                                   std::istreambuf_iterator<char>(),
                                                   '\n');
    }

    bool is_equal = true;
    for(int i = 1; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        is_equal = (is_equal && (line_count_in_each_source[0] == line_count_in_each_source[i]));
    }

    if(false == is_equal)
    {
        std::cout << "ERROR: timestamp lines counts are not consistent" << std::endl;
        return false;
    }
    _images_number = line_count_in_each_source[0];

    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        _time_stamp_stream[i].clear();
        _time_stamp_stream[i].seekg(0, std::ios::beg);
    }

    return true;
}

//==============================================================================//

bool StereoImageIOKITTI::getNextImageDataSet(ImageDataCV image_set[IMAGE_INPUT_SOURCE_COUNT])
{
    bool is_get_lines = true;
    std::string lines[IMAGE_INPUT_SOURCE_COUNT];
    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        is_get_lines = ( is_get_lines && std::getline(_time_stamp_stream[i], lines[i] ) );
    }
    if(false == is_get_lines)
    {
        return false;
    }

    _image_index++;
    for(int i = 0; i < IMAGE_INPUT_SOURCE_COUNT; ++i)
    {
        //int year = std::stoi(lines[GRAY_LEFT].substr(0,4));
        //int month = std::stoi(lines[GRAY_LEFT].substr(5,2));
        //int day = std::stoi(lines[GRAY_LEFT].substr(8,2));
        int hour = std::stoi(lines[i].substr(11,2));
        int minute = std::stoi(lines[i].substr(14,2));
        int second = std::stoi(lines[i].substr(17,2));
        int narosecond = std::stoi(lines[i].substr(20,9));
        image_set[i]._captured_time.tv_sec = hour * 3600 + minute * 60 + second;
        image_set[i]._captured_time.tv_usec = narosecond / 1000;

        std::ostringstream ss;
        ss << std::setw(10) << std::setfill('0') << _image_index << ".png";
        std::string image_file_path = _images_directory[i] + "//" + ss.str();

        image_set[i]._image = cvLoadImage(image_file_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    }

#if STEREO_IMAGE_IO_KITTI_DEBUG
    std::cout << "StereoImageIOKITTI::getNextImageDataSet - " << _image_index << std::endl;
#endif

    return true;
}
