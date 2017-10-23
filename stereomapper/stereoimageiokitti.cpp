#include "stereoimageiokitti.h"

StereoImageIOKITTI::StereoImageIOKITTI()
    : _images_number(0)
    , _left_gray_images(nullptr)
    , _right_gray_images(nullptr)
{

}

//==============================================================================//

StereoImageIOKITTI::~StereoImageIOKITTI()
{
    _left_gray_images.release();
    _right_gray_images.release();
}

//==============================================================================//

bool StereoImageIOKITTI::fetchGrayStereoImage(const std::string& left_images_directory,
                                              const std::string& left_images_timestamp,
                                              const std::string& right_images_directory,
                                              const std::string& right_images_timestamp)
{
    // Read the timestamp files.
    std::ifstream time_stamp_stream[IMAGE_INPUT_SOURCE_COUNT];
    time_stamp_stream[GRAY_LEFT].open(left_images_timestamp);
    time_stamp_stream[GRAY_RIGHT].open(right_images_timestamp);
    if (!(time_stamp_stream[GRAY_LEFT].is_open()) ||
        !(time_stamp_stream[GRAY_RIGHT].is_open()))
    {
        std::cout << "ERROR: cannot open left/right timestamp text file" << std::endl;
        return false;
    }


    // Count the lines number in both timestamp files.
    int32_t left_line_count = std::count( std::istreambuf_iterator<char>( time_stamp_stream[GRAY_LEFT] ),
                                           std::istreambuf_iterator<char>(),
                                           '\n');
    int32_t right_line_count = std::count( std::istreambuf_iterator<char>( time_stamp_stream[GRAY_RIGHT] ),
                                           std::istreambuf_iterator<char>(),
                                           '\n');
    if ((left_line_count != right_line_count) ||
        (0 == left_line_count))
    {
        std::cout << "ERROR: left and right gray timestamp lines counts are not consistent" << std::endl;
        return false;
    }
    _images_number = left_line_count;
    time_stamp_stream[GRAY_LEFT].clear();
    time_stamp_stream[GRAY_RIGHT].clear();
    time_stamp_stream[GRAY_LEFT].seekg(0, std::ios::beg);
    time_stamp_stream[GRAY_RIGHT].seekg(0, std::ios::beg);

    // Initialize the image arrays.
    _left_gray_images = std::unique_ptr<ImageDataCV[]>(new ImageDataCV[_images_number]);
    _right_gray_images = std::unique_ptr<ImageDataCV[]>(new ImageDataCV[_images_number]);

    // Read in timestamp.
    std::string lines[IMAGE_INPUT_SOURCE_COUNT];
    struct tm time;
    int count = 0;
    if (time_stamp_stream[GRAY_LEFT].is_open() && time_stamp_stream[GRAY_RIGHT].is_open())
    {

        while ((std::getline(time_stamp_stream[GRAY_LEFT], lines[GRAY_LEFT])) &&
               (std::getline(time_stamp_stream[GRAY_RIGHT], lines[GRAY_RIGHT])))
        {
            // USE
            // mktime to get epoch time in second from year, month, day, hour, min, and second.
            // and assign epoch time to timeval's second.
            // assign part of narosecond to timeval's microsecond.
            // Ref: https://stackoverflow.com/questions/28880849/convert-struct-tm-to-time-t

            int year = std::stoi(lines[GRAY_LEFT].substr(0,4));
            int month = std::stoi(lines[GRAY_LEFT].substr(5,2));
            int day = std::stoi(lines[GRAY_LEFT].substr(8,2));
            int hour = std::stoi(lines[GRAY_LEFT].substr(11,2));
            int minute = std::stoi(lines[GRAY_LEFT].substr(14,2));
            int second = std::stoi(lines[GRAY_LEFT].substr(17,2));
            int narosecond = std::stoi(lines[GRAY_LEFT].substr(20,9));

            time.tm_year = year;
            time.tm_mon = month;
            time.tm_mday = day;
            time.tm_hour = hour;
            time.tm_min = minute;
            time.tm_sec = second;
            mktime(&time);

            _left_gray_images[count]._captured_time.tv_sec = time.tm_sec;
            _left_gray_images[count]._captured_time.tv_usec = narosecond / 1000;

            year = std::stoi(lines[GRAY_RIGHT].substr(0,4));
            month = std::stoi(lines[GRAY_RIGHT].substr(5,2));
            day = std::stoi(lines[GRAY_RIGHT].substr(8,2));
            hour = std::stoi(lines[GRAY_RIGHT].substr(11,2));
            minute = std::stoi(lines[GRAY_RIGHT].substr(14,2));
            second = std::stoi(lines[GRAY_RIGHT].substr(17,2));
            narosecond = std::stoi(lines[GRAY_RIGHT].substr(20,9));

            time.tm_year = year;
            time.tm_mon = month;
            time.tm_mday = day;
            time.tm_hour = hour;
            time.tm_min = minute;
            time.tm_sec = second;
            mktime(&time);

            _right_gray_images[count]._captured_time.tv_sec = time.tm_sec;
            _right_gray_images[count]._captured_time.tv_usec = narosecond / 1000;

            std::ostringstream ss;
            ss << std::setw(10) << std::setfill('0') << count << ".png";
            std::string left_image_file_path = left_images_directory + "//" + ss.str();
            std::string right_image_file_path = right_images_directory + "//" + ss.str();

            std::cout << "Reading in " << ss.str() << std::endl;

            _left_gray_images[count]._image = cvLoadImage(left_image_file_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
            _right_gray_images[count]._image = cvLoadImage(right_image_file_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
            ++count;
        }
        time_stamp_stream[GRAY_LEFT].close();
        time_stamp_stream[GRAY_RIGHT].close();
    }
    else
    {
        std::cout << "ERROR: cannot open left/right timestamp text file" << std::endl;
        return false;
    }

    return true;
}

//==============================================================================//

bool StereoImageIOKITTI::getLeftRightImageData(int nth, ImageDataCV& left_image, ImageDataCV& right_image)
{
    // Check if a_Nth is valid.
    if( nth < 0 || nth >= _images_number )
    {
        return false;
    }

    // Mat shallow copy
    left_image = _left_gray_images[nth];
    right_image = _right_gray_images[nth];

    return true;
}
