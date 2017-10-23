/*
 * Name:        stereoimageImagesIOKITTI.h
 * Description: Interface to read gray stereo image and timestamp for KITTI data set
 * TODO: read color stereo image.
 * Will Huang [willsapgreen@gmail.com]
 */

// std
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <memory>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// dirent
#include "dirent.h"

enum ImageInputSource
{
    GRAY_LEFT = 0,
    GRAY_RIGHT,

    IMAGE_INPUT_SOURCE_COUNT
};

struct ImageDataCV
{
    IplImage* _image;
    timeval _captured_time;
    timeval _applied_time;
};

class StereoImageIOKITTI
{
public:
    /*
     * Default constructor.
     */
    StereoImageIOKITTI();

    /*
     * Default destructor.
     */
    virtual ~StereoImageIOKITTI();

    /*
     * Process the left and right gray images and timestamp.
     */
    bool fetchGrayStereoImage(const std::string& left_images_directory,
                              const std::string& left_images_timestamp,
                              const std::string& right_images_directory,
                              const std::string& right_images_timestamp);

    /*
     * Get the size of the storage.
     */
    inline int32_t getImagesNumber() const
    {
        return _images_number;
    }

    /*
     * Get the nth left and right gray ImageDataCV.
     * 0 <= nth < the size of the storage.
     */
    bool getLeftRightImageData(int nth, ImageDataCV& left_image, ImageDataCV& right_image);

private:

    int32_t _images_number;
    std::unique_ptr<ImageDataCV[]> _left_gray_images;
    std::unique_ptr<ImageDataCV[]> _right_gray_images;
};
