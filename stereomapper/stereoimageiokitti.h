/*
 * Name:        stereoimageImagesIOKITTI.h
 * Description: Interface to read gray stereo image and timestamp for KITTI data set
 * TODO: read color stereo image.
 * Will Huang [willsapgreen@gmail.com]
 */

#ifndef STEREOIMAGEIOKITTI_H
#define STEREOIMAGEIOKITTI_H

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
    IMAGE_INPUT_SOURCE_GRAY_LEFT = 0,
    IMAGE_INPUT_SOURCE_GRAY_RIGHT,

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
     * Set up left and right image directories and timestamps.
     */
    bool setUpDataPath(const std::string images_directory[IMAGE_INPUT_SOURCE_COUNT],
                       const std::string images_timestamp[IMAGE_INPUT_SOURCE_COUNT]);

    /*
     * Get the size of the storage.
     */
    inline int32_t getImagesNumber() const
    {
        return _images_number;
    }

    /*
     * Get the current image index.
     * TODO:
     * Using mutex!!!
     */
    inline int32_t getImageIndex() const
    {
        return _image_index;
    }

    /*
     * Get the next ImageDataCV set.
     */
    bool getNextImageDataSet(ImageDataCV image_set[IMAGE_INPUT_SOURCE_COUNT]);

private:

    int32_t _images_number;
    int32_t _image_index;
    //ImageDataCV _image_set[IMAGE_INPUT_SOURCE_COUNT];
    std::ifstream _time_stamp_stream[IMAGE_INPUT_SOURCE_COUNT];
    std::string _images_directory[IMAGE_INPUT_SOURCE_COUNT];
    std::string _timestamp_file[IMAGE_INPUT_SOURCE_COUNT];
};

#endif
