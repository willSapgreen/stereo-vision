#include "readfromfilesthread.h"
#include "QFileDialog"

using namespace std;

ReadFromFilesThread::ReadFromFilesThread( StereoImage *stereo_image, CalibIOKITTI *_calib,
                                          StereoImageIOKITTI* stereo_image_io, OxTSIOKITTI* oxts_io, QObject *parent)
    : QThread(parent)
    , _calib(_calib)
    , _stereo_image_io( stereo_image_io )
    , _oxts_io(oxts_io)
    , _stereo_image(stereo_image)
{
}

//==============================================================================//

ReadFromFilesThread::~ReadFromFilesThread()
{
}

//==============================================================================//

void ReadFromFilesThread::run()
{
    // Convert QString to std::string
    std::string input_dir_str = _input_dir.toStdString();

    // Check if the variables are valid.
    if ((NULL == _calib) ||
        (NULL == _stereo_image_io) ||
        (input_dir_str.empty()))
    {
        return;
    }

    // Read in the calibration file for KITTI dataset.
    if (_calib->readCalibFromFiles((input_dir_str + DEFAULT_CAM_TO_CAM_TXT_PATH),
                                   (input_dir_str + DEFAULT_IMU_TO_VELO_TXT_PATH),
                                   (input_dir_str + DEFAULT_VELO_TO_CAM_TXT_PATH)))
    {
        // Show calibration parameters.
        _calib->showCalibrationParameters();

        // Process left/right images and timestamp.
        bool process_succeed = _stereo_image_io->fetchGrayStereoImage(input_dir_str + DEFAULT_IMAGE00_DATA_PATH,
                                                                      input_dir_str + DEFAULT_IMAGE00_TIMESTAMP_TXT_PATH,
                                                                      input_dir_str + DEFAULT_IMAGE01_DATA_PATH,
                                                                      input_dir_str + DEFAULT_IMAGE01_TIMESTAMP_TXT_PATH) &&
                               _oxts_io->fetchGrayOxTSData(input_dir_str + DEFAULT_OXTS_DATA_PATH,
                                                           input_dir_str + DEFAULT_OXTS_TIMESTAMP_TXT_PATH);

        if (!process_succeed)
        {
            return;
        }

        // start output loop
        float fps = 10;
        ImageDataCV left_img;
        ImageDataCV right_img;
        unsigned char* left_img_data;
        unsigned char* right_img_data;
        for (int32_t i=0; i<(int32_t)_stereo_image_io->getImagesNumber(); ++i)
        {
            _stereo_image_io->getLeftRightImageData(i, left_img, right_img);

            // Get the raw data.
            cvGetRawData(left_img._image, &left_img_data);
            cvGetRawData(right_img._image, &right_img_data);

            _stereo_image->setImage(left_img_data, left_img._image->width,
                                    left_img._image->height, left_img._image->widthStep,
                                    true, true, left_img._captured_time);
            _stereo_image->setImage(right_img_data, right_img._image->width,
                                    right_img._image->height, right_img._image->widthStep,
                                    false, true, right_img._captured_time);
            usleep(1e6/fps);
        }
    }
    else
    {
        cout << "No calibration file found => No files read." << endl;
    }
}
