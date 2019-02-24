#include "readfromfilesthread.h"
#include "QFileDialog"

#define READ_FROM_FILES_THREAD_DEBUG 0

ReadFromFilesThread::ReadFromFilesThread(StereoImage *stereo_image, GPSInertialData *gps_inertial_data, CalibIOKITTI *_calib,
                                          StereoImageIOKITTI* stereo_image_io, GPSInertialDataIOKITTI* oxts_io, QObject *parent)
    : QThread(parent)
    , _calib(_calib)
    , _stereo_image_io( stereo_image_io )
    , _gps_inertial_data_io(oxts_io)
    , _stereo_image(stereo_image)
    , _gps_inertial_data(gps_inertial_data)
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
        std::string image_timestamp_files[IMAGE_INPUT_SOURCE_COUNT];
        std::string image_directories[IMAGE_INPUT_SOURCE_COUNT];
        image_timestamp_files[IMAGE_INPUT_SOURCE_GRAY_LEFT] = input_dir_str + DEFAULT_IMAGE00_TIMESTAMP_TXT_PATH;
        image_timestamp_files[IMAGE_INPUT_SOURCE_GRAY_RIGHT] = input_dir_str + DEFAULT_IMAGE01_TIMESTAMP_TXT_PATH;
        image_directories[IMAGE_INPUT_SOURCE_GRAY_LEFT] = input_dir_str + DEFAULT_IMAGE00_DATA_PATH;
        image_directories[IMAGE_INPUT_SOURCE_GRAY_RIGHT] = input_dir_str + DEFAULT_IMAGE01_DATA_PATH;

        bool process_succeed = _stereo_image_io->setUpDataPath(image_directories, image_timestamp_files) &&
                               _gps_inertial_data_io->setUpDataPath(input_dir_str + DEFAULT_OXTS_DATA_PATH,
                                                           input_dir_str + DEFAULT_OXTS_TIMESTAMP_TXT_PATH);
        if ((!process_succeed) ||
            (_stereo_image_io->getImagesNumber() != _gps_inertial_data_io->getOxTSDataSize()))
        {
            std::cout << "ERROR: Cannot fetch data or image/oxts data sizes are different\n";
            return;
        }

        int32_t data_size = _gps_inertial_data_io->getOxTSDataSize();

        // start output loop
        float fps = 10;
        //bool getOxTSData(int nth, OxSTData& oxts_data);
        OxSTData oxts_data;
        ImageDataCV image_set[IMAGE_INPUT_SOURCE_COUNT];
        unsigned char* left_img_data;
        unsigned char* right_img_data;

        for (int32_t i=0; i < data_size; ++i)
        {
            if(!(_stereo_image_io->getNextImageDataSet(image_set) &&
                 _gps_inertial_data_io->getNextGPSInertialData(oxts_data)))
            {
                continue;
            }

            // Convert OxST data to GPSInertialData.
            GPSInertialDataFormat gi_data;
            oxts_data.toGPSInertialDataFormat(gi_data);
            _gps_inertial_data->setData(gi_data);

            // Get the raw data.
            cvGetRawData(image_set[IMAGE_INPUT_SOURCE_GRAY_LEFT]._image, &left_img_data);
            cvGetRawData(image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._image, &right_img_data);

            _stereo_image->setImage(left_img_data,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_LEFT]._image->width,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_LEFT]._image->height,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_LEFT]._image->widthStep,
                                    true, true, image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._captured_time);
            _stereo_image->setImage(right_img_data,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._image->width,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._image->height,
                                    image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._image->widthStep,
                                    false, true, image_set[IMAGE_INPUT_SOURCE_GRAY_RIGHT]._captured_time);

            // Release the memory in IplImage.
            for(int j = 0; j < IMAGE_INPUT_SOURCE_COUNT; ++j)
            {
                cvReleaseImage(&(image_set[j]._image));
            }

            usleep(1e6/fps);
        } // for (int32_t i=0; i < data_size; ++i)

        emit playbackDataFinished();
    }
    else
    {
        std::cout << "No calibration file found => No files read." << std::endl;
    }
}
