/*
 * Name:        CalibIOKITTI.h
 * Description: Interface to calibration files for KITTI data set
 * Author(s):   Andreas Geiger [geiger@mrt.uka.de], Will Huang [willsapgreen@gmail.com]
 */

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QObject>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#ifndef __CALIB_IO_KITTI_H__
#define __CALIB_IO_KITTI_H__

#define KITTI_CAMERA_NUM 4

class CalibIOKITTI : public QObject
{
    Q_OBJECT

public:

    /*
     * Default constructor.
     */
    CalibIOKITTI( QObject* a_parent=0 );

    /*
     * Default destructor.
     */
    virtual ~CalibIOKITTI();

    /*
     * Read three calibration files in one KITTI data set.
     */
    bool readCalibFromFiles(const std::string& a_cam_cam_calib_file_name,
                            const std::string& a_imu_velo_calib_file_name,
                            const std::string& a_velo_cam_calib_file_name);

    /*
     * Show the calibration parameters.
     */
    void showCalibrationParameters() const;

    /*
     * Clear all calibration parameters.
     */
    void clear();

    /*
     * Check if the calibration parameters are loaded.
     */
    bool calibrated() const { return m_calibrated; }

    /*
     * The signal receiver will use this function to confirm the signal is picked.
     */
    void pickedUp() { m_picked = true; }

    /*
     * In KITTI, there are four cameras.
     * In the following array:
     * 0 is left gray camera.
     * 1 is right gray camera.
     * 2 is left color camera.
     * 3 is right color camera.
     */
    std::vector<std::string> m_cam_to_cam_calib_time;
    cv::Mat m_cam_to_cam_corner_dist;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_S;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_K;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_D;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_R;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_T;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_S_rect;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_R_rect;
    std::array< cv::Mat, KITTI_CAMERA_NUM> m_cam_to_cam_P_rect;

    std::vector<std::string> m_velo_to_cam_calib_time;
    cv::Mat m_velo_to_cam_R;
    cv::Mat m_velo_to_cam_T;
    cv::Mat m_velo_to_cam_delta_f;
    cv::Mat m_velo_to_cam_delta_c;

    std::vector<std::string> m_imu_to_velo_calib_time;
    cv::Mat m_imu_to_velo_R;
    cv::Mat m_imu_to_velo_T;

    bool m_picked;
    bool m_calibrated;

private:

    /*
     * Split the input string into the vector.
     */
    std::vector<std::string> splitLine(const std::string& a_line);

    /*
     * Show cv::Mat content with the name.
     */
    void showCvMat(const cv::Mat& a_m, const std::string& a_desc="") const;

    /*
     * Read the string in the calibration file.
     */
    bool readCalibFileString(FILE* a_calib_file, const char* a_string_name, std::vector<std::string>& a_calib_string);

    /*
     * Read the matrix setting in the calibration file.
     */
    bool readCalibFileMatrix(FILE* a_calib_file, const char* a_matrix_name, uint32_t a_m, uint32_t a_n, cv::Mat& a_Mat);

    /*
     * Read calib_cam_to_cam.txt.
     */
    bool readCamToCamCalibFromFile(const std::string& a_calib_file_name);

    /*
     * Read calib_imu_to_velo.txt
     */
    bool readImuToVeloCalibFromFile(const std::string& a_calib_file_name);

    /*
     * Read calib_velo_to_cam.txt.
     */
    bool readVeloToCamCalibFromFile(const std::string& a_calib_file_name);

signals:
    void newCalibrationData();
};

#endif
