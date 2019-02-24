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
#include "../libviso2/src/matrix.h"

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
    bool calibrated() const { return _calibrated; }

    /*
     * The signal receiver will use this function to confirm the signal is picked.
     */
    void pickedUp() { _picked = true; }


    /*
     * Show content in Matrix with the name.
     */
    static void showMatrix(const Matrix& matrix, const std::string& desc="");

    /*
     * In KITTI, there are four cameras.
     * In the following array:
     * 0 is left gray camera.
     * 1 is right gray camera.
     * 2 is left color camera.
     * 3 is right color camera.
     *
     * Example
     * # calibration time.
     * calib_time: 09-Jan-2012 13:57:47
     *
     * # TODO: I DO NOT KNOW.
     * corner_dist: 9.950000e-02
     *
     * # original image size.
     * S_00: 1.392000e+03 5.120000e+02
     *
     * # Instrinsic parameters calibration matrix for camera 0.
     * # Reference: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
     * # Purpose: transform the point in camera coordinate to image coordinate.
     * K_00: 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
     *
     * # Distortion cofficients for camera 0.
     * # Reference: http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
     * # Purpose: correct the distortion in image coordinate.
     * D_00: -3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 -7.233722e-02
     *
     * # Rotation matrix from camera 0 to camera 0.
     * # Reference: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
     * # Purpose: present the rotation relationship between two cameras( observed at camera 0 )
     * R_00: 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
     *
     * # Translation matrix from camera 0 to camera 0.
     * # Reference: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
     * # Purpose: present the translatioopencv integrate two different sizen relationship between two cameras( observed at camera 0 )
     * T_00: 2.573699e-16 -1.059758e-16 1.614870e-16
     *
     * # Image size after rectification.
     * S_rect_00: 1.242000e+03 3.750000e+02
     *
     * # Rectifying rotation matrix.
     * # Reference: https://en.wikipedia.org/wiki/Image_rectification
     * # Purpose: transform the origin camera coordinate to rectified camera coordinate.
     *            The left/right rectified camera coordinates are aligned in parallel.
     * R_rect_00: 9.999239e-01 9.837760e-03 -7.445048e-03 -9.869795e-03 9.999421e-01 -4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01
     *
     * # Projection matrix after rectification.
     * # Reference: http://wiki.ros.org/image_pipeline/CameraInfo
     * # Reference: https://stackoverflow.com/questions/29910548/how-do-i-get-the-projection-matrix-of-a-camera-after-stereo-rectification
     * # Purpose: transform the point in world coordinate to rectified image coordinate.
     * P_rect_00: 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
     */

    std::vector<std::string> _cam_to_cam_calib_time;
    Matrix _cam_to_cam_corner_dist;
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_S; // original image size. 1x2
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_K; // calibration matrices( unrectified ). 3x3
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_D; // distrotion matrices ( unrectified ). 1x5
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_R; // rotation matrices from camera 0 to camera i. 3x3
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_T; // translation matrices from camera 0 to camera i. 1x3
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_S_rect; // image size after rectification. 1x2
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_R_rect; // rectifying rotation matrix. 3x3
    std::array< Matrix, KITTI_CAMERA_NUM> _cam_to_cam_P_rect; // projection matrix after rectification. 3x4

    std::vector<std::string> _velo_to_cam_calib_time;
    Matrix _velo_to_cam_R; // 3x3
    Matrix _velo_to_cam_T; // 1x3
    Matrix _velo_to_cam_delta_f;
    Matrix _velo_to_cam_delta_c;

    std::vector<std::string> _imu_to_velo_calib_time;
    Matrix _imu_to_velo_R; // 3x3
    Matrix _imu_to_velo_T; // 1x3

    bool _picked;
    bool _calibrated;

private:

    /*
     * Split the input string into the vector.
     */
    std::vector<std::string> splitLine(const std::string& a_line);

    /*
     * Read the string in the calibration file.
     */
    bool readCalibFileString(FILE* a_calib_file, const char* a_string_name, std::vector<std::string>& a_calib_string);

    /*
     * Read the matrix setting in the calibration file.
     */
    bool readCalibFileMatrix(FILE* a_calib_file, const char* a_matrix_name,
                             uint32_t a_m, uint32_t a_n, Matrix& a_Matrix);

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
