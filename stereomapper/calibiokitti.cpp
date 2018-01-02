#include "calibiokitti.h"
#include <sstream>

#define CALIB_IO_KITTI_DEBUG 0

using namespace std;

CalibIOKITTI::CalibIOKITTI( QObject* a_parent )
    : QObject( a_parent )
    , _picked(false)
    , _calibrated(false)
{
}

//==============================================================================//

CalibIOKITTI::~CalibIOKITTI ()
{
    clear();
}

//==============================================================================//

bool CalibIOKITTI::readCalibFromFiles(const std::string& a_cam_cam_calib_file_name,
                                      const std::string& a_imu_velo_calib_file_name,
                                      const std::string& a_velo_cam_calib_file_name)
{
    bool success = readCamToCamCalibFromFile( a_cam_cam_calib_file_name ) &&
                   readImuToVeloCalibFromFile( a_imu_velo_calib_file_name ) &&
                   readVeloToCamCalibFromFile( a_velo_cam_calib_file_name );
    if (success)
    {
        _calibrated = true;

        //signal to main dialog that we have the new calibration data.
        _picked = false;
        emit newCalibrationData();
        while (!_picked) usleep(1000);
    }

    return success;
}

//==============================================================================//

// displays all calibration matrices
void CalibIOKITTI::showCalibrationParameters() const
{
    std::cout << std::endl << "========================" << std::endl;
    std::cout << "Cam-To-Cam calibration parameters:";
    std::cout << std::endl << "========================" << std::endl << std::endl;
    std::cout << "Calibration time: ";
    for (uint32_t i=0; i<_cam_to_cam_calib_time.size(); i++)
    {
        std::cout << _cam_to_cam_calib_time[i] << " ";
    }
    std::cout << std::endl << std::endl;
    showCvMat(_cam_to_cam_corner_dist, "corner_dist");
    for (int i = 0; i < KITTI_CAMERA_NUM; ++i)
    {
        showCvMat(_cam_to_cam_S[i],"S0"+std::to_string(i));
        showCvMat(_cam_to_cam_K[i],"K0"+std::to_string(i));
        showCvMat(_cam_to_cam_D[i],"D0"+std::to_string(i));
        showCvMat(_cam_to_cam_R[i],"R0"+std::to_string(i));
        showCvMat(_cam_to_cam_T[i],"T0"+std::to_string(i));
        showCvMat(_cam_to_cam_S_rect[i],"S_rect0"+std::to_string(i));
        showCvMat(_cam_to_cam_R_rect[i],"R_rect0"+std::to_string(i));
        showCvMat(_cam_to_cam_P_rect[i],"P_rect0"+std::to_string(i));
    }

    std::cout << std::endl << "========================" << std::endl;
    std::cout << "Velo-To-Cam calibration parameters:";
    std::cout << std::endl << "========================" << std::endl << std::endl;
    std::cout << "Calibration time: ";
    for (uint32_t i=0; i<_velo_to_cam_calib_time.size(); i++)
    {
        std::cout << _velo_to_cam_calib_time[i] << " ";
    }
    std::cout << std::endl << std::endl;
    showCvMat(_velo_to_cam_R,"R");
    showCvMat(_velo_to_cam_T,"T");
    showCvMat(_velo_to_cam_delta_f,"delta_f");
    showCvMat(_velo_to_cam_delta_c,"delta_c");

    std::cout << std::endl << "========================" << std::endl;
    std::cout << "IMU-To-Cam calibration parameters:";
    std::cout << std::endl << "========================" << std::endl << std::endl;
    std::cout << "Calibration time: ";
    for (uint32_t i=0; i<_imu_to_velo_calib_time.size(); i++)
    {
        std::cout << _imu_to_velo_calib_time[i] << " ";
    }
    std::cout << std::endl << std::endl;
    showCvMat(_imu_to_velo_R,"R");
    showCvMat(_imu_to_velo_T,"T");
}

//==============================================================================//

void CalibIOKITTI::clear()
{
    _cam_to_cam_calib_time.clear();
     _cam_to_cam_corner_dist.release();
    for (int i = 0; i < KITTI_CAMERA_NUM; ++i)
    {
        _cam_to_cam_S[i].release();
        _cam_to_cam_K[i].release();
        _cam_to_cam_D[i].release();
        _cam_to_cam_R[i].release();
        _cam_to_cam_T[i].release();
        _cam_to_cam_S_rect[i].release();
        _cam_to_cam_R_rect[i].release();
        _cam_to_cam_P_rect[i].release();
    }

    _velo_to_cam_calib_time.clear();
    _velo_to_cam_R.release();
    _velo_to_cam_T.release();
    _velo_to_cam_delta_f.release();
    _velo_to_cam_delta_c.release();

    _imu_to_velo_calib_time.clear();
    _imu_to_velo_R.release();
    _imu_to_velo_T.release();

    _picked = false;
    _calibrated = false;
}

//==============================================================================//

vector<string> CalibIOKITTI::splitLine(const std::string& a_line)
{
    vector<string> line_vector;
    if (a_line[0]=='\0' || a_line[1]=='\0')
    {
        return line_vector;
    }

    uint32_t k1=0,k2=0;
    do
    {
        k2++;
        if (k2>k1 && (a_line[k2]==' ' || a_line[k2]=='\n' || a_line[k2]=='\0' || a_line[k2]=='\t' || a_line[k2]==',' || a_line[k2]==';') )
        {
            char line_vector_curr[10000];
            for (uint32_t i=k1; i<k2; i++)
            {
                line_vector_curr[i-k1] = a_line[i];
            }
            line_vector_curr[k2-k1] = '\0';
            //cout << line_vector_curr << "<-" << endl;
            line_vector.push_back(line_vector_curr);
            k1=k2+1;
        }
    } while (a_line[k2]!= '\0' && a_line[k2]!='\n' && k2<a_line.size());
    return line_vector;
}

//==============================================================================//

void CalibIOKITTI::showCvMat(const cv::Mat& a_m, const std::string& a_desc) const
{
    if (a_desc.compare(""))
    {
        std::cout << "Matrix \"" << a_desc << "\":" << std::endl;
    }

    if (!a_m.empty())
    {
        cv::Size s = a_m.size();
        for (int32_t i=0; i<s.height; ++i)
        {
            for (int32_t j=0; j<s.width; ++j)
            {
                std::cout << std::setw(10) << a_m.at<float>(i,j) << " ";
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << " --- undefined --- " << std::endl;
    }
    std::cout << std::endl;
}

//==============================================================================//

bool CalibIOKITTI::readCalibFileString(FILE* a_calib_file,
                                       const char* a_string_name,
                                       std::vector<std::string>& a_calib_string)
{
    // reset result
    a_calib_string.clear();

    // go to beginning of file
    rewind(a_calib_file);

    // buffer for lines of file
    char line[20000];

    // for all lines of calib file do
    while (a_calib_file!=NULL && fgets(line,sizeof(line),a_calib_file)!=NULL)
    {
        // split current line into elements
        vector<string> line_vector =  splitLine(line);

        // if matrix_name is first passage of this line
        if (!line_vector[0].compare(a_string_name))
        {
            for (uint32_t i=1; i<line_vector.size(); i++)
            {
                a_calib_string.push_back(line_vector[i]);
            }
            return true;
        }
    }

    return false;
}

//==============================================================================//

bool CalibIOKITTI::readCalibFileMatrix(FILE* a_calib_file, const char* a_matrix_name,
                                       uint32_t a_m, uint32_t a_n, cv::Mat& a_Mat)
{
    // go to beginning of file
    rewind(a_calib_file);

    // buffer for lines of file
    char line[20000];

    // for all lines of calib file do
    while(a_calib_file!=NULL && fgets(line,sizeof(line),a_calib_file)!=NULL)
    {
        // split current line into elements
        vector<string> line_vector =  splitLine(line);

        // if a_matrix_name is first passage of this line
        if (!line_vector[0].compare(a_matrix_name))
        {

            // check for right numer of matrix elements
            if (line_vector.size()-1 != a_m * a_n)
            {
                std::cout << "ERROR Number of elements in " << a_matrix_name << ": " << line_vector.size()-1 << "!=" << a_m * a_n << std::endl;
                return false;
            }

            // create and fill opencv matrix
            a_Mat.create(a_m, a_n, CV_32FC1);
            float val;

            uint32_t k=1;
            for (uint32_t i = 0; i < a_m; ++i)
            {
                for (uint32_t j = 0; j < a_n; ++j)
                {
                    stringstream sst;
                    sst<<line_vector[k++];
                    sst>>val;
                    a_Mat.at<float>( i,j ) = val;
                }
            }
        }
    }

    return true;
}

//==============================================================================//

// read calibration text file
bool CalibIOKITTI::readCamToCamCalibFromFile(const std::string& a_calib_file_name)
{
    // open calibration file
    FILE *calib_file = fopen (a_calib_file_name.c_str(),"r");
    if (calib_file==NULL)
    {
        return false;
    }

    readCalibFileString(calib_file, "calib_time:", _cam_to_cam_calib_time);
    readCalibFileMatrix(calib_file, "corner_dist:", 1, 1, _cam_to_cam_corner_dist);
    for (int i = 0; i < KITTI_CAMERA_NUM; ++i)
    {
        std::string index = std::to_string(i);

        std::string name = "S_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 1, 2, _cam_to_cam_S[i]);
        name = "K_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 3, 3, _cam_to_cam_K[i]);
        name = "D_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 1, 5, _cam_to_cam_D[i]);
        name = "R_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 3, 3, _cam_to_cam_R[i]);
        name = "T_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 1, 3, _cam_to_cam_T[i]);
        name = "S_rect_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 1, 2, _cam_to_cam_S_rect[i]);
        name = "R_rect_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 3, 3, _cam_to_cam_R_rect[i]);
        name = "P_rect_0" + index + ":";
        readCalibFileMatrix(calib_file, name.c_str(), 3, 4, _cam_to_cam_P_rect[i]);
    }

    // close file and return success
    fclose (calib_file);

    return true;
}

//==============================================================================//

bool CalibIOKITTI::readVeloToCamCalibFromFile(const std::string& a_calib_file_name)
{
    // open calibration file
    FILE *calib_file = fopen (a_calib_file_name.c_str(),"r");
    if (calib_file==NULL)
    {
        return false;
    }

    readCalibFileString(calib_file, "calib_time:", _velo_to_cam_calib_time);
    readCalibFileMatrix(calib_file, "R:", 3, 3, _velo_to_cam_R);
    readCalibFileMatrix(calib_file, "T:", 1, 3, _velo_to_cam_T);
    readCalibFileMatrix(calib_file, "delta_t:", 1, 2, _velo_to_cam_delta_f);
    readCalibFileMatrix(calib_file, "delta_c:", 1, 2, _velo_to_cam_delta_c);

    // close file and return success
    fclose (calib_file);

    return true;
}

//==============================================================================//

bool CalibIOKITTI::readImuToVeloCalibFromFile(const std::string& a_calib_file_name)
{
    // open calibration file
    FILE *calib_file = fopen (a_calib_file_name.c_str(),"r");
    if (calib_file==NULL)
    {
        return false;
    }

    readCalibFileString(calib_file, "calib_time:", _imu_to_velo_calib_time);
    readCalibFileMatrix(calib_file, "R:", 3, 3, _imu_to_velo_R);
    readCalibFileMatrix(calib_file, "T:", 1, 3, _imu_to_velo_T);

    // close file and return success
    fclose (calib_file);

    return true;
}
