/*
 * Name:        CalibIO.h
 * Description: Interface to calibration files.
 * Author(s):   Andreas Geiger [geiger@mrt.uka.de]
 */

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <opencv/cv.h> // causes libdc error message!

#ifndef __CALIB_IO_H__
#define __CALIB_IO_H__

class CalibIO {

public:

  CalibIO();
  ~CalibIO();
  bool readCalibFromFile(std::string calib_file_name);
  void showCalibrationParameters();
  void showCvMat(CvMat *m,std::string desc="");
  bool calibrated() {
    return c_dist!=0 && roi!=0 && K1!=0 && D1!=0 && R1!=0 && K2!=0 && D2!=0 && R2!=0 &&
           R!=0 && T!=0 && P1!=0 && P1_roi!=0 && P2!=0 && P2_roi!=0;
  }
  uint32_t width();
  uint32_t height();
  void clear();
  
  std::vector<std::string> time;
  CvMat                    *c_dist,*roi;
  CvMat                    *K1,*D1,*R1,*K2,*D2,*R2;
  CvMat                    *R,*T,*P1,*P1_roi,*P2,*P2_roi;

private:

  std::vector<std::string> splitLine(std::string line);
  std::vector<std::string> readCalibFileString (FILE *calib_file,const char *string_name,bool &success);
  CvMat*                   readCalibFileMatrix (FILE *calib_file,const char *matrix_name,uint32_t m,uint32_t n,bool &success);

};

#endif
