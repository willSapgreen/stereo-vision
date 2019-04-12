/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef VISO_H
#define VISO_H

#include "matrix.h"
#include "matcher.h"

class VisualOdometry
{
public:

    // camera parameters (all are mandatory / need to be supplied)
    struct calibration
    {
        double f;  // focal length (in pixels)
        double cu; // principal point (u-coordinate)
        double cv; // principal point (v-coordinate)
        calibration()
        {
            f  = 1;
            cu = 0;
            cv = 0;
        }
    };

    // bucketing parameters
    struct bucketing
    {
        int32_t max_features;  // maximal number of features per bucket
        double  bucket_width;  // width of bucket
        double  bucket_height; // height of bucket
        bucketing()
        {
            max_features  = 2;
            bucket_width  = 50;
            bucket_height = 50;
        }
    };

    // general parameters
    struct parameters
    {
        Matcher::parameters         match;            // matching parameters
        VisualOdometry::bucketing   bucket;           // bucketing parameters
        VisualOdometry::calibration calib;            // camera calibration parameters
    };

    // constructor, takes as input a parameter structure:
    // do not instanciate this class directly, instanciate VisualOdometryMono
    // or VisualOdometryStereo instead!
    VisualOdometry(parameters param);

    // deconstructor
    ~VisualOdometry();

    // call this function instead of the specialized ones, if you already have
    // feature matches, and simply want to compute visual odometry from them, without
    // using the internal matching functions.
    bool process (std::vector<Matcher::p_match> p_matched)
    {
        _p_matched = p_matched;
        return updateMotion();
    }

    // returns transformation from previous to current coordinates as a 4x4
    // homogeneous transformation matrix Tr_delta, with the following semantics:
    // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
    // at time t_1 and maps it to the camera coordinate system at time t.
    // note: getDeltaMotion() returns the last transformation even when process()
    // has failed. this is useful if you wish to linearly extrapolate occasional
    // frames for which no correspondences have been found
    Matrix getDeltaMotion() const { return _Tr_delta; }

    void calculateRollPitchYawFromTransformation( double& roll, double& pitch, double& yaw ) const;

    void calculateVelocityFromTransformation( double& velocity ) const;

    void calculateAltitudeFromTransformation( double& altitude ) const;

    // returns previous to current feature matches from internal matcher
    std::vector<Matcher::p_match> getMatches() { return _matcher->getMatches(); }

    // returns the number of successfully matched points, after bucketing
    int32_t getNumberOfMatches() { return _p_matched.size(); }

    // returns the number of inliers: num_inliers <= num_matched
    int32_t getNumberOfInliers() { return _inliers.size(); }

    // returns the indices of all inliers
    std::vector<int32_t> getInlierIndices() { return _inliers; }

    // given a vector of inliers computes gain factor between the current and
    // the previous frame. this function is useful if you want to reconstruct 3d
    // and you want to cancel the change of (unknown) camera gain.
    float getGain(std::vector<int32_t> inliers) { return _matcher->getGain(inliers); }

    // streams out the current transformation matrix Tr_delta
    friend std::ostream& operator<<(std::ostream &os,VisualOdometry &viso)
    {
        Matrix p = viso.getDeltaMotion();
        os << p._val[0][0] << " " << p._val[0][1] << " "  << p._val[0][2]  << " "  << p._val[0][3] << " ";
        os << p._val[1][0] << " " << p._val[1][1] << " "  << p._val[1][2]  << " "  << p._val[1][3] << " ";
        os << p._val[2][0] << " " << p._val[2][1] << " "  << p._val[2][2]  << " "  << p._val[2][3];
        return os;
    }

protected:

  /**
   * @brief updateMotion
   * Kalman Filter Predict translational and rotational velocities,
   * [Vx, Vy, Vz, Wx, Wy, Wz]
   * and the transformation matrix(4x4).
   *
   * @return
   * True if valid.
   */
  bool updateMotion();

  /**
   * @brief transformationVectorToMatrix
   * Calculate the transformation matrix from translational/rotational velocities.
   *
   * @param tr
   * [Vx, Vy, Vz, Wx, Wy, Wz]: 6x1
   *
   * @return
   * [R | T]: 4x4 transformation matrix from previous frame to current frame.
   */
  Matrix transformationVectorToMatrix(std::vector<double> tr);

  // compute motion from previous to current coordinate system
  // if motion could not be computed, resulting vector will be of size 0
  virtual std::vector<double> estimateMotion(std::vector<Matcher::p_match> p_matched) = 0;
  
  // get random and unique sample of num numbers from 1:N
  std::vector<int32_t> getRandomSample(int32_t N,int32_t num);

  /*
   * 4x4 matrix
   * R T
   * 0 1
   * R: 3x3
   * T: 3x1
   */
  Matrix                         _Tr_delta;   // transformation (previous -> current frame)


  bool                           _Tr_valid;   // motion estimate exists?
  Matcher*                       _matcher;    // feature matcher
  std::vector<int32_t>           _inliers;    // inlier set
  double*                        _J;          // jacobian
  double*                        _p_observe;  // observed 2d points
  double*                        _p_predict;  // predicted 2d points
  std::vector<Matcher::p_match>  _p_matched;  // feature point matches
  
private:
  
  parameters                    _param;     // common parameters

};

#endif // VISO_H

