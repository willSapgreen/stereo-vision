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

    /**
     * @brief process:
     *        Process p_matched points to estimate transformation
     * @param p_matched: matched points
     * @return
     *        True if transformaiton calculation works.
     */
    bool process(std::vector<Matcher::p_match> p_matched)
    {
        _p_matched = p_matched;
        return updateMotion();
    }

    /**
     * @brief getDeltaMotion:
     * Get the latest delta transformation.
     * Note: getDeltaMotion() returns the last transformation even when process()
     * has failed.
     * @return
     * the latest delta transformation.
     */
    Matrix getDeltaMotion() const { return _Tr_delta; }

    /**
     * @brief calculateRollPitchYawFromTransformation:
     *        Calculate roll, pitch, and yaw from rotation part in transformation
     * @param roll [in/out]: delta roll angle ( radian )
     * @param pitch [in/out]: delta pitch angle ( radian )
     * @param yaw [in/out]: delta yaw angle ( radian )
     */
    void calculateRollPitchYawFromTransformation( double& roll, double& pitch, double& yaw ) const;

    /**
     * @brief calculateVelocityFromTransformation:
     *        Calculate delta velocity from translation part in transformation
     * @param velocity [in/out]: velocity ( meter )
     */
    void calculateVelocityFromTransformation( double& velocity ) const;

    /**
     * @brief calculateAltitudeFromTransformation:
     *        Calculate delta altitude from translation part in transformation
     * @param altitude [in/out]: delta altitude ( meter )
     */
    void calculateAltitudeFromTransformation( double& altitude ) const;

    /**
     * @brief  getMatches:
     *         Get matched points ( previous to current feature matches from internal matcher )
     * @return
     *         matched points
     */
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
   *        Calculate delta rotational and translational movement
   *        [rx, ry, rz, tx, ty, tz]
   *        And then construct transformation matrix(4x4).
   *
   * @return
   *         True if valid.
   */
  bool updateMotion();

  /**
   * @brief transformationVectorToMatrix:
   *        Calculate the transformation matrix from translational/rotational velocities.
   *
   * @param tr: 6x1 [rx, ry, rz, tx, ty, tz]
   * @return
   *         4x4 transformation matrix
   *
   * Ref: http://www.songho.ca/opengl/gl_anglestoaxes.html
   */
  Matrix transformationVectorToMatrix(std::vector<double> tr);

  bool inverseTransformationVector( const std::vector<double>& tr, std::vector<double>& tr_inv );

  /**
   * virtual Function - For implementation detail, please refer to viso_stereo.h
   * @brief estimateMotion
   * @param p_matched
   * @return
   */
  virtual std::vector<double> estimateMotion(std::vector<Matcher::p_match> p_matched) = 0;
  
  // get random and unique sample of num numbers from 1:N
  std::vector<int32_t> getRandomSample(int32_t N,int32_t num);

  /* Delta transformation.
   * 4x4 matrix
   * |R T|
   * |0 1|
   * R: 3x3
   * T: 3x1
   *
   * This transformation( _Tr_delta ) is vehicle transformation from time K to time K+1.
   * The transformation is wrt. the time K coordinate. ( passive transformation )
   */
  Matrix                         _Tr_delta;


  bool                           _Tr_valid;   // flag to determine if motion estimate exists
  Matcher*                       _matcher;    // feature matcher
  std::vector<int32_t>           _inliers;    // inlier set

  /**
   * Jacobian of residual function.
   * Residual function is
   * ( predicted_u - observed_u ) and ( predicted_v - observed_v )
   * Similar to " SVO: Fast Semi-Direct Monocular Visual Odometry "
   * In the SVO paper, the residual function is based on the intensity difference.
   * Here is based on the distance difference.
   */
  double*                        _J;

  // Observed 2D points
  // 4 * N points where N is determined by number of measurments each time
  // 4: u1 v1 u2 v2
  double*                        _p_observe;

  // Predicted 2D points
  // 4 * N points where N is determined by number of measurments each time
  // 4: u1 v1 u2 v2
  double*                        _p_predict;

  // feature point matches
  std::vector<Matcher::p_match>  _p_matched;
  
private:
  
  // common parameters
  parameters                    _param;

};

#endif // VISO_H

