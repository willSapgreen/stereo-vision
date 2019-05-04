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

#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"

class VisualOdometryStereo : public VisualOdometry
{
public:

    // stereo-specific parameters (mandatory: base)
    struct parameters : public VisualOdometry::parameters
    {
        double  base;             // baseline (meters)
        int32_t ransac_iters;     // number of RANSAC iterations
        double  inlier_threshold; // fundamental matrix inlier threshold
        bool    reweighting;      // lower border weights (more robust to calibration errors)
        parameters()
        {
            base             = 1.0;
            ransac_iters     = 200;
            inlier_threshold = 2.0;
            reweighting      = true;
        }
    };

    // constructor, takes as inpute a parameter structure
    VisualOdometryStereo(parameters param);

    // deconstructor
    virtual ~VisualOdometryStereo();

    // process a new images, push the images back to an internal ring buffer.
    // valid motion estimates are available after calling process for two times.
    // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
    //         I2 ........ pointer to rectified right image (uint8, row-aligned)
    //         dims[0] ... width of I1 and I2 (both must be of same size)
    //         dims[1] ... height of I1 and I2 (both must be of same size)
    //         dims[2] ... bytes per line (often equal to width)
    //         replace ... replace current images with I1 and I2, without copying last current
    //                     images to previous images internally. this option can be used
    //                     when small/no motions are observed to obtain Tr_delta wrt
    //                     an older coordinate system / time step than the previous one.
    // output: returns false if an error occured
    bool process(uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace=false);

    using VisualOdometry::process;



private:

    enum result { UPDATED, FAILED, CONVERGED };

    /**
     * @brief estimateMotion
     *        calculate the motion based on passed matched points, based on
     *        RANSAC and Iterated Gauss-Newton algorithms.
     *        RANSAC( Random sample consensus )
     *        Ref: http://www.cse.psu.edu/~rtc12/CSE486/lecture15.pdf
     *             http://www.cse.yorku.ca/~kosta/CompVis_Notes/ransac.pdf
     *
     *        Iterated Gauss-Newton
     *        Ref: https://see.stanford.edu/materials/lsoeldsee263/07-ls-reg.pdf
     *             SVO: Fast Semi-Direct Monocular Visual Odometry
     * @param p_matched: [in] matched points
     * @return
     *         Delta rotaitons and translations from time_K to time_K+1 wrt. time_K coordinate
     *         [rx, ry, rz, tx, ty, tz]
     */
    std::vector<double>  estimateMotion (std::vector<Matcher::p_match> p_matched);



    /**
     * @brief updateParameters
     *        Apply Iterated Gauss-Newton algorithm ( maximum likelihood estimate ) to estimate delta transformation.
     *
     * @param p_matched: [in] matched points
     * @param active: [in] the indices of the matched points used in Kalman Filter
     * @param tr: [in/out] the estimated motion which will be estimated based on active matched points
     * @param step_size
     * @param eps
     * @return the process result
     *
     * Ref: 1. Christian Forster, Matia Pizzoli, Davide Scaramuzza -
     *        "SVO: Fast Semi-Direct Monocular Visual Odometry"
     *      2. lucas-kanade.20.years.on.a.unifying.framework.part.1
     */
    result updateParameters(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);

    /**
     * @brief computeObservations:
     *        Construct the measurement(observations) matrix
     *
     * @param p_matched: [in] matched feature points
     * @param active: [in] random selected indices in matched feature points
     */
    void computeObservations(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active);

    /**
     * @brief computeResidualsAndJacobian:
     *        Compute residual and its derivative matrices.
     *        residual matrix: the distance difference between the observation and prediction 2D points
     *        derivative matrix: the Jacobian of residual matrix
     *
     *        State model ( nonlinear ):
     *        Pc = Tr * Pp
     *        where Pc is the current 3D position in camera homogenerous coordinate. (4x1)
     *              Tr is the transition matrix |R|T|
     *                                          |0|1|. (4x4)
     *              Pp is the previous 3D position in camera homogenerous coordinate. (4x1)
     *
     *        Measurement model ( nonlinear ):
     *        pc = Proj( Pc )
     *        where pc is the current 2D position in image coordinate. (2x1)
     *              pc is (u,v)
     *              Proj is the measurement function
     *              Pc is the 3D position in camera coordinate. (3x1)
     *              Pc is (X,Y,Z)
     *              u = fx * X/Z + cu
     *              v = fy * Y/Z + cv
     * @param tr: [in] transformation(motion) matrix
     * @param active: [in] random selected indices in mathced feature points
     */
    void computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);


    std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched,std::vector<double> &tr);

    // 3d points in previous left frame
    double* _X;
    double* _Y;
    double* _Z;

    // innovation ( pre-fit residuals ): p_residual = p_observe - p_predict
    double* _p_residual;

    // parameters
    parameters _param;
};

#endif // VISO_STEREO_H

