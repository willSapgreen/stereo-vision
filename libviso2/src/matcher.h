/*
Copyright 2012. All rights reserved.
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

#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

#include "matrix.h"

class Matcher
{
public:

    // parameter settings
    struct parameters
    {
        int32_t nms_n;                  // non-max-suppression: min. distance between maxima (in pixels)
        int32_t nms_tau;                // non-max-suppression: interest point peakiness threshold
        int32_t match_binsize;          // matching bin width/height (affects efficiency only)
        int32_t match_radius;           // matching radius (du/dv in pixels)
        int32_t match_disp_tolerance;   // dv tolerance for stereo matches (in pixels)
        int32_t outlier_disp_tolerance; // outlier removal: disparity tolerance (in pixels)
        int32_t outlier_flow_tolerance; // outlier removal: flow tolerance (in pixels)
        int32_t multi_stage;            // 0=disabled,1=multistage matching (denser and faster)
        int32_t half_resolution;        // 0=disabled,1=match at half resolution, refine at full resolution
        int32_t refinement;             // refinement (0=none,1=pixel,2=subpixel)
        double  f,cu,cv,base;           // calibration (only for match prediction)

        // default settings
        parameters()
        {
            nms_n                  = 3;
            nms_tau                = 50;
            match_binsize          = 50;
            match_radius           = 200;
            match_disp_tolerance   = 2;
            outlier_disp_tolerance = 5;
            outlier_flow_tolerance = 5;
            multi_stage            = 1;
            half_resolution        = 1;
            refinement             = 1;
        }
    };

    // constructor (with default parameters)
    Matcher(parameters param);

    // deconstructor
    ~Matcher();

    // intrinsics
    void setIntrinsics(double f,double cu,double cv,double base)
    {
        _param.f = f;
        _param.cu = cu;
        _param.cv = cv;
        _param.base = base;
    }

    // structure for storing matches
    struct p_match
    {
        float   u1p,v1p; // u,v-coordinates in previous left  image
        int32_t i1p;     // feature index (for tracking)
        float   u2p,v2p; // u,v-coordinates in previous right image
        int32_t i2p;     // feature index (for tracking)
        float   u1c,v1c; // u,v-coordinates in current  left  image
        int32_t i1c;     // feature index (for tracking)
        float   u2c,v2c; // u,v-coordinates in current  right image
        int32_t i2c;     // feature index (for tracking)
        p_match(){}
        p_match(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
                float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
                u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
                u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) {}
    };

    // computes features from left/right images and pushes them back to a ringbuffer,
    // which interally stores the features of the current and previous image pair
    // use this function for stereo or quad matching
    // input: I1,I2 .......... pointers to left and right image (row-aligned), range [0..255]
    //        dims[0,1] ...... image width and height (both images must be rectified and of same size)
    //        dims[2] ........ bytes per line (often equals width)
    //        replace ........ if this flag is set, the current image is overwritten with
    //                         the input images, otherwise the current image is first copied
    //                         to the previous image (ring buffer functionality, descriptors need
    //                         to be computed only once)
    void pushBack(uint8_t *I1,uint8_t* I2,int32_t* dims,const bool replace);

    // computes features from a single image and pushes it back to a ringbuffer,
    // which interally stores the features of the current and previous image pair
    // use this function for flow computation
    // parameter description see above
    void pushBack(uint8_t *I1,int32_t* dims,const bool replace) { pushBack(I1,0,dims,replace); }

    /**
     * @brief matchFeatures
     *        match features currently stored in ring buffer ( current and previous frames )
     * @param method: [in] 0 = flow, 1 = stereo, 2 = quad matching TODO: replace with enum
     * @param Tr_delta: [in] motion from previous process to better search for matches, if specified.
     */
    void matchFeatures(int32_t method, Matrix *Tr_delta = 0);

    // feature bucketing: keeps only max_features per bucket, where the domain
    // is split into buckets of size (bucket_width,bucket_height)
    /**
     * @brief bucketFeatures
     *        organize the matched points in _p_matched.
     *        keeps only max_feature number of matched points per bucket,
     *        where the domain is split into bucket_width * bucket_height
     * @param max_features: [in] the maximum number of matched points stored in each bucket
     * @param bucket_width: [in] the width of a bucket
     * @param bucket_height: [in] the height of a bucket
     */
    void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

    // return vector with matched feature points and indices
    /**
     * @brief getMatches:
     *        get matched points calculated from dense maximum algorithm
     * @return matched points
     */
    std::vector<Matcher::p_match> getMatches() { return _p_matched_2; }

    // given a vector of inliers computes gain factor between the current and
    // the previous frame. this function is useful if you want to reconstruct 3d
    // and you want to cancel the change of (unknown) camera gain.
    float getGain(std::vector<int32_t> inliers);

private:

    // structure for storing interest points
    // TODO: replace int32_t c with enum
    struct maximum
    {
        int32_t u;   // u-coordinate
        int32_t v;   // v-coordinate
        int32_t val; // value
        int32_t c;   // class
        int32_t d1,d2,d3,d4,d5,d6,d7,d8; // descriptor
        maximum() {}
        maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c) {}
    };

    // u/v ranges for matching stage 0-3
    struct range
    {
        float u_min[4];
        float u_max[4];
        float v_min[4];
        float v_max[4];
    };

    struct delta
    {
        float val[8];
        delta () {}
        delta (float v)
        {
            for (int32_t i=0; i<8; i++)
            {
                val[i] = v;
            }
        }
    };

    // computes the address offset for coordinates u,v of an image of given width
    inline int32_t getAddressOffsetImage(const int32_t& u,const int32_t& v,const int32_t& width)
    {
        return v*width+u;
    }

    // Alexander Neubeck and Luc Van Gool: Efficient Non-Maximum Suppression, ICPR'06, algorithm 4
    /**
     * @brief nonMaximumSuppression:
     *        store the maximum points based on I_f1 and I_f2 feature maps
     * @param I_f1: [in] feature map ( blob5x5 )
     * @param I_f2: [in] feature map ( checkboard5x5 )
     * @param dims: [in] I_f1 and I_f2 dimensions
     * @param maxima: [in/out] stored maximum points
     * @param nms_n: [in/out] the distance between feature point candidates
     */
    void nonMaximumSuppression(int16_t* I_f1,int16_t* I_f2,const int32_t* dims,
                               std::vector<Matcher::maximum> &maxima,int32_t nms_n);

    // descriptor functions
    inline uint8_t saturate(int16_t in);
    void filterImageAll(uint8_t* I,uint8_t* I_du,uint8_t* I_dv,int16_t* I_f1,int16_t* I_f2,const int* dims);
    void filterImageSobel(uint8_t* I,uint8_t* I_du,uint8_t* I_dv,const int* dims);

    /**
     * @brief computeDescriptor:
     *        compute descriptor in a maximum.
     *        This function is called for each maximum in computeDescriptors
     * @param I_du: [in] feature map ( sobel5x5 horizontal direction )
     * @param I_dv: [in] feature map ( sobel5x5 vertical direction )
     * @param bpl: [in] byte per line ( the 16-multiple width )
     * @param u: [in] x position of the point
     * @param v: [in] y position of the point
     * @param desc_addr: [in] d1's first 8 bit address
     */
    inline void computeDescriptor(const uint8_t* I_du,const uint8_t* I_dv,
                                  const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);

    /**
     * @brief computeDescriptor:
     *        compute descriptor in a maximum.
     *        This function only calculate d1-d4 descriptors.
     * @param I_du: [in] feature map ( sobel5x5 horizontal direction )
     * @param I_dv: [in] feature map ( sobel5x5 vertical direction )
     * @param bpl: [in] byte per line ( the 16-multiple width )
     * @param u: [in] x position of the point
     * @param v: [in] y position of the point
     * @param desc_addr: [in] d1's first 8 bit address
     */
    inline void computeSmallDescriptor(const uint8_t* I_du,const uint8_t* I_dv,
                                       const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
    /**
     * @brief computeDescriptors:
     *        compute descriptors in each maximum extracted from nonMaximumSuppression
     * @param I_du: [in] feature map ( sobel5x5 horizontal direction )
     * @param I_dv: [in] feature map ( sobel5x5 vertical direction )
     * @param bpl: [in] byte per line ( the 16-multiple width )
     * @param maxima: [in/out] stored maximums
     */
    void computeDescriptors(uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<Matcher::maximum> &maxima);

    /**
     * @brief getHalfResolutionDimensions:
     *        calculate half dimensions
     * @param dims: [in] input dimensions
     * @param dims_half: [out] output half dimensions
     */
    void getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half);

    /**
     * @brief createHalfResolutionImage
     *        using 16-multiple-width speeds up the convertion.
     * @param I: [in] input image
     * @param dims: [in] input image dimension
     * @return half resolution image
     */
    uint8_t* createHalfResolutionImage(uint8_t *I,const int32_t* dims);

    /**
     * @brief computeFeatures:
     *        compute feature descriptors from input image
     * @param I: [in] input image
     * @param dims: [in] input image dimensions - [width, height, 16-multiple width]
     * @param max1: [in/out] sparse maximums (1st pass) with descriptors
     * @param num1: [in/out] number of sparse maximums
     * @param max2: [in/out] dense maximums (2nd pass) with descriptors
     * @param num2: [in/out] number of dense maximums
     * @param I_du: [in/out] feature map ( sobel5x5 horizontal direction ) from half resolution image
     * @param I_dv: [in/out] feature map ( sobel5x5 vertical direction ) from half resolution image
     * @param I_du_full: [in/out] feature map ( sobel5x5 horizontal direction )
     * @param I_dv_full: [in/out] feature map ( sobel5x5 vertical direction )
     * WARNING: max,I_du,I_dv has to be freed by yourself!
     */
    void computeFeatures(uint8_t *I,const int32_t* dims,
                         int32_t* &max1, int32_t &num1,
                         int32_t* &max2, int32_t &num2,
                         uint8_t* &I_du, uint8_t* &I_dv,
                         uint8_t* &I_du_full, uint8_t* &I_dv_full);

    // matching functions
    void computePriorStatistics (std::vector<Matcher::p_match> &p_matched,int32_t method);

    /**
     * @brief createIndexVector:
     *        create an array of vector.
     *        The array represents a 3D matrix, class_num by v_bin_num by u_bin_num, in one dimension.
     *        Each grid in the array is a vector.
     * @param m: [in] maximums
     * @param n: [in] number of maximums
     * @param k: [in/out] index vector pointer
     *           the vector can be considered as a (class_num by u_bin_num by v_bin_num) 3D matrix
     *           stored in one dimension vector.
     * @param u_bin_num: [in] bin number in horizontal direction
     * @param v_bin_num: [in] bin number in vertical direction
     */
    void createIndexVector(int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num);


    /**
     * @brief findMatch
     *
     * @param m1: [in] maximums
     * @param i1: [in] maximum index in m1
     * @param m2: [in] maximums
     * @param step_size: [in] step size in m1 or m2's memory.
     * @param k2: [in] index vector of m2
     * @param u_bin_num: [in] bin number in horizontal direction
     * @param v_bin_num: [in] bin number in vertical direction
     * @param stat_bin: [in] used to limit the search area if use_prior is true
     * @param min_ind: [in/out] the index of the best matched candidate in m2
     * @param stage: [in] used to limit the search area if use_prior is true
     * @param flow: [in] true if the matching method is STEREO. limit the search in "horizontal" direction
     * @param use_prior: [in] true if using the prior information ( restrict the search area )
     * @param u_: [in] horizontal motion from precious image to current image
     * @param v_: [in] vertical motion from precious image to current image
     */
    inline void findMatch(int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                          std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                          int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);

    /**
     * @brief matching:
     *        search the matched maximums which appear in previous 1st+2nd images and also current 1st+2nd images
     * @param m1p: [in] maximums in previous 1st frame
     * @param m2p: [in] maximums in previous 2nd frame
     * @param m1c: [in] maximums in current 1st frame
     * @param m2c: [in] maximums in current 2nd frame
     * @param n1p: [in] number of maximums in previous 1st frame
     * @param n2p: [in] number of maximums in previous 2nd frame
     * @param n1c: [in] number of maximums in current 1st frame
     * @param n2c: [in] number of maximums in current 2nd frame
     * @param p_matched: [in/out] stored p_match points
     * @param method: [in] the matching method
     * @param use_prior: [in] flag to determine if using prior information ( sparse information )
     * @param Tr_delta: [in] motion estimated from previous frame
     */
    void matching(int32_t *m1p,int32_t *m2p,int32_t *m1c,int32_t *m2c,
                  int32_t n1p,int32_t n2p,int32_t n1c,int32_t n2c,
                  std::vector<Matcher::p_match> &p_matched,int32_t method,bool use_prior,Matrix *Tr_delta = 0);

    // outlier removal
    void removeOutliers(std::vector<Matcher::p_match> &p_matched,int32_t method);

    // parabolic fitting
    bool parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                          const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                          const float &u1,const float &v1,
                          float       &u2,float       &v2,
                          Matrix At,Matrix AtA,
                          uint8_t* desc_buffer);
    void relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                         const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                         const float &u1,const float &v1,
                         float       &u2,float       &v2,
                         uint8_t* desc_buffer);
    void refinement(std::vector<Matcher::p_match> &p_matched,int32_t method);

    // mean for gain computation
    inline float mean(const uint8_t* I,const int32_t &bpl,const int32_t &u_min,const int32_t &u_max,const int32_t &v_min,const int32_t &v_max);

    // parameter naming rule:
    // 'c': current
    // 'p': previous
    // 'v': vertical direction
    // 'u': horizontal direction
    // 'm': struct maximum
    // 'n': number of struct maximum
    // 1st '1': first image ( left image in the current design )
    // 1st '2': second image ( right image in the current design )
    // 2nd '1': 1st pass via non-maximum suppression: sparse maximum
    // 2nd '2': 2nd pass via non-maximum suppression: dense maximum
    parameters _param;
    int32_t    _margin;
    int32_t* _m1p1;
    int32_t* _m2p1;
    int32_t* _m1c1;
    int32_t* _m2c1;
    int32_t* _m1p2;
    int32_t* _m2p2;
    int32_t* _m1c2;
    int32_t* _m2c2;
    int32_t _n1p1;
    int32_t _n2p1;
    int32_t _n1c1;
    int32_t _n2c1;
    int32_t _n1p2;
    int32_t _n2p2;
    int32_t _n1c2;
    int32_t _n2c2;
    uint8_t* _I1p;
    uint8_t* _I2p;
    uint8_t* _I1c;
    uint8_t* _I2c;
    uint8_t* _I1p_du;
    uint8_t* _I2p_du;
    uint8_t* _I1c_du;
    uint8_t* _I2c_du;
    uint8_t* _I1p_dv;
    uint8_t* _I2p_dv;
    uint8_t* _I1c_dv;
    uint8_t* _I2c_dv;
    uint8_t* _I1p_du_full;
    uint8_t* _I2p_du_full;
    uint8_t* _I1c_du_full;
    uint8_t* _I2c_du_full; // only needed for
    uint8_t* _I1p_dv_full;
    uint8_t* _I2p_dv_full;
    uint8_t* _I1c_dv_full;
    uint8_t* _I2c_dv_full; // half-res matching
    int32_t _dims_p[3]; // [0]: width [1]: height [2]: new width ( multiple of 16 )
    int32_t _dims_c[3];

    std::vector<Matcher::p_match> _p_matched_1; // matched points in sparse maximum
    std::vector<Matcher::p_match> _p_matched_2; // matched points in dense maximum
    std::vector<Matcher::range>   _ranges;
};

#endif
