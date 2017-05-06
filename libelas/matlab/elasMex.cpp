/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "mex.h"
#include "elas.h"

using namespace std;

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // check for proper number of arguments
  if (nrhs!=2 && nrhs!=3)
    mexErrMsgTxt("2 or 3 inputs required: I1 (left image), I2 (right image), [param].");
  if (nlhs!=2) 
    mexErrMsgTxt("2 outputs required: D1 (left disparities), D2 (right disparities).");  

  // check for proper argument types and sizes
  if (!mxIsUint8(prhs[0]) || mxGetNumberOfDimensions(prhs[0])!=2)
    mexErrMsgTxt("Input I1 (left image) must be a uint8 image.");
  if (!mxIsUint8(prhs[1]) || mxGetNumberOfDimensions(prhs[1])!=2)
    mexErrMsgTxt("Input I2 (right image) must be a uint8 image.");
  if (mxGetM(prhs[0])!=mxGetM(prhs[1]) || mxGetN(prhs[0])!=mxGetN(prhs[1]))
    mexErrMsgTxt("Input I1 and I2 must be images of the same size.");
  
  // load default robotics parameter
  Elas::parameters param(Elas::ROBOTICS);
  //Elas::parameters param(Elas::MIDDLEBURY);
   
  // read parameters from input
  if (nrhs==3) {
    
    // check if we have a parameter structure
    if (!mxIsStruct(prhs[2]))
      mexErrMsgTxt("Input param be a structure.");

    // for all fields of parameter structure overwrite parameters
    for (int32_t i=0; i<mxGetNumberOfFields(prhs[2]); i++) {
      const char *field_name = mxGetFieldNameByNumber(prhs[2],i);
      mxArray    *field_val  = mxGetFieldByNumber(prhs[2],0,i);
      if (mxIsDouble(field_val)) {
        double val = *((double*)mxGetPr(field_val));
        if (!strcmp(field_name,"disp_min"))              param.disp_min = val;
        if (!strcmp(field_name,"disp_max"))              param.disp_max = val;
        if (!strcmp(field_name,"support_threshold"))     param.support_threshold = val;
        if (!strcmp(field_name,"support_texture"))       param.support_texture = val;
        if (!strcmp(field_name,"candidate_stepsize"))    param.candidate_stepsize = val;
        if (!strcmp(field_name,"incon_window_size"))     param.incon_window_size = val;
        if (!strcmp(field_name,"incon_threshold"))       param.incon_threshold = val;
        if (!strcmp(field_name,"incon_min_support"))     param.incon_min_support = val;
        if (!strcmp(field_name,"add_corners"))           param.add_corners = val;
        if (!strcmp(field_name,"grid_size"))             param.grid_size = val;
        if (!strcmp(field_name,"beta"))                  param.beta = val;
        if (!strcmp(field_name,"gamma"))                 param.gamma = val;
        if (!strcmp(field_name,"sigma"))                 param.sigma = val;
        if (!strcmp(field_name,"sradius"))               param.sradius = val;
        if (!strcmp(field_name,"match_texture"))         param.match_texture = val;
        if (!strcmp(field_name,"lr_threshold"))          param.lr_threshold = val;        
        if (!strcmp(field_name,"speckle_sim_threshold")) param.speckle_sim_threshold = val;
        if (!strcmp(field_name,"speckle_size"))          param.speckle_size = val;
        if (!strcmp(field_name,"ipol_gap_width"))        param.ipol_gap_width = val;
        if (!strcmp(field_name,"filter_median"))         param.filter_median = val;
        if (!strcmp(field_name,"filter_adaptive_mean"))  param.filter_adaptive_mean = val;
        if (!strcmp(field_name,"postprocess_only_left")) param.postprocess_only_left = val;
        if (!strcmp(field_name,"subsampling"))           param.subsampling = val;
      }
    }
  }

  // get input pointers
  uint8_t*       I1     = (uint8_t*)mxGetPr(prhs[0]);
  uint8_t*       I2     = (uint8_t*)mxGetPr(prhs[1]);
  const int32_t *I_dims = mxGetDimensions(prhs[0]);
  
  // create outputs
  int32_t D_dims[2] = {I_dims[0],I_dims[1]};
  if (param.subsampling) {
    D_dims[0] = I_dims[0]/2;
    D_dims[1] = I_dims[1]/2;
  }
  plhs[0]     = mxCreateNumericArray(2,D_dims,mxSINGLE_CLASS,mxREAL);
  float*   D1 = (float*)mxGetPr(plhs[0]);
  plhs[1]     = mxCreateNumericArray(2,D_dims,mxSINGLE_CLASS,mxREAL);
  float*   D2 = (float*)mxGetPr(plhs[1]);
  
  // create image dimension variable containing bytes-per-line information
  const int32_t I_dims_with_bpl[] = {I_dims[0],I_dims[1],I_dims[0]};
  
  // perform matching
  Elas elas(param);
  elas.process(I1,I2,D1,D2,I_dims_with_bpl);
}
