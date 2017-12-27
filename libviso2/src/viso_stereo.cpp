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

#include "viso_stereo.h"

using namespace std;

VisualOdometryStereo::VisualOdometryStereo( parameters param ) :
                      VisualOdometry( param ),
                      _param( param )
{
    _matcher->setIntrinsics(param.calib.f,param.calib.cu,param.calib.cv,param.base);
}

//==============================================================================//

VisualOdometryStereo::~VisualOdometryStereo()
{
}

//==============================================================================//

bool VisualOdometryStereo::process(uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace)
{
    // push back images
    _matcher->pushBack(I1, I2, dims, replace);

    // bootstrap motion estimate if invalid
    if(!_Tr_valid)
    {
        _matcher->matchFeatures(2);
        _matcher->bucketFeatures(_param.bucket.max_features, _param.bucket.bucket_width, _param.bucket.bucket_height);
        _p_matched = _matcher->getMatches();
        updateMotion();
    }

    // match features and update motion
    if (_Tr_valid)
    {
        _matcher->matchFeatures(2,&_Tr_delta);
    }
    else
    {
        _matcher->matchFeatures(2);
    }

    _matcher->bucketFeatures(_param.bucket.max_features, _param.bucket.bucket_width, _param.bucket.bucket_height);
    _p_matched = _matcher->getMatches();
    return updateMotion();
}

//==============================================================================//

vector<double> VisualOdometryStereo::estimateMotion(vector<Matcher::p_match> p_matched)
{
    // return value
    bool success = true;

    // compute minimum distance for RANSAC samples
    double width=0,height=0;
    for(vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++)
    {
        if (it->u1c>width)  width  = it->u1c;
        if (it->v1c>height) height = it->v1c;
    }
    //double min_dist = min(width,height)/3.0;

    // get number of matches
    int32_t N  = p_matched.size();
    if(N<6)
    {
        return vector<double>();
    }

    // allocate dynamic memory
    _X          = new double[N];
    _Y          = new double[N];
    _Z          = new double[N];
    _J          = new double[4*N*6];
    _p_predict  = new double[4*N];
    _p_observe  = new double[4*N];
    _p_residual = new double[4*N];

    // project matches of previous image into 3d
    for(int32_t i=0; i<N; i++)
    {
        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        _X[i] = (p_matched[i].u1p - _param.calib.cu) * _param.base / d;
        _Y[i] = (p_matched[i].v1p - _param.calib.cv) * _param.base / d;
        _Z[i] = _param.calib.f * _param.base / d;
    }

    // loop variables
    vector<double> tr_delta;
    vector<double> tr_delta_curr;
    tr_delta_curr.resize(6);

    // clear parameter vector
    _inliers.clear();

    // initial RANSAC estimate
    for(int32_t k = 0; k < _param.ransac_iters; k++)
    {
        // draw random sample set
        vector<int32_t> active = getRandomSample(N,3);

        // clear parameter vector
        for(int32_t i=0; i<6; i++)
        {
            tr_delta_curr[i] = 0;
        }

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        int32_t iter=0;
        while(result==UPDATED)
        {
            result = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
            if(iter++ > 20 || result==CONVERGED)
            {
                break;
            }
        }

        // overwrite best parameters if we have more inliers
        if(result!=FAILED)
        {
            vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
            if(inliers_curr.size() > _inliers.size())
            {
                _inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }

    // final optimization (refinement)
    if (_inliers.size()>=6)
    {
        int32_t iter=0;
        VisualOdometryStereo::result result = UPDATED;
        while (result==UPDATED)
        {
            result = updateParameters(p_matched, _inliers, tr_delta, 1, 1e-8);
            if (iter++ > 100 || result==CONVERGED)
            {
                break;
            }
        }

        // not converged
        if (result!=CONVERGED)
        {
            success = false;
        }
    }
    // not enough inliers
    else
    {
        success = false;
    }

    // release dynamic memory
    delete[] _X;
    delete[] _Y;
    delete[] _Z;
    delete[] _J;
    delete[] _p_predict;
    delete[] _p_observe;
    delete[] _p_residual;

    // parameter estimate succeeded?
    if (success) return tr_delta;
    else         return vector<double>();
}

//==============================================================================//

vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched,vector<double> &tr)
{
    // mark all observations active
    vector<int32_t> active;
    for(int32_t i=0; i<(int32_t)p_matched.size(); i++)
    {
        active.push_back(i);
    }

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);

    // compute inliers
    vector<int32_t> inliers;
    for(int32_t i=0; i<(int32_t)p_matched.size(); i++)
    {
        if(pow(_p_observe[4*i+0]-_p_predict[4*i+0],2)+pow(_p_observe[4*i+1]-_p_predict[4*i+1],2) +
           pow(_p_observe[4*i+2]-_p_predict[4*i+2],2)+pow(_p_observe[4*i+3]-_p_predict[4*i+3],2) < _param.inlier_threshold * _param.inlier_threshold)
      {
            inliers.push_back(i);
      }
    }
    return inliers;
}

//==============================================================================//

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(vector<Matcher::p_match> &p_matched,
                                                                    vector<int32_t> &active,vector<double> &tr,double step_size,double eps)
{
    // we need at least 3 observations
    if (active.size()<3)
    {
        return FAILED;
    }

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);

    // init
    Matrix A(6,6);
    Matrix B(6,1);

    // fill matrices A and B
    for (int32_t m=0; m<6; m++)
    {
        for (int32_t n=0; n<6; n++)
        {
            double a = 0;
            for (int32_t i=0; i<4*(int32_t)active.size(); i++)
            {
                a += _J[i*6+m]*_J[i*6+n];
            }
            A._val[m][n] = a;
        }
        double b = 0;
        for (int32_t i=0; i<4*(int32_t)active.size(); i++)
        {
            b += _J[i*6+m]*(_p_residual[i]);
        }
        B._val[m][0] = b;
    }

    // perform elimination
    if (B.solve(A))
    {
        bool converged = true;
        for (int32_t m=0; m<6; m++)
        {
            tr[m] += step_size*B._val[m][0];
            if (fabs(B._val[m][0])>eps)
            {
                converged = false;
            }
        }
        if (converged) return CONVERGED;
        else return UPDATED;
    }
    else
    {
        return FAILED;
    }
}

//==============================================================================//

void VisualOdometryStereo::computeObservations(vector<Matcher::p_match> &p_matched,vector<int32_t> &active)
{
    // set all observations
    for (int32_t i=0; i<(int32_t)active.size(); i++)
    {
        _p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
        _p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
        _p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
        _p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
    }
}

//==============================================================================//

void VisualOdometryStereo::computeResidualsAndJacobian(vector<double> &tr,vector<int32_t> &active)
{
    // extract motion parameters
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

    // precompute sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

    // compute rotation matrix and derivatives
    double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
    double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
    double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
    double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
    double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
    double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
    double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
    double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
    double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
    double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
    double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

    // loop variables
    double X1p,Y1p,Z1p;
    double X1c,Y1c,Z1c,X2c;
    double X1cd,Y1cd,Z1cd;

    // for all observations do
    for (int32_t i=0; i<(int32_t)active.size(); i++)
    {
        // get 3d point in previous coordinate system
        X1p = _X[active[i]];
        Y1p = _Y[active[i]];
        Z1p = _Z[active[i]];

        // compute 3d point in current left coordinate system
        X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
        Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
        Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

        // weighting
        double weight = 1.0;
        if (_param.reweighting)
        {
            weight = 1.0/(fabs(_p_observe[4*i+0] - _param.calib.cu)/fabs(_param.calib.cu) + 0.05);
        }

        // compute 3d point in current right coordinate system
        X2c = X1c - _param.base;

        // for all paramters do
        for (int32_t j=0; j<6; j++)
        {
            // derivatives of 3d pt. in curr. left coordinates wrt. param j
            switch (j)
            {
                case 0: X1cd = 0;
                        Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                        Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                        break;
                case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                        Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                        Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                        break;
                case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                        Y1cd = rdrz10*X1p+rdrz11*Y1p;
                        Z1cd = rdrz20*X1p+rdrz21*Y1p;
                        break;
                case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
                case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
                case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
            }

            // set jacobian entries (project via K)
            _J[(4*i+0)*6+j] = weight * _param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
            _J[(4*i+1)*6+j] = weight * _param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
            _J[(4*i+2)*6+j] = weight * _param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
            _J[(4*i+3)*6+j] = weight * _param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
        }

        // set prediction (project via K)
        _p_predict[4*i+0] = _param.calib.f * X1c / Z1c + _param.calib.cu; // left u
        _p_predict[4*i+1] = _param.calib.f * Y1c / Z1c + _param.calib.cv; // left v
        _p_predict[4*i+2] = _param.calib.f * X2c / Z1c + _param.calib.cu; // right u
        _p_predict[4*i+3] = _param.calib.f * Y1c / Z1c + _param.calib.cv; // right v

        // set residuals
        _p_residual[4*i+0] = weight*(_p_observe[4*i+0]-_p_predict[4*i+0]);
        _p_residual[4*i+1] = weight*(_p_observe[4*i+1]-_p_predict[4*i+1]);
        _p_residual[4*i+2] = weight*(_p_observe[4*i+2]-_p_predict[4*i+2]);
        _p_residual[4*i+3] = weight*(_p_observe[4*i+3]-_p_predict[4*i+3]);
    }
}
