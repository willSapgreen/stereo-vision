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

#include "matrix.h"
#include <math.h>
#include <algorithm>

using namespace std;

#define SWAP(a,b) {temp=a;a=b;b=temp;}
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static FLOAT sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
static FLOAT maxarg1,maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))
static int32_t iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))


using namespace std;

Matrix::Matrix()
{
    _m   = 0;
    _n   = 0;
    _val = 0;
}

//==============================================================================//

Matrix::Matrix(const int32_t m,const int32_t n)
{
    allocateMemory(m, n);
}

//==============================================================================//

Matrix::Matrix(const int32_t m,const int32_t n,const FLOAT* val)
{
    allocateMemory(m,n);
    int32_t k=0;
    for (int32_t i=0; i<m; i++)
    {
        for (int32_t j=0; j<n; j++)
        {
            _val[i][j] = val[k++];
        }
    }
}

//==============================================================================//

Matrix::Matrix(const Matrix &M)
{
    allocateMemory(M._m,M._n);
    for (int32_t i=0; i<M._m; i++)
    {
        memcpy(_val[i],M._val[i],M._n*sizeof(FLOAT));
    }
}

//==============================================================================//

Matrix::~Matrix()
{
    releaseMemory();
}

//==============================================================================//

Matrix& Matrix::operator=(const Matrix &M)
{
    if (this!=&M)
    {
        if (M._m!=_m || M._n!=_n)
        {
            releaseMemory();
            allocateMemory(M._m,M._n);
        }
        if (M._n>0)
        {
            for (int32_t i=0; i<M._m; i++)
            {
                memcpy(_val[i],M._val[i],M._n*sizeof(FLOAT));
            }
        }
    }
    return *this;
}

//==============================================================================//

void Matrix::getData(FLOAT* val_,int32_t i1,int32_t j1,int32_t i2,int32_t j2)
{
    if (i2==-1) i2 = _m-1;
    if (j2==-1) j2 = _n-1;
    int32_t k=0;
    for (int32_t i=i1; i<=i2; i++)
    {
        for (int32_t j=j1; j<=j2; j++)
        {
            val_[k++] = _val[i][j];
        }
    }
}

//==============================================================================//

Matrix Matrix::getMat(int32_t i1,int32_t j1,int32_t i2,int32_t j2)
{
    if (i2==-1) i2 = _m-1;
    if (j2==-1) j2 = _n-1;
    if (i1<0 || i2>=_m || j1<0 || j2>=_n || i2<i1 || j2<j1)
    {
        cerr << "ERROR: Cannot get submatrix [" << i1 << ".." << i2 <<
            "] x [" << j1 << ".." << j2 << "]" <<
            " of a (" << _m << "x" << _n << ") matrix." << endl;
        exit(0);
    }
    Matrix M(i2-i1+1,j2-j1+1);
    for (int32_t i=0; i<M._m; i++)
    {
        for (int32_t j=0; j<M._n; j++)
        {
            M._val[i][j] = _val[i1+i][j1+j];
        }
    }
    return M;
}

//==============================================================================//

void Matrix::setMat(const Matrix &M,const int32_t i1,const int32_t j1)
{
    if (i1<0 || j1<0 || i1+M._m>_m || j1+M._n>_n)
    {
        cerr << "ERROR: Cannot set submatrix [" << i1 << ".." << i1+M._m-1 <<
            "] x [" << j1 << ".." << j1+M._n-1 << "]" <<
            " of a (" << _m << "x" << _n << ") matrix." << endl;
        exit(0);
    }
    for (int32_t i=0; i<M._m; i++)
    {
        for (int32_t j=0; j<M._n; j++)
        {
            _val[i1+i][j1+j] = M._val[i][j];
        }
    }
}

//==============================================================================//

void Matrix::setVal(FLOAT s,int32_t i1,int32_t j1,int32_t i2,int32_t j2)
{
    if (i2==-1) i2 = _m-1;
    if (j2==-1) j2 = _n-1;
    if (i2<i1 || j2<j1)
    {
        cerr << "ERROR in setVal: Indices must be ordered (i1<=i2, j1<=j2)." << endl;
        exit(0);
    }
    for (int32_t i=i1; i<=i2; i++)
    {
        for (int32_t j=j1; j<=j2; j++)
        {
            _val[i][j] = s;
        }
    }
}

//==============================================================================//

void Matrix::setDiag(FLOAT s,int32_t i1,int32_t i2)
{
    if (i2==-1) i2 = min(_m-1,_n-1);

    for (int32_t i=i1; i<=i2; i++)
    {
        _val[i][i] = s;
    }
}

//==============================================================================//

void Matrix::zero()
{
    setVal(0);
}

//==============================================================================//

Matrix Matrix::extractCols(vector<int> idx)
{
    Matrix M(_m,idx.size());
    for (int32_t j=0; j<M._n; j++)
    {
        if (idx[j]<_n)
        {
            for (int32_t i=0; i<_m; i++)
            {
                M._val[i][j] = _val[i][idx[j]];
            }
        }
    }
    return M;
}

//==============================================================================//

Matrix Matrix::eye(const int32_t _m)
{
    Matrix M(_m,_m);
    for (int32_t i=0; i<_m; i++)
    {
        M._val[i][i] = 1;
    }
    return M;
}

//==============================================================================//

void Matrix::eye()
{
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            _val[i][j] = 0;
        }
    }
    for (int32_t i=0; i<min(_m,_n); i++)
    {
        _val[i][i] = 1;
    }
}

//==============================================================================//

Matrix Matrix::diag(const Matrix &M)
{
    if (M._m>1 && M._n==1)
    {
        Matrix D(M._m,M._m);
        for (int32_t i=0; i<M._m; i++)
        {
            D._val[i][i] = M._val[i][0];
        }
        return D;
    }
    else if (M._m==1 && M._n>1)
    {
        Matrix D(M._n,M._n);
        for (int32_t i=0; i<M._n; i++)
        {
            D._val[i][i] = M._val[0][i];
        }
        return D;
    }
    cout << "ERROR: Trying to create diagonal matrix from vector of size (" << M._m << "x" << M._n << ")" << endl;
    exit(0);
}

//==============================================================================//

Matrix Matrix::reshape(const Matrix &M,int32_t m_,int32_t n_)
{
    if (M._m*M._n != m_*n_)
    {
        cerr << "ERROR: Trying to reshape a matrix of size (" << M._m << "x" << M._n <<
                ") to size (" << m_ << "x" << n_ << ")" << endl;
        exit(0);
    }
    Matrix M2(m_,n_);
    for (int32_t k=0; k<m_*n_; k++)
    {
        int32_t i1 = k/M._n;
        int32_t j1 = k%M._n;
        int32_t i2 = k/n_;
        int32_t j2 = k%n_;
        M2._val[i2][j2] = M._val[i1][j1];
    }
    return M2;
}

//==============================================================================//

Matrix Matrix::rotMatX(const FLOAT &angle)
{
    FLOAT s = sin(angle);
    FLOAT c = cos(angle);
    Matrix R(3,3);
    R._val[0][0] = +1;
    R._val[1][1] = +c;
    R._val[1][2] = -s;
    R._val[2][1] = +s;
    R._val[2][2] = +c;
    return R;
}

//==============================================================================//

Matrix Matrix::rotMatY(const FLOAT &angle)
{
    FLOAT s = sin(angle);
    FLOAT c = cos(angle);
    Matrix R(3,3);
    R._val[0][0] = +c;
    R._val[0][2] = +s;
    R._val[1][1] = +1;
    R._val[2][0] = -s;
    R._val[2][2] = +c;
    return R;
}

//==============================================================================//

Matrix Matrix::rotMatZ(const FLOAT &angle)
{
    FLOAT s = sin(angle);
    FLOAT c = cos(angle);
    Matrix R(3,3);
    R._val[0][0] = +c;
    R._val[0][1] = -s;
    R._val[1][0] = +s;
    R._val[1][1] = +c;
    R._val[2][2] = +1;
    return R;
}

//==============================================================================//

Matrix Matrix::operator+(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A._m!=B._m || A._n!=B._n)
    {
        cerr << "ERROR: Trying to add matrices of size (" << A._m << "x" << A._n <<
            ") and (" << B._m << "x" << B._n << ")" << endl;
        exit(0);
    }
    Matrix C(A._m,A._n);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[i][j] = A._val[i][j]+B._val[i][j];
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator-(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A._m!=B._m || A._n!=B._n)
    {
        cerr << "ERROR: Trying to subtract matrices of size (" << A._m << "x" << A._n <<
            ") and (" << B._m << "x" << B._n << ")" << endl;
        exit(0);
    }
    Matrix C(A._m,A._n);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[i][j] = A._val[i][j]-B._val[i][j];
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator*(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A._n!=B._m)
    {
        cerr << "ERROR: Trying to multiply matrices of size (" << A._m << "x" << A._n <<
            ") and (" << B._m << "x" << B._n << ")" << endl;
        exit(0);
    }
    Matrix C(A._m,B._n);
    for (int32_t i=0; i<A._m; i++)
    {
        for (int32_t j=0; j<B._n; j++)
        {
            for (int32_t k=0; k<A._n; k++)
            {
                C._val[i][j] += A._val[i][k]*B._val[k][j];
            }
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator*(const FLOAT &s)
{
    Matrix C(_m,_n);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[i][j] = _val[i][j]*s;
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator/(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;

    if (A._m==B._m && A._n==B._n)
    {
        Matrix C(A._m,A._n);
        for (int32_t i=0; i<A._m; i++)
        {
            for (int32_t j=0; j<A._n; j++)
            {
                if (B._val[i][j]!=0)
                {
                    C._val[i][j] = A._val[i][j]/B._val[i][j];
                }
            }
        }
        return C;
    }
    else if (A._m==B._m && B._n==1)
    {
        Matrix C(A._m,A._n);
        for (int32_t i=0; i<A._m; i++)
        {
            for (int32_t j=0; j<A._n; j++)
            {
                if (B._val[i][0]!=0)
                {
                    C._val[i][j] = A._val[i][j]/B._val[i][0];
                }
            }
        }
        return C;
    }
    else if (A._n==B._n && B._m==1)
    {
        Matrix C(A._m,A._n);
        for (int32_t i=0; i<A._m; i++)
        {
            for (int32_t j=0; j<A._n; j++)
            {
                if (B._val[0][j]!=0)
                {
                    C._val[i][j] = A._val[i][j]/B._val[0][j];
                }
            }
        }
        return C;
    }
    else
    {
        cerr << "ERROR: Trying to divide matrices of size (" << A._m << "x" << A._n <<
            ") and (" << B._m << "x" << B._n << ")" << endl;
        exit(0);
    }
}

//==============================================================================//

Matrix Matrix::operator/(const FLOAT &s)
{
    if (fabs(s)<1e-20)
    {
        cerr << "ERROR: Trying to divide by zero!" << endl;
        exit(0);
    }
    Matrix C(_m,_n);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[i][j] = _val[i][j]/s;
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator-()
{
    Matrix C(_m,_n);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[i][j] = -_val[i][j];
        }
    }
    return C;
}

//==============================================================================//

Matrix Matrix::operator~()
{
    Matrix C(_n,_m);
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            C._val[j][i] = _val[i][j];
        }
    }
    return C;
}

//==============================================================================//

FLOAT Matrix::l2norm()
{
    FLOAT norm = 0;
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            norm += _val[i][j]*_val[i][j];
        }
    }
    return sqrt(norm);
}

//==============================================================================//

FLOAT Matrix::mean()
{
    FLOAT mean = 0;
    for (int32_t i=0; i<_m; i++)
    {
        for (int32_t j=0; j<_n; j++)
        {
            mean += _val[i][j];
        }
    }
    return mean/(FLOAT)(_m*_n);
}

//==============================================================================//

Matrix Matrix::cross(const Matrix &a, const Matrix &b)
{
    if (a._m!=3 || a._n!=1 || b._m!=3 || b._n!=1)
    {
        cerr << "ERROR: Cross product vectors must be of size (3x1)" << endl;
        exit(0);
    }
    Matrix c(3,1);
    c._val[0][0] = a._val[1][0]*b._val[2][0]-a._val[2][0]*b._val[1][0];
    c._val[1][0] = a._val[2][0]*b._val[0][0]-a._val[0][0]*b._val[2][0];
    c._val[2][0] = a._val[0][0]*b._val[1][0]-a._val[1][0]*b._val[0][0];
    return c;
}

//==============================================================================//

Matrix Matrix::inv(const Matrix &M)
{
    if (M._m!=M._n)
    {
        cerr << "ERROR: Trying to invert matrix of size (" << M._m << "x" << M._n << ")" << endl;
        exit(0);
    }
    Matrix A(M);
    Matrix B = eye(M._m);
    B.solve(A);
    return B;
}

//==============================================================================//

bool Matrix::inv()
{
    if (_m!=_n)
    {
        cerr << "ERROR: Trying to invert matrix of size (" << _m << "x" << _n << ")" << endl;
        exit(0);
    }
    Matrix A(*this);
    eye();
    solve(A);
    return true;
}

//==============================================================================//

FLOAT Matrix::det()
{
    if (_m != _n)
    {
        cerr << "ERROR: Trying to compute determinant of a matrix of size (" << _m << "x" << _n << ")" << endl;
        exit(0);
    }

    Matrix A(*this);
    int32_t *idx = (int32_t*)malloc(_m*sizeof(int32_t));
    FLOAT d = 1;
    A.lu(idx,d);
    for( int32_t i=0; i<_m; i++)
    {
        d *= A._val[i][i];
    }
    free(idx);
    return d;
}

//==============================================================================//

bool Matrix::solve(const Matrix &M, FLOAT eps)
{
    // substitutes
    const Matrix &A = M;
    Matrix &B       = *this;

    if (A._m != A._n || A._m != B._m || A._m<1 || B._n<1)
    {
        cerr << "ERROR: Trying to eliminate matrices of size (" << A._m << "x" << A._n <<
                ") and (" << B._m << "x" << B._n << ")" << endl;
        exit(0);
    }

    // index vectors for bookkeeping on the pivoting
    int32_t* indxc = new int32_t[_m];
    int32_t* indxr = new int32_t[_m];
    int32_t* ipiv  = new int32_t[_m];

    // loop variables
    int32_t i, icol, irow, j, k, l, ll;
    FLOAT big, dum, pivinv, temp;
    icol = 0;
    irow = 0;

    // initialize pivots to zero
    for (j=0;j<_m;j++) ipiv[j]=0;

    // main loop over the columns to be reduced
    for (i=0;i<_m;i++)
    {
        big=0.0;

        // search for a pivot element
        for (j=0;j<_m;j++)
        {
            if (ipiv[j]!=1)
            {
                for (k=0;k<_m;k++)
                {
                    if (ipiv[k]==0)
                    {
                        if (fabs(A._val[j][k])>=big)
                        {
                            big=fabs(A._val[j][k]);
                            irow=j;
                            icol=k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);

        // We now have the pivot element, so we interchange rows, if needed, to put the pivot
        // element on the diagonal. The columns are not physically interchanged, only relabeled.
        if (irow != icol)
        {
            for (l=0;l<_m;l++) SWAP(A._val[irow][l], A._val[icol][l])
            for (l=0;l<_n;l++) SWAP(B._val[irow][l], B._val[icol][l])
        }

        indxr[i]=irow; // We are now ready to divide the pivot row by the
        indxc[i]=icol; // pivot element, located at irow and icol.

        // check for singularity
        if (fabs(A._val[icol][icol]) < eps)
        {
            delete[] indxc;
            delete[] indxr;
            delete[] ipiv;
            return false;
        }

        pivinv=1.0/A._val[icol][icol];
        A._val[icol][icol]=1.0;
        for (l=0;l<_m;l++) A._val[icol][l] *= pivinv;
        for (l=0;l<_n;l++) B._val[icol][l] *= pivinv;

        // Next, we reduce the rows except for the pivot one
        for (ll=0;ll<_m;ll++)
        {
            if (ll!=icol)
            {
                dum = A._val[ll][icol];
                A._val[ll][icol] = 0.0;
                for (l=0;l<_m;l++) A._val[ll][l] -= A._val[icol][l]*dum;
                for (l=0;l<_n;l++) B._val[ll][l] -= B._val[icol][l]*dum;
            }
        }
    }

    // This is the end of the main loop over columns of the reduction. It only remains to unscramble
    // the solution in view of the column interchanges. We do this by interchanging pairs of
    // columns in the reverse order that the permutation was built up.
    for (l=_m-1;l>=0;l--)
    {
        if (indxr[l]!=indxc[l])
        {
          for (k=0;k<_m;k++)
          {
            SWAP(A._val[k][indxr[l]], A._val[k][indxc[l]])
          }
        }
    }

    // success
    delete[] indxc;
    delete[] indxr;
    delete[] ipiv;
    return true;
}

//==============================================================================//

// Given a matrix a[1.._n][1.._n], this routine replaces it by the LU decomposition of a rowwise
// permutation of itself. a and _n are input. a is output, arranged as in equation (2.3.14) above;
// indx[1.._n] is an output vector that records the row permutation effected by the partial
// pivoting; d is output as ±1 depending on whether the number of row interchanges was even
// or odd, respectively. This routine is used in combination with lubksb to solve linear equations
// or invert a matrix.

bool Matrix::lu(int32_t *idx, FLOAT &d)//, FLOAT eps)
{
    if (_m != _n)
    {
        cerr << "ERROR: Trying to LU decompose a matrix of size (" << _m << "x" << _n << ")" << endl;
        exit(0);
    }

    int32_t i,imax,j,k;
    imax = 0;
    FLOAT   big,dum,sum,temp;
    FLOAT* vv = (FLOAT*)malloc(_n*sizeof(FLOAT)); // vv stores the implicit scaling of each row.
    d = 1.0;

    // Loop over rows to get the implicit scaling information.
    for (i=0; i<_n; i++)
    {
        big = 0.0;
        for (j=0; j<_n; j++)
        {
            if ((temp=fabs(_val[i][j]))>big)
            {
                big = temp;
            }
        }

        // No nonzero largest element.
        if (big == 0.0)
        {
            free(vv);
            return false;
        }
        vv[i] = 1.0/big; // Save the scaling.
    }

    // This is the loop over columns of Crout’s method.
    for (j=0; j<_n; j++)
    {
        // This is equation (2.3.12) except for i = j.
        for (i=0; i<j; i++)
        {
            sum = _val[i][j];
            for (k=0; k<i; k++)
              sum -= _val[i][k]*_val[k][j];
            _val[i][j] = sum;
        }

        big = 0.0; // Initialize the search for largest pivot element.

        for (i=j; i<_n; i++)
        {
            sum = _val[i][j];
            for (k=0; k<j; k++)
            {
                sum -= _val[i][k]*_val[k][j];
            }
            _val[i][j] = sum;
            if ( (dum=vv[i]*fabs(sum))>=big)
            {
                big  = dum;
                imax = i;
            }
        }

        // Do we need to interchange rows?
        if (j!=imax)
        {
            // Yes, do so...
            for (k=0; k<_n; k++)
            {
                dum          = _val[imax][k];
                _val[imax][k] = _val[j][k];
                _val[j][k]    = dum;
            }
            d = -d;     // ...and change the parity of d.
            vv[imax]=vv[j]; // Also interchange the scale factor.
        }

        idx[j] = imax;

        // Now, finally, divide by the pivot element.
        if (j!=_n-1)
        {
            dum = 1.0/_val[j][j];
            for (i=j+1; i<_n; i++)
            {
                _val[i][j] *= dum;
            }
        }
    } // Go back for the next column in the reduction.

    // success
    free(vv);
    return true;
}

//==============================================================================//

// Given a matrix M/A[1.._m][1.._n], this routine computes its singular value decomposition, M/A =
// U·W·V T. Thematrix U replaces a on output. The diagonal matrix of singular values W is output
// as a vector w[1.._n]. Thematrix V (not the transpose V T ) is output as v[1.._n][1.._n].
void Matrix::svd(Matrix &U2,Matrix &W,Matrix &V)
{
    Matrix U = Matrix(*this);
    U2 = Matrix(_m,_m);
    V  = Matrix(_n,_n);

    FLOAT* w   = (FLOAT*)malloc(_n*sizeof(FLOAT));
    FLOAT* rv1 = (FLOAT*)malloc(_n*sizeof(FLOAT));

    int32_t flag,i,its,j,jj,k,l,nm;
    FLOAT   anorm,c,f,g,h,s,scale,x,y,z;

    g = scale = anorm = 0.0; // Householder reduction to bidiagonal form.
    for (i=0;i<_n;i++)
    {
        l = i+1;
        rv1[i] = scale*g;
        g = s = scale = 0.0;
        if (i < _m)
        {
            for (k=i;k<_m;k++) scale += fabs(U._val[k][i]);
            if (scale)
            {
                for (k=i;k<_m;k++)
                {
                    U._val[k][i] /= scale;
                    s += U._val[k][i]*U._val[k][i];
                }
                f = U._val[i][i];
                g = -SIGN(sqrt(s),f);
                h = f*g-s;
                U._val[i][i] = f-g;
                for (j=l;j<_n;j++)
                {
                    for (s=0.0,k=i;k<_m;k++) s += U._val[k][i]*U._val[k][j];
                    f = s/h;
                    for (k=i;k<_m;k++) U._val[k][j] += f*U._val[k][i];
                }
                for (k=i;k<_m;k++) U._val[k][i] *= scale;
            }
        }
        w[i] = scale*g;
        g = s = scale = 0.0;
        if (i<_m && i!=_n-1)
        {
            for (k=l;k<_n;k++) scale += fabs(U._val[i][k]);
            if (scale)
            {
                for (k=l;k<_n;k++)
                {
                    U._val[i][k] /= scale;
                    s += U._val[i][k]*U._val[i][k];
                }
                f = U._val[i][l];
                g = -SIGN(sqrt(s),f);
                h = f*g-s;
                U._val[i][l] = f-g;
                for (k=l;k<_n;k++) rv1[k] = U._val[i][k]/h;
                for (j=l;j<_m;j++)
                {
                    for (s=0.0,k=l;k<_n;k++) s += U._val[j][k]*U._val[i][k];
                    for (k=l;k<_n;k++) U._val[j][k] += s*rv1[k];
                }
                for (k=l;k<_n;k++) U._val[i][k] *= scale;
            }
        }
        anorm = FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }

    // Accumulation of right-hand transformations.
    for (i=_n-1;i>=0;i--)
    {
        if (i<_n-1)
        {
            if (g)
            {
                // Double division to avoid possible underflow.
                for (j=l;j<_n;j++)
                {
                    V._val[j][i]=(U._val[i][j]/U._val[i][l])/g;
                }
                for (j=l;j<_n;j++)
                {
                    for (s=0.0,k=l;k<_n;k++) s += U._val[i][k]*V._val[k][j];
                    for (k=l;k<_n;k++) V._val[k][j] += s*V._val[k][i];
                }
            }
            for (j=l;j<_n;j++) V._val[i][j] = V._val[j][i] = 0.0;
        }
        V._val[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }

    // Accumulation of left-hand transformations.
    for (i=IMIN(_m,_n)-1;i>=0;i--)
    {
        l = i+1;
        g = w[i];
        for (j=l;j<_n;j++) U._val[i][j] = 0.0;
        if (g)
        {
            g = 1.0/g;
            for (j=l;j<_n;j++)
            {
                for (s=0.0,k=l;k<_m;k++) s += U._val[k][i]*U._val[k][j];
                f = (s/U._val[i][i])*g;
                for (k=i;k<_m;k++) U._val[k][j] += f*U._val[k][i];
            }
            for (j=i;j<_m;j++) U._val[j][i] *= g;
        }
        else
        {
            for (j=i;j<_m;j++) U._val[j][i]=0.0;
        }
        ++U._val[i][i];
    }

    // Diagonalization of the bidiagonal form: Loop over singular values,
    for (k=_n-1;k>=0;k--)
    {
        // and over allowed iterations.
        for (its=0;its<30;its++)
        {
            flag = 1;
            // Test for splitting.
            for (l=k;l>=0;l--)
            {
                nm = l-1;
                if ((FLOAT)(fabs(rv1[l])+anorm) == anorm) { flag = 0; break; }
                if ((FLOAT)(fabs( w[nm])+anorm) == anorm) { break; }
            }
            if (flag)
            {
                c = 0.0; // Cancellation of rv1[l], if l > 1.
                s = 1.0;
                for (i=l;i<=k;i++)
                {
                    f = s*rv1[i];
                    rv1[i] = c*rv1[i];
                    if ((FLOAT)(fabs(f)+anorm) == anorm) break;
                    g = w[i];
                    h = pythag(f,g);
                    w[i] = h;
                    h = 1.0/h;
                    c = g*h;
                    s = -f*h;
                    for (j=0;j<_m;j++)
                    {
                        y = U._val[j][nm];
                        z = U._val[j][i];
                        U._val[j][nm] = y*c+z*s;
                        U._val[j][i]  = z*c-y*s;
                    }
                }
            }
            z = w[k];

            // Convergence.
            if (l==k)
            {
                // Singular value is made nonnegative.
                if (z<0.0)
                {
                    w[k] = -z;
                    for (j=0;j<_n;j++) V._val[j][k] = -V._val[j][k];
                }
                break;
            }
            if (its == 29)
            {
                cerr << "ERROR in SVD: No convergence in 30 iterations" << endl;
            }
            x = w[l]; // Shift from bottom 2-by-2 minor.
            nm = k-1;
            y = w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
            g = pythag(f,1.0);
            f = ((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
            c = s = 1.0; // Next QR transformation:
            for (j=l;j<=nm;j++)
            {
                i = j+1;
                g = rv1[i];
                y = w[i];
                h = s*g;
                g = c*g;
                z = pythag(f,h);
                rv1[j] = z;
                c = f/z;
                s = h/z;
                f = x*c+g*s;
                g = g*c-x*s;
                h = y*s;
                y *= c;
                for (jj=0;jj<_n;jj++)
                {
                    x = V._val[jj][j];
                    z = V._val[jj][i];
                    V._val[jj][j] = x*c+z*s;
                    V._val[jj][i] = z*c-x*s;
                }
                z = pythag(f,h);
                w[j] = z; // Rotation can be arbitrary if z = 0.
                if (z)
                {
                    z = 1.0/z;
                    c = f*z;
                    s = h*z;
                }
                f = c*g+s*y;
                x = c*y-s*g;
                for (jj=0;jj<_m;jj++)
                {
                    y = U._val[jj][j];
                    z = U._val[jj][i];
                    U._val[jj][j] = y*c+z*s;
                    U._val[jj][i] = z*c-y*s;
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }

    // sort singular values and corresponding columns of u and v
    // by decreasing magnitude. Also, signs of corresponding columns are
    // flipped so as to maximize the number of positive elements.
    int32_t s2,inc=1;
    FLOAT   sw;
    FLOAT* su = (FLOAT*)malloc(_m*sizeof(FLOAT));
    FLOAT* sv = (FLOAT*)malloc(_n*sizeof(FLOAT));
    do { inc *= 3; inc++; } while (inc <= _n);
    do
    {
        inc /= 3;
        for (i=inc;i<_n;i++)
        {
            sw = w[i];
            for (k=0;k<_m;k++) su[k] = U._val[k][i];
            for (k=0;k<_n;k++) sv[k] = V._val[k][i];
            j = i;
            while (w[j-inc] < sw)
            {
                w[j] = w[j-inc];
                for (k=0;k<_m;k++) U._val[k][j] = U._val[k][j-inc];
                for (k=0;k<_n;k++) V._val[k][j] = V._val[k][j-inc];
                j -= inc;
                if (j < inc) break;
            }
            w[j] = sw;
            for (k=0;k<_m;k++) U._val[k][j] = su[k];
            for (k=0;k<_n;k++) V._val[k][j] = sv[k];
        }
    } while (inc > 1);

    // flip signs
    for (k=0;k<_n;k++)
    {
        s2=0;
        for (i=0;i<_m;i++) if (U._val[i][k] < 0.0) s2++;
        for (j=0;j<_n;j++) if (V._val[j][k] < 0.0) s2++;
        if (s2 > (_m+_n)/2)
        {
            for (i=0;i<_m;i++) U._val[i][k] = -U._val[i][k];
            for (j=0;j<_n;j++) V._val[j][k] = -V._val[j][k];
        }
    }

    // create vector and copy singular values
    W = Matrix(min(_m,_n),1,w);

    // extract mxm submatrix U
    U2.setMat(U.getMat(0,0,_m-1,min(_m-1,_n-1)),0,0);

    // release temporary memory
    free(w);
    free(rv1);
    free(su);
    free(sv);
}

//==============================================================================//

ostream& operator<<(ostream& out,const Matrix& M)
{
    if (M._m==0 || M._n==0)
    {
        out << "[empty matrix]";
    }
    else
    {
        char buffer[1024];
        for (int32_t i=0; i<M._m; i++)
        {
            for (int32_t j=0; j<M._n; j++)
            {
                sprintf(buffer,"%12.7f ",M._val[i][j]);
                out << buffer;
            }
            if (i<M._m-1)
            {
                out << endl;
            }
        }
    }
    return out;
}

//==============================================================================//

void Matrix::allocateMemory(const int32_t m, const int32_t n)
{
    _m = abs(m); _n = abs(n);
    if (_m==0 || _n==0)
    {
        _val = 0;
        return;
    }
    _val    = (FLOAT**)malloc(_m*sizeof(FLOAT*));
    _val[0] = (FLOAT*)calloc(_m*_n,sizeof(FLOAT));
    for(int32_t i=1; i<_m; i++)
    {
        _val[i] = _val[i-1]+_n;
    }
}

//==============================================================================//

void Matrix::releaseMemory()
{
    if (_val!=0)
    {
        free(_val[0]);
        free(_val);
    }
}

//==============================================================================//

FLOAT Matrix::pythag(FLOAT a,FLOAT b)
{
    FLOAT absa,absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb)
    {
        return absa*sqrt(1.0+SQR(absb/absa));
    }
    else
    {
        return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
    }
}
