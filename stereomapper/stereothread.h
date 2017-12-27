#ifndef STEREOTHREAD_H
#define STEREOTHREAD_H


#include <QThread>

#include "view3d.h"
#include "calibiokitti.h"
#include "stereoimage.h"
#include "planeestimation.h"

#include "../libelas/src/elas.h"

class StereoThread : public QThread
{
    Q_OBJECT

public:

    StereoThread(CalibIOKITTI *calib,View3D *modelView,QObject *parent = 0);
    ~StereoThread();
    void pushBack(StereoImage::simage& s,Matrix H_total,float gain);
    void pushBack(StereoImage::simage& s,bool subsampling);
    StereoImage::simage* getStereoImage() { return _simg; }
    std::vector< std::vector<View3D::point_3d> > getPoints ()
    {
        std::vector< std::vector<View3D::point_3d> > p;
        if(_points.size() == 1)
        {
            p.push_back(_points[0]);
        }
        else if(_points.size() > 1)
        {
            // Get the latest two sets of point_3d.
            p.push_back(_points[_points.size()-2]);
            p.push_back(_points[_points.size()-1]);
        }
        return p;
    }
    void pickedUp() { _picked = true; }
    Matrix getHomographyTotal() { return _H_total; }
    float* getColorDisparityMap() { return _D_color; }
    void clearReconstruction();

protected:

    void run();

private:

    struct map3d
    {
        float*   I;      // image
        float*   D;      // disparity map
        float*   X;      // 3d coordinates
        float*   Y;
        float*   Z;
        Matrix   H;      // extrinsics
        int32_t  width;  // image dimensions
        int32_t  height;
        int32_t  idx;    // index in point list

        map3d()
        {
            I = 0;
            D = 0;
            X = 0;
            Y = 0;
            Z = 0;

            H = Matrix();
            width = 0;
            height = 0;
            idx = 0;
        }

        ~map3d()
        {
            // Release the memory.
            if(I != 0)
            {
                free(I);
                I = 0;
            }
            if(D != 0)
            {
                free(D);
                D = 0;
            }
            if(X != 0)
            {
                free(X);
                X = 0;
            }
            if(Y != 0)
            {
                free(Y);
                Y = 0;
            }
            if(Z != 0)
            {
                free(Z);
                Z = 0;
            }

            H = Matrix();
            width = 0;
            height = 0;
            idx = 0;

        }

        inline map3d& operator=(const map3d& map)
        {
            if(height != map.height || width != map.width)
            {
                // Release the memory.
                if(I != 0)
                {
                    free(I);
                    I = 0;
                }
                if(D != 0)
                {
                    free(D);
                    D = 0;
                }
                if(X != 0)
                {
                    free(X);
                    X = 0;
                }
                if(Y != 0)
                {
                    free(Y);
                    Y = 0;
                }
                if(Z != 0)
                {
                    free(Z);
                    Z = 0;
                }

                // Alloc the memory
                I      = (float*)malloc(map.width * map.height * sizeof(float));
                D      = (float*)malloc(map.width * map.height * sizeof(float));
                X      = (float*)malloc(map.width * map.height * sizeof(float));
                Y      = (float*)malloc(map.width * map.height * sizeof(float));
                Z      = (float*)malloc(map.width * map.height * sizeof(float));
            }

            // Copy
            memcpy(I, map.I ,map.width * map.height * sizeof(float));
            memcpy(D, map.D ,map.width * map.height * sizeof(float));
            memcpy(X, map.X ,map.width * map.height * sizeof(float));
            memcpy(Y, map.Y ,map.width * map.height * sizeof(float));
            memcpy(Z, map.Z ,map.width * map.height * sizeof(float));
            H = map.H;
            width = map.width;
            height = map.height;
            idx = map.idx;

            return *this;
        }
    };

    void  addDisparityMapToReconstruction();
    void  suppressStrongGradients (float* D,const int32_t *dims);

    map3d createCurrentMap();
    void  releaseMap(map3d& m);
    void  getIntrinsics();

    CalibIOKITTI*          _calib;
    View3D*                _modelView;
    PlaneEstimation*       _plane;
    StereoImage::simage*   _simg;
    float*                 _D_color;
    Matrix                 _H_total;
    Matrix                 _H_init;
    float                  _gain;
    std::vector< std::vector<View3D::point_3d> > _points;

    // To fix the memory leak issue ( because memory used in map3d is not released until clearReconstruction is called.
    // The memory alloc failure will occur if the sytem is with limited memory and runs on the big data set.
    // In the current algorithm design, only the previous maps3d is used.
    // TODO: use circular buffer to hold a series of maps3d.
    //std::vector<map3d>     _maps3d;
    map3d _previous_map3d;
    bool                   _plane_estimated;
    bool                   _picked;
    float                  _f;
    float                  _cu;
    float                  _cv;
    float                  _base;
    float                  _max_dist;
    Matrix                 _K;
    int                    _mode;
    bool                   _subsampling;

signals:

    void newDisparityMapArrived();

public slots:

};

#endif // STEREOTHREAD_H
