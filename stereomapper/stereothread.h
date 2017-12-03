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
    void pushBack(StereoImage::simage &s,Matrix H_total,float gain);
    void pushBack(StereoImage::simage &s,bool subsampling);
    StereoImage::simage* getStereoImage() { return _simg; }
    std::vector< std::vector<View3D::point_3d> > getPoints ()
    {
        std::vector< std::vector<View3D::point_3d> > p;
        if (_points.size()==1)
        {
            p.push_back(_points[0]);
        }
        else if (_points.size()>1)
        {
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
    };

    void  addDisparityMapToReconstruction();
    void  suppressStrongGradients (float* D,const int32_t *dims);

    map3d createCurrentMap();
    void  releaseMap(map3d m);
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
    std::vector<map3d>     _maps3d;
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
