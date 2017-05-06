#ifndef STEREOTHREAD_H
#define STEREOTHREAD_H


#include <QThread>

#include "view3d.h"
#include "calibio.h"
#include "stereoimage.h"
#include "planeestimation.h"

#include "../libelas/src/elas.h"

class StereoThread : public QThread {
  Q_OBJECT

public:

  StereoThread(CalibIO *calib,View3D *modelView,QObject *parent = 0);
  ~StereoThread();
  void pushBack(StereoImage::simage &s,Matrix H_total,float gain_);
  void pushBack(StereoImage::simage &s,bool subsampling_);
  StereoImage::simage* getStereoImage() { return simg; }
  std::vector< std::vector<View3D::point_3d> > getPoints () {
    std::vector< std::vector<View3D::point_3d> > p;
    if (points.size()==1) {
      p.push_back(points[0]);
    } else if (points.size()>1) {
      p.push_back(points[points.size()-2]);
      p.push_back(points[points.size()-1]);
    }
    return p;
  }
  void pickedUp() { picked = true; }
  Matrix getHomographyTotal() { return H_total; }
  float* getColorDisparityMap() { return D_color; }
  void clearReconstruction();

protected:

  void run();

private:

  struct map3d {
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

  CalibIO               *calib;
  View3D                *modelView;
  PlaneEstimation       *plane;
  StereoImage::simage   *simg;
  float                 *D_color;
  Matrix                 H_total,H_init;
  float                  gain;
  std::vector< std::vector<View3D::point_3d> > points;
  std::vector<map3d>     maps3d;
  bool                   plane_estimated;

  bool                   picked;
  float                  f,cu,cv,base;
  float                  max_dist;
  Matrix                 K;
  int                    mode;
  bool                   subsampling;

signals:

  void newDisparityMapArrived();

public slots:

};

#endif // STEREOTHREAD_H
