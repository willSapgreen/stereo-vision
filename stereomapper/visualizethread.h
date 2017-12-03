#ifndef VISUALIZETHREAD_H
#define VISUALIZETHREAD_H

#include <QThread>
#include <QGLWidget>
#include "view2d.h"
#include "view3d.h"

class VisualizeThread : public QThread {

  Q_OBJECT

public:

  VisualizeThread(View2D *disparityView,View3D *modelView,QObject *parent = 0);
  void addPoints (std::vector< std::vector<View3D::point_3d> > p) { _p = p; }

protected:

  void run();

private:

  View2D* _disparityView;
  View3D* _modelView;
  std::vector< std::vector<View3D::point_3d> > _p;

signals:

public slots:

};

#endif // VISUALIZETHREAD_H
