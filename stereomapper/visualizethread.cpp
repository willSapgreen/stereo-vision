#include "visualizethread.h"

VisualizeThread::VisualizeThread(View2D *disparityView,View3D *modelView,QObject *parent) :
    QThread(parent),
    disparityView(disparityView),
    modelView(modelView) {
}


void VisualizeThread::run () {
  if (p.size()>0) {
    //makeCurrent();
    modelView->addPoints(p);
    //doneCurrent();
    p.clear();
  }
}
