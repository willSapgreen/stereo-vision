#include "visualizethread.h"

VisualizeThread::VisualizeThread(View2D *disparityView,View3D *modelView,QObject *parent) :
    QThread(parent),
    _disparityView(disparityView),
    _modelView(modelView)
{
}

//==============================================================================//

void VisualizeThread::run()
{
    if (_p.size()>0)
    {
        //makeCurrent();
        _modelView->addPoints(_p);
        //doneCurrent();
        _p.clear();
    }
}
