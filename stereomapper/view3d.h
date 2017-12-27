#ifndef VIEW3D_H
#define VIEW3D_H

#include <GL/glu.h>
#include <QGLWidget>
#include <QMouseEvent>
#include "../libviso2/src/matrix.h"

class View3D : public QGLWidget
{
    Q_OBJECT

public:

    struct point_3d
    {
        float x,y,z;
        float val;
        point_3d (float x,float y,float z,float val) : x(x),y(y),z(z),val(val) {}
    };

    View3D(QWidget *parent = 0);
    ~View3D();
    void addCamera(Matrix H_total, float s, bool keyframe);
    void addPoints(std::vector<std::vector<point_3d>> p);
    void clearAll ()
    {
        _cams.clear();
        for (int32_t i=0; i<(int32_t)_gl_list.size(); i++)
        {
            glDeleteLists(_gl_list[i],1);
        }
        _gl_list.clear();
    }
    void setBackgroundWallFlag(bool bg_wall_flag_) {_bg_wall_flag = bg_wall_flag_; updateGL(); }
    void setBackgroundWallPosition(float bg_wall_pos_) {_bg_wall_pos = bg_wall_pos_; updateGL(); }
    void setShowCamerasFlag(bool show_cam_flag_) { _show_cam_flag = show_cam_flag_; updateGL(); }
    void setGridFlag(bool show_grid_flag_) { _show_grid_flag = show_grid_flag_; updateGL(); }
    void setWhiteFlag(bool show_white_flag_) { _show_white_flag = show_white_flag_; updateGL(); }
    void addPose() { _poses.push_back(_pose_curr); std::cout << "Poses: " << _poses.size() << std::endl; }
    void delPose() { if (_poses.size()>0) _poses.pop_back(); std::cout << "Poses: " << _poses.size() << std::endl; }
    void playPoses(bool record);
    void recordHuman();

protected:

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:

    std::string createNewRecordDirectory();

    struct cam
    {
        float p[10][3];
        bool  keyframe;
    };

    struct pose
    {
        float  zoom;
        float  rotx,roty;
        float  tx,ty,tz;
    };

    std::vector<cam> _cams;
    std::vector<pose> _poses;
    pose   _pose_curr;

    QPoint _last_pos;
    QColor _green,purple;
    std::vector<GLuint> _gl_list;
    bool   _bg_wall_flag;
    float  _bg_wall_pos;
    bool   _show_cam_flag;
    bool   _show_grid_flag;
    bool   _show_white_flag;

signals:

};

#endif // VIEW3D_H
