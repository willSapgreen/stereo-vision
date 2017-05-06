#ifndef VIEW2D_H
#define VIEW2D_H

#include <QtOpenGL>
#include "../libviso2/src/matcher.h"

class View2D : public QGLWidget
{
    Q_OBJECT

public:

    View2D(QWidget *parent = 0);

    void setImage(unsigned char* data,int width,int height);
    void setColorImage(float* data,int width,int height);
    void setMatches(const std::vector<Matcher::p_match> &m,const std::vector<bool> &i,bool l);
    void clearMatches() { matches.clear(); }

private:

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    // image
    GLuint image_texture;
    int    image_width,image_height;

    // matches
    std::vector<Matcher::p_match> matches;
    std::vector<bool>             inliers;
    bool                          left_image;

signals:

public slots:

};

#endif // VIEW2D_H
