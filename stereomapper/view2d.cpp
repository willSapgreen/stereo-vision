#include <iostream>
#include "view2d.h"

using namespace std;

const int MAX_WIDTH  = 2048;
const int MAX_HEIGHT = 2048;

View2D::View2D(QWidget *parent) :
    QGLWidget(QGLFormat(QGL::SampleBuffers),parent) {
  image_width  = 1;
  image_height = 1;
}

void View2D::setImage(unsigned char* data,int width,int height) {
  makeCurrent();
  image_width  = max(width,1);
  image_height = max(height,1);
  glTexImage2D(GL_TEXTURE_2D,0,1,image_width,image_height,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,data);
  clearMatches();
  doneCurrent();
  updateGL();
}

void View2D::setColorImage(float* data,int width,int height) {
  makeCurrent();
  image_width  = max(width,1);
  image_height = max(height,1);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,image_width,image_height,0,GL_RGB,GL_FLOAT,data);
  doneCurrent();
  updateGL();
}

void View2D::setMatches(const std::vector<Matcher::p_match> &m,const std::vector<bool> &i,bool l) {
  matches    = m;
  inliers    = i;
  left_image = l;
  updateGL();
}

void View2D::initializeGL() {
  makeCurrent();
  glGenTextures(1,&image_texture);
  glBindTexture(GL_TEXTURE_2D,image_texture);
  //glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  doneCurrent();
}

void View2D::paintGL()
{
  makeCurrent();

  // clear screen & set matrices
  glMatrixMode(GL_PROJECTION);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glOrtho(0.0f,1.0f,1.0f,0.0f,-1.0f,1.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glColor3f(1.0,1.0,1.0);

  // paint image as textured quad
  if (image_width>1 && image_height>1) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,image_texture);
    glBegin(GL_QUADS);
    glTexCoord2f(1,1); glVertex3f(1,1,0);
    glTexCoord2f(0,1); glVertex3f(0,1,0);
    glTexCoord2f(0,0); glVertex3f(0,0,0);
    glTexCoord2f(1,0); glVertex3f(1,0,0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
  }

  // paint matches
  glDisable(GL_DEPTH_TEST);
  glPointSize(5);
  glLineWidth(2);

  // left image
  if (left_image) {
    for (int32_t i=0; i<(int32_t)matches.size(); i++) {
      float col = max(min(matches[i].u1p-matches[i].u2p,(float)100.0),(float)0.0)/100.0;
      if (inliers[i]) glColor3f(col,1-col,0);
      else            glColor3f(0,0,1);
      glBegin(GL_LINES);
        glVertex3f(matches[i].u1p/(float)image_width,matches[i].v1p/(float)image_height,0);
        glVertex3f(matches[i].u1c/(float)image_width,matches[i].v1c/(float)image_height,0);
      glEnd();
      glBegin(GL_POINTS);
        glVertex3f(matches[i].u1c/(float)image_width,matches[i].v1c/(float)image_height,0);
      glEnd();
    }

  // right image
  } else {
    for (int32_t i=0; i<(int32_t)matches.size(); i++) {
      float col = max(min(matches[i].u1p-matches[i].u2p,(float)100.0),(float)0.0)/100.0;
      if (inliers[i]) glColor3f(col,1-col,0);
      else            glColor3f(0,0,1);
      glBegin(GL_LINES);
        glVertex3f(matches[i].u2p/(float)image_width,matches[i].v2p/(float)image_height,0);
        glVertex3f(matches[i].u2c/(float)image_width,matches[i].v2c/(float)image_height,0);
      glEnd();
      glBegin(GL_POINTS);
        glVertex3f(matches[i].u2c/(float)image_width,matches[i].v2c/(float)image_height,0);
      glEnd();
    }
  }

  glEnable(GL_DEPTH_TEST);
  doneCurrent();
}

void View2D::resizeGL(int width, int height) {
  makeCurrent();
  glViewport(0,0,width,height);
  doneCurrent();
}
