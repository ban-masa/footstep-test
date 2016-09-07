#pragma once

class Square
{
public:
  
  Square()
  {
    x = 0;
    y = 0;
    theta = 0;
    a = 5;
    b = 3;
  }

  Square(double gx, double gy, double gtheta, double ga, double gb)
  {
    x = gx;
    y = gy;
    theta = gtheta;
    a = ga;
    b = gb;
  }

  void draw(void)
  {
    glPushMatrix();
    glTranslated(x, y, 0);
    glPushMatrix();
    glRotated(-theta, 0, 0, 1);
    glPushMatrix();

    glBegin(GL_LINE_LOOP);
    glVertex2d(a/2.0, b/2.0);
    glVertex2d(-a/2.0, b/2.0);
    glVertex2d(-a/2.0, -b/2.0);
    glVertex2d(a/2.0, -b/2.0);
    glEnd();
    glPointSize(5);
    glBegin(GL_POINTS);
    glVertex2i(0, 0);
    glEnd();

    glPopMatrix();
    glPopMatrix();
    glPopMatrix();
  }

  double x;
  double y;
  double theta;
  double a;
  double b;
};
