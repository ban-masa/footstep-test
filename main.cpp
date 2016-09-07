#include "lib.h"

ZMPViewer zv;

void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT);
  zv.calc_footsteps(10, 1, 0.0, 0.05);
  glColor3d(0.0, 0.0, 0.0);
  zv.draw_cog_track(WIDTH, HEIGHT);
  glColor3d(1.0, 0.0, 0.0);
  zv.draw_fs(WIDTH, HEIGHT);
  glColor3d(0.0, 0.0, 0.0);
  zv.draw_fixed_fs(WIDTH, HEIGHT);
  glFlush();
}

void resize(int w, int h)
{
  glViewport(0, 0, w, h);
  glLoadIdentity();
  glOrtho(-0.5, (GLdouble)w - 0.5, (GLdouble)h - 0.5, -0.5, -1.0, 1.0);
}

void mouse(int button, int state, int x, int y)
{
}

void keyboard(unsigned char key, int x, int y)
{
  /*
  Square sq1(100, 100, 10, 50, 30);
  Square sq2(0, 0, 0, 50, 30);
  glColor3d(0.0, 0.0, 0.0);
  sq1.draw();
  glColor3d(1.0, 0.0, 0.0);
  sq2.draw();
  glFlush();
  */
}

void init(void)
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
}

int main(int argc, char *argv[])
{
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA);
  glutCreateWindow(argv[0]);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutMouseFunc(mouse);
  glutKeyboardFunc(keyboard);
  init();
  if (!zv.read(argv[1])) {
    std::cerr << "Error" << std::endl;
    exit(1);
  }
  zv.show_param();
  glutMainLoop();
  return 0;
}
