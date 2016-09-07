#pragma once

#include <fstream>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include "Square.hpp"

class ZMPViewer
{
public:

  bool read(char* fname)
  {
    std::ifstream ifs(fname);
    if (ifs.fail()) {
      std::cerr << "Failed to read file" << std::endl;
      return false;
    }
    char c;
    double val;
    ifs >> c;
    if (c != 'G') return false;
    ifs >> g;
    ifs >> c;
    if (c != 'M') return false;
    ifs >> m;
    ifs >> c;
    if (c != 'Z') return false;
    ifs >> z;
    ifs >> c;
    if (c != 'T') return false;
    ifs >> T;
    ifs >> c;
    if (c != 'N') return false;
    ifs >> step_num;

    for (int i = 0; i < step_num; i++) {
      std::array<double, 2> temp;
      ifs >> temp.at(0);
      ifs >> temp.at(1);
      foot_steps.push_back(temp);
      if (i == step_num - 1) {
        foot_steps.push_back(temp);
      }
    }
    Tc = std::sqrt(z/g);
    C = std::cosh(T/Tc);
    S = std::sinh(T/Tc);
    return true;
  }

  void show_param(void)
  {
    std::cout << "m:" << m << std::endl;
    std::cout << "z:" << z << std::endl;
    std::cout << "T:" << T << std::endl;
    std::cout << "step_num:" << step_num << std::endl;
    for (int i = 0; i < step_num; i++) {
      std::cout << i << ":" << foot_steps[i].at(0) << " " << foot_steps[i].at(1) << std::endl;
    }
  }

  double pos_calc(double t, double x, double vx, double px)
  {
    return (x - px) * std::cosh(t/Tc) + Tc * vx * std::sinh(t/Tc) + px;
  }

  double vel_calc(double t, double x, double vx, double px)
  {
    return (x - px) * std::sinh(t/Tc) / Tc + vx * cosh(t/Tc);
  }

  double end_velx(double x)
  {
    return x * (C + 1.0) / (Tc * S);
  }

  double end_vely(double y)
  {
    return y * (C - 1.0) / (Tc * S);
  }

  void calc_footsteps(double a, double b, double vcx0, double vcy0)
  {
    init_vcx = vcx0;
    init_vcy = vcy0;

    double t = 0;
    int n = 0;
    double px = foot_steps[0].at(0);
    double py = foot_steps[0].at(1);
    double cx = foot_steps[0].at(0);
    double cy = foot_steps[0].at(1);
    double vcx = vcx0;
    double vcy = vcy0;
    fixed_foot_steps.push_back(foot_steps[0]);

    for (int i = 1; i < step_num; i++) {
      double tempcx = pos_calc(T, cx, vcx, px);
      double tempcy = pos_calc(T, cy, vcy, py);
      double tempvcx = vel_calc(T, cx, vcx, px);
      double tempvcy = vel_calc(T, cy, vcy, py);
      cx = tempcx;
      cy = tempcy;
      vcx = tempvcx;
      vcy = tempvcy;

      t += T;
      px = foot_steps[i].at(0);
      py = foot_steps[i].at(1);
      double ppx = foot_steps[i + 1].at(0);
      double ppy = foot_steps[i + 1].at(1);

      double bx = (ppx - px)/2;
      double by = (ppy - py)/2;

      double xd = px + bx;
      double yd = py + by;
      double vxd = end_velx(bx);
      double vyd = end_vely(by);
      
      double D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

      double fixed_px = -a * (C - 1) * (xd - C * cx - Tc * S * vcx) / D
                        - b * S * (vxd - S * cx / Tc - C * vcx) / (Tc * D);
      double fixed_py = -a * (C - 1) * (yd - C * cy - Tc * S * vcy) / D
                        - b * S * (vyd - S * cy / Tc - C * vcy) / (Tc * D);
      
      std::array<double, 2> fixed_fs;
      fixed_fs.at(0) = fixed_px;
      fixed_fs.at(1) = fixed_py;
      fixed_foot_steps.push_back(fixed_fs);
      px = fixed_px;
      py = fixed_py;
    }
  }

  void draw_cog_track(int w, int h)
  {
    int step_val = 100;
    double cx = fixed_foot_steps[0].at(0);
    double cy = fixed_foot_steps[0].at(1);
    double vcx = init_vcx;
    double vcy = init_vcy;

    for (int i = 0; i < step_num; i++) {
      double t = 0.0;
      double px = fixed_foot_steps[i].at(0);
      double py = fixed_foot_steps[i].at(1);

      for (int j = 0; j < step_val; j++) {
        t += T/step_val;
        double tcx = pos_calc(t, cx, vcx, px);
        double tcy = pos_calc(t, cy, vcy, py);

        tcx *= 100;
        tcy = h - tcy * 100;

        if (0<= tcx && tcx <= w && 0 <= tcy && tcy <= h) {
          glPointSize(2);
          glBegin(GL_POINTS);
          glVertex2d(tcx, tcy);
          glEnd();
        }
      }
      double tempcx = pos_calc(T, cx, vcx, px);
      double tempcy = pos_calc(T, cy, vcy, py);
      double tempvcx = vel_calc(T, cx, vcx, px);
      double tempvcy = vel_calc(T, cy, vcy, py);
      cx = tempcx;
      cy = tempcy;
      vcx = tempvcx;
      vcy = tempvcy;
    }
  }

  void draw_fs(int w, int h)
  {
    for (int i = 0; i < step_num; i++) {
      double tx = foot_steps[i].at(0);
      double ty = foot_steps[i].at(1);
      tx = tx * 100;
      ty = h - ty * 100;
      if (0 <= tx && tx <= w && 0 <= ty && ty <= h) {
        Square sq(tx, ty, 0.0, 30, 20);
        sq.draw();
      }
    }
  }

  void draw_fixed_fs(int w, int h)
  {
    for (int i = 0; i < step_num; i++) {
      double tx = fixed_foot_steps[i].at(0);
      double ty = fixed_foot_steps[i].at(1);
      tx = tx * 100;
      ty = h - ty * 100;
      if (0 <= tx && tx <= w && 0 <= ty && ty <= h) {
        Square sq(tx, ty, 0.0, 30, 20);
        sq.draw();
      }
    }
  }

  void show_fs(void)
  {
    std::cout << "fs" << std::endl;
    for (int i = 0; i < step_num; i++) {
      std::cout << i << ":" << foot_steps[i].at(0) << " " << foot_steps[i].at(1) << std::endl;
    }
    std::cout << std::endl;
  }

  void show_fixed_fs(void)
  {
    std::cout << "fixed_fs" << std::endl;
    for (int i = 0; i < step_num; i++) {
      std::cout << i << ":" << fixed_foot_steps[i].at(0) << " " << fixed_foot_steps[i].at(1) << std::endl;
    }
    std::cout << std::endl;
  }

  double g;
  double m;
  double z;
  double T;
  double Tc;
  double C;
  double S;
  int step_num;
  std::vector<std::array<double, 2>> foot_steps;
  std::vector<std::array<double, 2>> fixed_foot_steps;
  double init_vcx;
  double init_vcy;
};
