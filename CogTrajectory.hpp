#pragma once

#include <fstream>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include "Square.hpp"

class CogTrajectory
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
    if (c != 'I') return false;
    for (int i = 0; i < 4; i++) {
      ifs >> initpos.at(i);
    }
    ifs >> c;
    if (c != 'N') return false;
    ifs >> step_num;

    for (int i = 0; i < step_num; i++) {
      std::array<double, 3> temp;
      ifs >> temp.at(0);
      ifs >> temp.at(1);
      ifs >> temp.at(2);
      foot_steps.push_back(temp);
    }
  }

  std::array<double, 5> solve_trajectory(double x0, double y0, double vx0, double vy0, double zmpx, double zmpy, double zc, double t, double t0)
  {
    double k = std::sqrt(g/zc);
    double C1x = (x0 - zmpx + vx0 / k) / 2.0;
    double C2x = (x0 - zmpx - vx0 / k) / 2.0;
    double C1y = (y0 - zmpy + vy0 / k) / 2.0;
    double C2y = (y0 - zmpy - vy0 / k) / 2.0;

    std::array<double, 5> p;
    for (int i = 0; i < 100; i++) {
      p.at(0) = zmpx + C1x * std::exp(k * t * i / 100.0) + C2x * std::exp(-k * t * i / 100.0);
      p.at(1) = zmpy + C1y * std::exp(k * t * i / 100.0) + C2y * std::exp(-k * t * i / 100.0);
      p.at(2) = k * C1x * std::exp(k * t * i / 100.0) - k * C2x * std::exp(-k * t * i / 100.0);
      p.at(3) = k * C1y * std::exp(k * t * i / 100.0) - k * C2y * std::exp(-k * t * i / 100.0);
      p.at(4) = t0 + t * i / 100.0;

      cog_trajectory.push_back(p);
    }

    return p;
  }
  
  void calc_cog_trajectory(double x0, double y0, double vx0, double vy0)
  {
    cog_trajectory.clear();
    cog_trajectory.shrink_to_fit();

    double x = x0;
    double y = y0;
    double vx = vx0;
    double vy = vy0;
    double t = 0;
    for (int i = 0; i < step_num; i++) {
      std::array<double, 5> p = solve_trajectory(x, y, vx, vy, foot_steps[i].at(0), foot_steps[i].at(1), 1.0, foot_steps[i].at(2), t);
      x = p.at(0);
      y = p.at(1);
      vx = p.at(2);
      vy = p.at(3);
      t = p.at(4);
    }
  }
       
  void calc_cog_trajectory()
  {
    cog_trajectory.clear();
    cog_trajectory.shrink_to_fit();

    double x = initpos.at(0);
    double y = initpos.at(1);
    double vx = initpos.at(2);
    double vy = initpos.at(3);
    double t = 0;
    for (int i = 0; i < step_num; i++) {
      std::array<double, 5> p = solve_trajectory(x, y, vx, vy, foot_steps[i].at(0), foot_steps[i].at(1), 1.0, foot_steps[i].at(2), t);
      x = p.at(0);
      y = p.at(1);
      vx = p.at(2);
      vy = p.at(3);
      t = p.at(4);
    }
  }

  void draw_cog_track(int w, int h)
  {
    glPointSize(2);
    glBegin(GL_POINTS);
    for (std::vector<std::array<double, 5>>::iterator it = cog_trajectory.begin(); it != cog_trajectory.end(); it++) {
      glVertex2d(it->at(0) * 100, h - 100 * it->at(1));
    }
    glEnd();
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

  std::vector<std::array<double, 5>> cog_trajectory;
  double g;
  double m;
  double z;
  double T;
  double Tc;
  double C;
  double S;
  int step_num;
  std::vector<std::array<double, 3>> foot_steps;
  std::array<double, 4> initpos;
};
