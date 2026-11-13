#pragma once
// Pure functions shared by the plugins and the unit tests (no ROS types, so they test without a running graph).
#include <algorithm>
#include <cmath>
#include <vector>

namespace nav_robot_cpp
{

inline double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// Number of finite beams in a scan.
inline int countFinite(const std::vector<float> & ranges)
{
  return static_cast<int>(std::count_if(ranges.begin(), ranges.end(), [](float r) {return std::isfinite(r);}));
}

// Points spaced about `step` apart on the segment (x0,y0)->(x1,y1), both ends included.
inline std::vector<std::pair<double, double>> interpolate(double x0, double y0, double x1, double y1, double step)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    pts.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
  }
  return pts;
}

// Pure pursuit: given the goal point in the robot frame, return curvature (1/m).
inline double pursuitCurvature(double lx, double ly)
{
  const double d2 = lx * lx + ly * ly;
  return d2 < 1e-9 ? 0.0 : 2.0 * ly / d2;
}

}  // namespace nav_robot_cpp
