#ifndef BERNSTEIN_HPP
#define BERNSTEIN_HPP

#include <cmath>
#include <numeric>
#include <vector>

struct Point2D {
  double x, y;

  Point2D() : x(0), y(0) {}
  Point2D(double x, double y) : x(x), y(y) {}

  Point2D operator+(const Point2D &p) const {
    return Point2D(x + p.x, y + p.y);
  }
  Point2D operator-(const Point2D &p) const {
    return Point2D(x - p.x, y - p.y);
  }
  Point2D operator*(double scalar) const {
    return Point2D(x * scalar, y * scalar);
  }
  double norm() const { return std::sqrt(x * x + y * y); }
};

// Compute binomial coefficient
inline double binomial_coeff(int n, int k) {
  if (k > n)
    return 0.0;
  if (k == 0 || k == n)
    return 1.0;

  double result = 1.0;
  for (int i = 0; i < k; ++i) {
    result = result * (n - i) / (i + 1);
  }
  return result;
}

// Compute Bernstein polynomial basis function
inline double bernstein_basis(int i, int n, double t) {
  double binom = binomial_coeff(n, i);
  double t_pow = std::pow(t, i);
  double one_minus_t_pow = std::pow(1.0 - t, n - i);
  return binom * t_pow * one_minus_t_pow;
}

// Generate Bezier curve from control points
template <typename Point>
std::vector<Point> bernstein_curve(const std::vector<Point> &control_points,
                                   int n_points = 100) {
  std::vector<Point> curve;
  curve.reserve(n_points);

  int n = control_points.size() - 1;

  for (int step = 0; step < n_points; ++step) {
    double t = static_cast<double>(step) / (n_points - 1);
    Point p;

    for (int i = 0; i <= n; ++i) {
      double basis = bernstein_basis(i, n, t);
      p = p + control_points[i] * basis;
    }

    curve.push_back(p);
  }

  return curve;
}

#endif // BERNSTEIN_HPP
