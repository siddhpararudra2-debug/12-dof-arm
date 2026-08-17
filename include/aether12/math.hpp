#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace aether12 {

constexpr std::size_t kDof = 12;
constexpr double kEpsilon = 1e-10;

inline double clamp(double value, double lower, double upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
  Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
  Vec3 operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
  Vec3 operator/(double scale) const { return {x / scale, y / scale, z / scale}; }
  Vec3& operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
  double squaredNorm() const { return x * x + y * y + z * z; }
  double norm() const { return std::sqrt(squaredNorm()); }
  Vec3 normalized() const {
    const double n = norm();
    return n < kEpsilon ? Vec3{} : *this / n;
  }
};

inline Vec3 operator*(double scale, const Vec3& value) { return value * scale; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Matrix3 {
  std::array<std::array<double, 3>, 3> value{};
  static Matrix3 identity() {
    Matrix3 result;
    for (std::size_t i = 0; i < 3; ++i) result.value[i][i] = 1.0;
    return result;
  }
  std::array<double, 3>& operator[](std::size_t row) { return value[row]; }
  const std::array<double, 3>& operator[](std::size_t row) const { return value[row]; }
};

inline Matrix3 operator*(const Matrix3& a, const Matrix3& b) {
  Matrix3 result;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      for (std::size_t k = 0; k < 3; ++k) result[i][j] += a[i][k] * b[k][j];
    }
  }
  return result;
}

inline Vec3 operator*(const Matrix3& matrix, const Vec3& vector) {
  return {
    matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
    matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
    matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z
  };
}

inline Matrix3 transpose(const Matrix3& matrix) {
  Matrix3 result;
  for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = 0; j < 3; ++j) result[i][j] = matrix[j][i];
  return result;
}

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};

  Quaternion normalized() const {
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (n < kEpsilon) return {};
    return {x / n, y / n, z / n, w / n};
  }
};

inline Quaternion quaternionFromRotation(const Matrix3& rotation) {
  Quaternion q;
  const double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (rotation[2][1] - rotation[1][2]) / s;
    q.y = (rotation[0][2] - rotation[2][0]) / s;
    q.z = (rotation[1][0] - rotation[0][1]) / s;
  } else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2]) {
    const double s = std::sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0;
    q.w = (rotation[2][1] - rotation[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (rotation[0][1] + rotation[1][0]) / s;
    q.z = (rotation[0][2] + rotation[2][0]) / s;
  } else if (rotation[1][1] > rotation[2][2]) {
    const double s = std::sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0;
    q.w = (rotation[0][2] - rotation[2][0]) / s;
    q.x = (rotation[0][1] + rotation[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (rotation[1][2] + rotation[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0;
    q.w = (rotation[1][0] - rotation[0][1]) / s;
    q.x = (rotation[0][2] + rotation[2][0]) / s;
    q.y = (rotation[1][2] + rotation[2][1]) / s;
    q.z = 0.25 * s;
  }
  return q.normalized();
}

inline Matrix3 rotationFromQuaternion(const Quaternion& input) {
  const Quaternion q = input.normalized();
  Matrix3 r = Matrix3::identity();
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  r[0][0] = 1.0 - 2.0 * (yy + zz); r[0][1] = 2.0 * (xy - wz); r[0][2] = 2.0 * (xz + wy);
  r[1][0] = 2.0 * (xy + wz); r[1][1] = 1.0 - 2.0 * (xx + zz); r[1][2] = 2.0 * (yz - wx);
  r[2][0] = 2.0 * (xz - wy); r[2][1] = 2.0 * (yz + wx); r[2][2] = 1.0 - 2.0 * (xx + yy);
  return r;
}

inline Quaternion slerp(const Quaternion& first, const Quaternion& second, double t) {
  Quaternion a = first.normalized();
  Quaternion b = second.normalized();
  double cosine = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  if (cosine < 0.0) { b = {-b.x, -b.y, -b.z, -b.w}; cosine = -cosine; }
  if (cosine > 0.9995) {
    return Quaternion{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z), a.w + t * (b.w - a.w)}.normalized();
  }
  const double angle = std::acos(clamp(cosine, -1.0, 1.0));
  const double denominator = std::sin(angle);
  const double first_scale = std::sin((1.0 - t) * angle) / denominator;
  const double second_scale = std::sin(t * angle) / denominator;
  return {first_scale * a.x + second_scale * b.x, first_scale * a.y + second_scale * b.y,
          first_scale * a.z + second_scale * b.z, first_scale * a.w + second_scale * b.w};
}

struct Transform {
  Matrix3 rotation{Matrix3::identity()};
  Vec3 translation{};

  static Transform identity() { return {}; }
};

inline Transform operator*(const Transform& a, const Transform& b) {
  return {a.rotation * b.rotation, a.translation + a.rotation * b.translation};
}

inline Transform inverse(const Transform& transform) {
  const Matrix3 r = transpose(transform.rotation);
  return {r, r * (transform.translation * -1.0)};
}

inline Vec3 rotationError(const Matrix3& current, const Matrix3& target) {
  const Matrix3 relative = target * transpose(current);
  return {0.5 * (relative[2][1] - relative[1][2]), 0.5 * (relative[0][2] - relative[2][0]), 0.5 * (relative[1][0] - relative[0][1])};
}

template <std::size_t Rows, std::size_t Columns>
using Matrix = std::array<std::array<double, Columns>, Rows>;

template <std::size_t Rows, std::size_t Columns>
inline Matrix<Rows, Columns> zeroMatrix() { return {}; }

}  // namespace aether12
