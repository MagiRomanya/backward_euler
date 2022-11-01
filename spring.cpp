#include "spring.h"

vec3 Spring::force(const System &s) const {
  // Computes the spring force
  const vec3 &x1 = s.particle_position(i);
  const vec3 &x2 = s.particle_position(j);
  double L = (x1 - x2).length();
  double f = -k * (1.0 - L0 / L);
  vec3 force = f * (x1 - x2);
  return force;
}

Eigen::Matrix3d Spring::force_derivative(const System &s) const {
  // Calculates the derivative of the force of a string with respect to position
  // df/dx
  const vec3 &x1 = s.particle_position(i);
  const vec3 &x2 = s.particle_position(j);
  // Distance between particles
  double L = (x1 -x2).length();
  // u is the normalized vector between particles 1 and 2
  vec3 u = (x1 - x2)/L;
  // Initialize the derivative matrix
  Eigen::Matrix3d df_dx = (L - L0) * Eigen::Matrix3d::Identity();
  // The u · u^T matrix
  // TODO: Use the built in Eigen functionality to multiply u · u^T (external product)
  Eigen::Matrix3d uut;
  uut << u.x() * u.x(), u.x() * u.y(), u.x() * u.z(),
    u.y() * u.x(), u.y() * u.y(), u.y() * u.z(),
    u.z() * u.x(), u.z() * u.y(), u.z() * u.z();
  // Calculate the final derivative matrix
  df_dx = - k / L * (df_dx + L0 * uut);
  return df_dx; // 3x3 matrix
}

