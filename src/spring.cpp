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

Eigen::Matrix3d outer_product(const vec3& v1, const vec3& v2){
  Eigen::Matrix3d result;
  result << v1.x() * v2.x(), v1.x() * v2.y(), v1.x() * v2.z(),
    v1.y() * v2.x(), v1.y() * v2.y(), v1.y() * v2.z(),
    v1.z() * v2.x(), v1.z() * v2.y(), v1.z() * v2.z();
  return result;
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
  // The u · u.transpose() matrix
  Eigen::Matrix3d uut = outer_product(u, u);

  // Calculate the final derivative matrix
  df_dx = - k / L * (df_dx + L0 * uut);
  return df_dx; // 3x3 matrix
}

void Spring::add_derivatives(System &s) const {
  Eigen::Matrix3d df_dx = this->force_derivative(s);
  typedef Eigen::Triplet<double> tri;
  const int p1 = this->i;
  const int p2 = this->j;
  const double h2 = - s.h * s.h;
  if (s.fixed[p1] and s.fixed[p2]) return;
  if (s.fixed[p1]){
    for (int j=0; j < 3; j++){
      for (int k=0; k < 3; k++){
        s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, df_dx(j, k)));
        s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, h2 * df_dx(j, k)));
      }
    }
    return;
  }
  if (s.fixed[p2]){
    for (int j=0; j < 3; j++){
      for (int k=0; k < 3; k++){
      s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, df_dx(j, k)));
      s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, h2 * df_dx(j, k)));
      }
    }
    return;
  }
  for (int j=0; j < 3; j++){
    for (int k=0; k < 3; k++){
      // The df_dx derivative
      s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, df_dx(j, k)));
      s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p2 + k, -df_dx(j, k)));
      s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p1 + k, -df_dx(j, k)));
      s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, df_dx(j, k)));

      // The full equation matrix
      // In this equation we need
      // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;
      s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, h2 * df_dx(j, k)));
      s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p2 + k, -h2 * df_dx(j, k)));
      s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p1 + k, -h2 * df_dx(j, k)));
      s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, h2 * df_dx(j, k)));
    }
  }
}
