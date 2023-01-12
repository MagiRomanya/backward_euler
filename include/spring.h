#ifndef SPRING_H
#define SPRING_H

#include "system.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math.h>
#include <ostream>
#include <vector>

// TODO: Add the spring force as a method of a class Spring
// TODO: Convert the new_spring function to the constructor of the same class
class Spring {
  public:
    Spring() {}
    Spring(unsigned int pi, unsigned int pj, double sk, double sL0){
      i = pi;
      j = pj;
      k = sk;
      L0 = sL0;
    }
    // The two particles bound by a spring
    unsigned int i, j;
    // The spring stiffness & the spring rest length
    double k, L0;

    vec3 force(const System &s) const ;
    Eigen::Matrix3d force_derivative(const System &s) const ;
    void add_derivatives(System &s) const ;
};

#endif
