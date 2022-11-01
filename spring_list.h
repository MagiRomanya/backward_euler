#ifndef SPRING_LIST_H
#define SPRING_LIST_H

#include "spring.h"

class Spring_list {
  public:
    // All the springs of the simulation
    std::vector<Spring> springs;

    // Add a spring to the simulation
    inline void add_spring(unsigned int i, unsigned int j, double k, double L0) {
      springs.push_back(Spring(i, j, k, L0));
    }

    // TODO: Push the forces as vec3 and not by coordinates.
    void add_spring_forces(System &s) const;

    void spring_derivatives_finite(System &s) const;

    // TODO: For the sparse matrix: Replace the .coeffReff by pushing the elements by triplets.
    // TODO: For the dense matrix: Push the 3x3 matrix as a whole (not by coordinates)
    void add_spring_derivatives(System &s) const;
};


#endif // SPRING_LIST_H
