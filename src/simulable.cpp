#include "simulable.hpp"
#include "integrator.hpp"

void Simulable::get_initial_state_jacobian(Eigen::SparseMatrix<double>& dx0dp, Eigen::SparseMatrix<double>& dv0dp) {
    // Set the matrix to zero
    dx0dp.resize(nDoF, integrator->diff_manager.get_size());
    dv0dp.resize(nDoF, integrator->diff_manager.get_size());
};
