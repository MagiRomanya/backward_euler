#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/IterativeLinearSolvers>

#include <vector>

class Integrator {
    public:

        Integrator() {};
        ~Integrator() {};

        unsigned int nDoF;
        double h;

        Eigen::VectorXd f0;
        Eigen::VectorXd x;
        Eigen::VectorXd v;
        Eigen::VectorXd delta_v;

        Eigen::SparseMatrix<double> mass;
        Eigen::SparseMatrix<double> df_dx;
        Eigen::SparseMatrix<double> df_dv;

        typedef Eigen::Triplet<double> tri;
        std::vector<tri> df_dx_triplets;
        std::vector<tri> df_dv_triplets;
        std::vector<tri> equation_matrix_triplets;
        std::vector<tri> mass_triplets;

        void resize_containers();

        void resize_containers(unsigned int newDoF);

        void clear_containers();

        void integration_step();

    private:

        void implicit_euler();
};

#endif // INTEGRATOR_H_
