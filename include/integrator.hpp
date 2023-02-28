#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/IterativeLinearSolvers>

#include <vector>

#include "simulable.hpp"

class Simulable;

class Integrator {
    typedef Eigen::Triplet<double> tri;

    public:

        Integrator(double h) : h(h) {
            nDoF = 0;
        }
        ~Integrator() {};


        Eigen::VectorXd x;
        Eigen::VectorXd v;
        Eigen::VectorXd f0;
        Eigen::VectorXd delta_v;

        void resize_containers();

        void resize_containers(unsigned int newDoF);

        void clear_containers();

        void integration_step();

        inline double getTimeStep() const { return h; }

        inline void add_df_dx_triplet(tri triplet) { df_dx_triplets.push_back(triplet); }
        inline void add_df_dv_triplet(tri triplet) { df_dv_triplets.push_back(triplet); }
        inline void add_equation_triplet(tri triplet) { equation_matrix_triplets.push_back(triplet); }
        inline void add_mass_triplet(tri triplet) { mass_triplets.push_back(triplet); }

        void add_simulable(Simulable* simulable);

    private:
        void implicit_euler();

    private:
        double h;
        unsigned int nDoF;

        Eigen::SparseMatrix<double> mass;
        Eigen::SparseMatrix<double> df_dx;
        Eigen::SparseMatrix<double> df_dv;

        std::vector<tri> df_dx_triplets;
        std::vector<tri> df_dv_triplets;
        std::vector<tri> equation_matrix_triplets;
        std::vector<tri> mass_triplets;

        std::vector<Simulable*> simulables;
};

#endif // INTEGRATOR_H_
