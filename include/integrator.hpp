#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/IterativeLinearSolvers>

#include <vector>

#include "simulable.hpp"
#include "constraint.hpp"

class Integrator {
    typedef Eigen::Triplet<double> tri;

    public:

        Integrator(double h) : h(h) {
            nDoF = 0;
            nConstraints = 0;
        }
        ~Integrator() {};


        Eigen::VectorXd x;
        Eigen::VectorXd v;
        Eigen::VectorXd f0;
        Eigen::VectorXd delta_v;
        Eigen::VectorXd constraint_value;

        void clear_containers();

        void integration_step();

        inline double getTimeStep() const { return h; }

        inline void add_df_dx_triplet(tri triplet) { df_dx_triplets.push_back(triplet); }
        inline void add_df_dv_triplet(tri triplet) { df_dv_triplets.push_back(triplet); }
        inline void add_equation_triplet(tri triplet) { equation_matrix_triplets.push_back(triplet); }
        inline void add_mass_triplet(tri triplet) { mass_triplets.push_back(triplet); }
        inline void add_constraint_jacobian_triplet(tri triplet) { constraint_jacobian_triplets.push_back(triplet); }

        void fill_containers();

        Eigen::SparseMatrix<double> getEquationMatrix();

        Eigen::VectorXd getEquationVector();

        void reciveDeltaV(Eigen::VectorXd delta_v);

        void add_simulable(Simulable* simulable);

        void add_constraint(Constraint* constraint);

    private:
        void resize_containers();

        void resize_containers(unsigned int newDoF, unsigned int newNConstraints);

        void generate_equation_with_constraints(Eigen::SparseMatrix<double>& equation_matrix, Eigen::VectorXd equation_vector);

        void update_constraint_indices();

        void implicit_euler();

        double h;
        unsigned int nDoF;
        unsigned int nConstraints;

        Eigen::SparseMatrix<double> mass;
        Eigen::SparseMatrix<double> df_dx;
        Eigen::SparseMatrix<double> df_dv;

        std::vector<tri> df_dx_triplets;
        std::vector<tri> df_dv_triplets;
        std::vector<tri> equation_matrix_triplets;
        std::vector<tri> mass_triplets;
        std::vector<tri> constraint_jacobian_triplets;

        std::vector<Simulable*> simulables;

        std::vector<Constraint*> constraints;
};

#endif // INTEGRATOR_H_
