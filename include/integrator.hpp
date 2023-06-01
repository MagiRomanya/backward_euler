#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/IterativeLinearSolvers>

#include <iostream>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "simulable.hpp"
#include "constraint.hpp"

class Integrator {
    typedef Eigen::Triplet<double> tri;

    public:
        Integrator(double h) : h(h) {
            nDoF = 0;
            nConstraints = 0;
        }

        ~Integrator() { }


        Eigen::VectorXd x;
        Eigen::VectorXd v;
        Eigen::VectorXd f0;
        Eigen::VectorXd delta_v;
        Eigen::VectorXd constraint_value;

        void clear_containers();

        void integration_step();

        inline double getTimeStep() const { return h; }
        inline Eigen::SparseMatrix<double> getMassMatrix() const { return mass; }

        inline void add_df_dx_triplet(tri triplet) { df_dx_triplets.push_back(triplet); }
        inline void add_df_dv_triplet(tri triplet) { df_dv_triplets.push_back(triplet); }
        inline void add_equation_triplet(tri triplet) { equation_matrix_triplets.push_back(triplet); }
        inline void add_mass_triplet(tri triplet) { mass_triplets.push_back(triplet); }
        inline void add_constraint_jacobian_triplet(tri triplet) { constraint_jacobian_triplets.push_back(triplet); }

        inline void add_to_df_dp_element(unsigned int i, unsigned int j, double value) { df_dp(i,j) += value; }

        void set_state(Eigen::VectorXd xi, Eigen::VectorXd vi);

        void fill_containers();

        Eigen::SparseMatrix<double> getEquationMatrix();

        Eigen::VectorXd getEquationVector();

        Eigen::VectorXd getForceVector();

        inline Eigen::MatrixXd getParameterJacobian() { return df_dp; }
        inline Eigen::SparseMatrix<double> getForcePositionJacobian() { return df_dx; }
        inline int getDoF() { return nDoF; }

        void reciveDeltaV(Eigen::VectorXd delta_v);

        void add_simulable(Simulable* simulable);

        void add_constraint(Constraint* constraint);

        void clear_simulables();

    private:
        void resize_containers();

        void resize_containers(unsigned int newDoF, unsigned int newNConstraints, unsigned int newNParameters);

        void generate_equation_with_constraints(Eigen::SparseMatrix<double>& equation_matrix, Eigen::VectorXd equation_vector);

        void update_constraint_indices();

        void implicit_euler();

        double h;
        unsigned int nDoF;
        unsigned int nConstraints;
        unsigned int nParameters;

        Eigen::SparseMatrix<double> mass;
        Eigen::SparseMatrix<double> df_dx;
        Eigen::SparseMatrix<double> df_dv;
        Eigen::MatrixXd df_dp;

        std::vector<tri> df_dx_triplets;
        std::vector<tri> df_dv_triplets;
        std::vector<tri> equation_matrix_triplets;
        std::vector<tri> mass_triplets;
        std::vector<tri> constraint_jacobian_triplets;

        std::vector<Simulable*> simulables;

        std::vector<Constraint*> constraints;
};

#endif // INTEGRATOR_H_
