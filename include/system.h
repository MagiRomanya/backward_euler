#ifndef SYSTEM_H
#define SYSTEM_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/IterativeLinearSolvers>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>
#include <eigen3/Eigen/src/SparseCore/SparseMatrix.h>
#include <eigen3/Eigen/src/SparseCore/SparseMatrixBase.h>
#include <iostream>
#include <string>
#include <vector>

#include "vec3.h"
#include "interaction.h"
#include "mesh.h"
#include "object.h"
#include "spring.h"

class System {
    public:
        System(){};
        ~System();

        // particle number
        int num;

        // System positions and velocities
        Eigen::VectorXd x;
        Eigen::VectorXd v;

        // Mass matrix
        Eigen::SparseMatrix<double> Mass_s;

        // Forces and their derivatives
        Eigen::VectorXd f0;
        Eigen::SparseMatrix<double> df_dx_s;
        Eigen::SparseMatrix<double> df_dv_s;

        typedef Eigen::Triplet<double> tri;
        std::vector<tri> df_dx_triplets;
        std::vector<tri> df_dv_triplets;
        std::vector<tri> equation_matrix_triplets;

        // Fixed particles
        std::vector<bool> fixed; // whether a particle is fixed or not (size = # of particles)
        std::vector<int> fixed_particles; // list of fixed particles (size = # of fixed particles)

        // Velocity difference
        Eigen::VectorXd delta_v;

        // List of interactions of the system
        std::vector<Interaction*> interactions;

        double h = 1.0; // Default integration step
        double energy = 0.0;

        SimpleMesh mesh;

        inline vec3 particle_position(int i) const {
            // Returns the i-th particle position
            return vec3(x(i*3), x(i*3 +1), x(i*3 +2));
        }
        inline vec3 particle_velocity(int i) const {
            // Returns the i-th particle velocity
            return vec3(v(i*3), v(i*3 +1), v(i*3 +2));
        }

        inline vec3 particle_force(int i) const {
            // Returns the i-th particle force
            return vec3(f0(i * 3), f0(i * 3 + 1), f0(i * 3 + 2));
        }

        inline void fix_particle(int p){
            // Make a particle fixed: this particle is not affected by any interaction and will not move
            fixed[p] = true;
            fixed_particles.push_back(p);
        }

        void load_from_mesh(const SimpleMesh &mesh, double k_spring);

        void update_mesh();

        void update_dimensions(int new_particle_number);

        void update_vel_and_pos();

        void backward_euler_sparse();

        void begin_equation_matrix();

        void calculate_kinetic_energy();

        void update();
};

#endif
