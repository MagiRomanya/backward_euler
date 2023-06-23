#ifndef PYSIMULATION_H_
#define PYSIMULATION_H_

#include <vector>
#include <memory>

#include "integrator.hpp"
#include "mass_spring.hpp"
#include "object_manager.hpp"
#include "renderer.h"

/* Simulation class exposed to python for the client to do the neccesary
 * computations. */

#define NODE_MASS 1
#define TimeStep 0.1f

// Grid Dimensions
#define N 20
#define M 20

class PySimulation {
    public:
        PySimulation(double k, double k_bend, bool graphics=false);

        PySimulation(std::vector<double> k, std::vector<double> k_bend, bool graphics=false);

        ~PySimulation();

        void fill_containers() {integrator->fill_containers();}

        void recieve_deltaV(Eigen::VectorXd delta_v) {integrator->reciveDeltaV(delta_v);}

        void set_state(Eigen::VectorXd xi, Eigen::VectorXd vi) {integrator->set_state(xi,vi);}

        Eigen::SparseMatrix<double> getEquationMatrix() {return integrator->getEquationMatrix();}

        Eigen::VectorXd getEquationVector() {return integrator->getEquationVector();}

        Eigen::VectorXd getForce() {return integrator->getForceVector();}

        Eigen::VectorXd getPosition() {return integrator->x;}

        Eigen::VectorXd getVelocity() {return integrator->v;}

        std::vector<double> getDiffParameteres() {return integrator->diff_manager.get_parameters();}

        Eigen::SparseMatrix<double> getMassMatrix() {return integrator->getMassMatrix();}

        Eigen::MatrixXd getParameterJacobian() {return integrator->getParameterJacobian();}

        Eigen::SparseMatrix<double> getForcePositionJacobian() {return integrator->getForcePositionJacobian();}

        int getDoF() {return integrator->getDoF();}

        double getTimeStep() {return integrator->getTimeStep();}
        // Eigen::SparseMatrix<double> getForceVelocityJacobian() {return integrator->getForceVelocityJacobian();}

        void render_state();

    private:
        void setUpCloth();

        SimpleMesh mesh;
        Object cloth;
        std::unique_ptr<Integrator> integrator;
        std::unique_ptr<MassSpring> mass_spring;

        // Weather or not to display the simulation graphically
        bool graphical;
        std::unique_ptr<Renderer> renderer;
        ObjectManager omanager;

};

#endif // PYSIMULATION_H_
