#ifndef PYSIMULATION_H_
#define PYSIMULATION_H_

#include <vector>
#include <memory>

#include "integrator.hpp"
#include "mass_spring.hpp"
#include "mesh.h"
#include "object_manager.hpp"
#include "renderer.h"
#include "contact.hpp"

/* Simulation class exposed to python for the client to do the neccesary
 * computations. */

#define NODE_MASS 1
// #define TimeStep 0.2f
#define TimeStep 0.2f

#define ENABLE_CONTACT

// Grid Dimensions
#define N 10
#define M 10

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

        inline int getDoF() {return integrator->getDoF();}

        inline double getTimeStep() {return integrator->getTimeStep();}
        // Eigen::SparseMatrix<double> getForceVelocityJacobian() {return integrator->getForceVelocityJacobian();}

        inline SimpleMesh getMesh() { return mesh; }

        std::vector<unsigned int> getSpringNodeIndices();
        std::vector<unsigned int> getBendSpringNodeIndices();

        inline std::vector<unsigned int> getGridDimensions() { return {N, M}; }

        void render_state();

    private:
        void setUpCloth();

        SimpleMesh mesh;
        Object* cloth;
        std::unique_ptr<Integrator> integrator;
        std::unique_ptr<MassSpring> mass_spring;

        // Boundary information

        // Weather or not to display the simulation graphically
        bool graphical;
        std::unique_ptr<Renderer> renderer;
        std::unique_ptr<Contact> contact;
        ObjectManager omanager;

};

#endif // PYSIMULATION_H_
