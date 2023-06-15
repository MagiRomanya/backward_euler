#include <pybind11/eigen.h> // for sparse and dense matrices
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for std::vector as argument

#include "Eigen/src/Core/Matrix.h"
#include "clock.h"
#include "contact.h"
#include "integrator.hpp"
#include "mass_spring.hpp"

#include "mass_spring_gui.hpp"
#include "mesh.h"
#include "object_manager.hpp"
#include "renderer.h"

#define N 20
#define M 20

// Define physical parameters of the simulation
#define K_SPRING 1
#define NODE_MASS 1
#define TimeStep 0.1f

Renderer *renderer = new Renderer();
ObjectManager manager;
Integrator* integrator = new Integrator(TimeStep);
MassSpring *mass_spring;

Object mfloor;
Object cloth;
Object bunny;
Contact *planeContact;

int initialize_scene() {
  ///////////////// CREATE RENDERER & MESHES /////////////////////

  const double step = 0.5;
  SimpleMesh mesh;
  SimpleMesh planeMesh;
  CreateGrid(mesh, N, M, step);
  CreateGrid(planeMesh, N, M, step);

  manager.loadMesh("bunny", TEXTURE_PATH "/bunny.obj");
  manager.loadMesh("cloth", mesh);
  manager.loadMesh("plane", planeMesh);

  manager.loadShader("texture", SHADER_PATH "/test.v0.vert",
                     SHADER_PATH "/texture.frag");
  manager.loadShader("color", SHADER_PATH "/test.v0.vert",
                     SHADER_PATH "/vert_color.frag");
  manager.loadShader("normals", SHADER_PATH "/test.v0.vert",
                     SHADER_PATH "/normals.frag");
  manager.loadShader("geo_normals", SHADER_PATH "/normals_geom.vert",
                     SHADER_PATH "/normals.geom", SHADER_PATH "/color.frag");

  manager.loadTexture("gandalf", TEXTURE_PATH "/gandalf.png");
  manager.loadTexture("floor", TEXTURE_PATH "/floor.jpg");

  mfloor = manager.createObject("plane", "texture", "geo_normals");
  cloth = manager.createObject("cloth", "normals", "geo_normals");
  bunny = manager.createObject("bunny", "normals");

  cloth.useTexture("gandalf", manager.getTextureID("gandalf"));
  mfloor.useTexture("gandalf", manager.getTextureID("floor"));

  // Set up model matrix for the object
  cloth.translation = glm::vec3(-0.5 * N, 0.0f, -4.0f*N);
  cloth.scaling = glm::vec3(3.0);
  cloth.updateModelMatrix();

  mfloor.translation = glm::vec3(-25.0f, -1.05f, -25.0f);
  mfloor.scaling = glm::vec3(5);
  mfloor.updateModelMatrix();

  // Add the object to the renderer
  renderer->addObject(&cloth);
// renderer.addObject(&mfloor);

////////////////// ADD INTEGRATOR & SIMULABLES ////////////////////
  mass_spring = new MassSpring(integrator, &cloth, NODE_MASS, K_SPRING);

  InfPlane plane;
  plane.normal = vec3(0, 1, 0);
  plane.center = vec3(0, -1, 0);
  planeContact = new Contact(plane);
  // mass_spring->add_interaction(planeContact);

  // Fix corners
  const int index = M * (N - 1);
  mass_spring->fix_particle(0);
  mass_spring->fix_particle(index);

  integrator->fill_containers();
  return 0;
}

void pyResetSimulation(std::vector<double> parameters) {
  const double k = parameters[0];
  const double step = 0.5;
  SimpleMesh mesh;
  CreateGrid(mesh, N, M, step);
  manager.loadMesh("cloth", mesh);

  delete mass_spring;
  // BUG: Deleting and creating a new Integrator does not reset it properly
  integrator->clear_simulables();
  mass_spring = new MassSpring(integrator, &cloth, NODE_MASS, k);

  // mass_spring->add_interaction(planeContact);

  // Fix corners
  const int index = M * (N - 1);
  mass_spring->fix_particle(0);
  mass_spring->fix_particle(index);
  integrator->fill_containers();
  // std::cout << "Mass spring k = " << mass_spring->get_k() << std::endl;
  // std::cout << "Mass spring k bend = " << mass_spring->get_k_bend() << std::endl;
}

void pySetNewK(double k) { mass_spring->set_k(k); }

double pyGetK() { return mass_spring->get_k(); }

void pyFillContainers() { integrator->fill_containers(); }

void pyRecieveDeltaV(Eigen::VectorXd delta_v) {
  integrator->reciveDeltaV(delta_v);
}

Eigen::SparseMatrix<double> pyGetEquationMatrix() {
  return integrator->getEquationMatrix();
}

Eigen::VectorXd pyGetEquationVector() { return integrator->getEquationVector(); }

Eigen::VectorXd pyGetForceVector() { return integrator->getForceVector(); }

Eigen::MatrixXd pyGetParametersJacobian() {
  return integrator->getParameterJacobian();
}

Eigen::SparseMatrix<double> pyGetMassMatrix() {
  return integrator->getMassMatrix();
}

Eigen::VectorXd pyGetPosition() { return integrator->x; }

Eigen::VectorXd pyGetVelocity() { return integrator->v; }

float pyGetTimeStep() { return integrator->getTimeStep(); }

Eigen::SparseMatrix<double> pyGetForcePositionJacobian() {
  return integrator->getForcePositionJacobian();
}

void pyRenderState() { renderer->render(); }

bool pyWindowShouldClose() { return renderer->windowShouldClose(); }

void pyCameraInput() { renderer->cameraInput(); }

int pyGetDoF() { return integrator->getDoF(); }

void pyDisableRendering() { delete renderer; }

void pySetState(Eigen::VectorXd xi, Eigen::VectorXd vi) {
  integrator->set_state(xi, vi);
}

PYBIND11_MODULE(symulathon, m) {
  m.doc() = "Simple simulation interface"; // optional module docstring

  m.def("initialize_scene", &initialize_scene,
        "Generates the context + the objects in the scene");

  m.def("restart_simulation", &pyResetSimulation,
        "Resets the simulables in the scene with new parameters");

  m.def("fill_containers", &pyFillContainers,
        "Fills the position, velocity, force and derivative containers");

  m.def("recieve_delta_v", &pyRecieveDeltaV,
        "Accepts an increment of velocity and updates the system accordingly");

  m.def("get_equation_matrix", &pyGetEquationMatrix,
        "Returns the current equation matrix. Needs a call to fill_containers "
        "first to have the current forces and derivatives.");

  m.def("get_equation_vector", &pyGetEquationVector,
        "Returns the current equation vector. Needs a call to fill_containers "
        "first to have the current forces and derivatives.");

  m.def("get_force_position_jacobian", &pyGetForcePositionJacobian,
        "Returns the current dfdx. Needs a call to fill_containers first to "
        "have the current forces and derivatives.");

  m.def("get_parameter_jacobian", &pyGetParametersJacobian,
        "Returns the current df/dp jacobian. Needs a call to fill_containers "
        "first to be updated.");

  m.def("get_mass_matrix", &pyGetMassMatrix,
        "Returns the mass matrix. (nDoF x nDoF dimensionality)");

  m.def("get_position", &pyGetPosition,
        "Returns the position vector in the current state");

  m.def("get_velocity", &pyGetVelocity,
        "Returns the velocity vector in the current state");

  m.def("get_force", &pyGetForceVector,
        "Returns the force vector in the current state. Needs to call "
        "fill_containers first.");

  m.def("get_time_step", &pyGetTimeStep,
        "Returns the timestep of the simultaion");

  m.def("render_state", &pyRenderState, "Render call");

  m.def("window_should_close", &pyWindowShouldClose,
        "Tells weather or not the simulation window should close");

  m.def("process_input", &pyCameraInput,
        "Reads and process the window's input");

  m.def("set_new_k", &pySetNewK, "Recieves ands updates the spring constant");

  m.def("get_k", &pyGetK, "Returns current spring constant");

  m.def("get_nDoF", &pyGetDoF,
        "Returns number of degrees of freedom of the system");

  m.def("disable_rendering", &pyDisableRendering,
        "Destroys renderer to simulate not graphically");

  m.def("set_state", &pySetState, "Sets positions and velocities");
}
