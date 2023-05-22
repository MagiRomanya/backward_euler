#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "Eigen/src/Core/Matrix.h"
#include "mass_spring.hpp"
#include "integrator.hpp"
#include "clock.h"
#include "contact.h"

#include "mass_spring_gui.hpp"
#include "mesh.h"
#include "renderer.h"
#include "object_manager.hpp"

#define N 5
#define M 5


// Define physical parameters of the simulation
#define K_SPRING 100000
#define NODE_MASS 100

Renderer renderer;
ObjectManager manager;
Integrator integrator(0.05f);
MassSpring* mass_spring;

Object mfloor;
Object cloth;
Object bunny;
Contact* planeContact;

int initialize_scene() {
    ///////////////// CREATE RENDERER & MESHES /////////////////////

    const double step = 0.5;
    SimpleMesh mesh;
    SimpleMesh planeMesh;
    CreateGrid(mesh, N, M, step);
    CreateGrid(planeMesh, N, M, step);

    manager.loadMesh("bunny", TEXTURE_PATH"/bunny.obj");
    manager.loadMesh("cloth", mesh);
    manager.loadMesh("plane", planeMesh);

    manager.loadShader("texture",SHADER_PATH"/test.v0.vert", SHADER_PATH"/texture.frag");
    manager.loadShader("color",SHADER_PATH"/test.v0.vert", SHADER_PATH"/vert_color.frag");
    manager.loadShader("normals",SHADER_PATH"/test.v0.vert", SHADER_PATH"/normals.frag");
    manager.loadShader("geo_normals", SHADER_PATH"/normals_geom.vert", SHADER_PATH"/normals.geom", SHADER_PATH"/color.frag");

    manager.loadTexture("gandalf", TEXTURE_PATH"/gandalf.png");
    manager.loadTexture("floor", TEXTURE_PATH"/floor.jpg");

    mfloor = manager.createObject("plane", "texture", "geo_normals");
    cloth = manager.createObject("cloth", "normals", "geo_normals");
    bunny = manager.createObject("bunny", "normals");

    cloth.useTexture("gandalf", manager.getTextureID("gandalf"));
    mfloor.useTexture("gandalf", manager.getTextureID("floor"));

    // Set up model matrix for the object
    cloth.translation = glm::vec3(0.0f, 0.25 * N, -4.0f);
    // cloth.scaling = glm::vec3(3.0);
    cloth.updateModelMatrix();

    mfloor.translation = glm::vec3(-25.0f, -1.05f, -25.0f);
    mfloor.scaling = glm::vec3(5);
    mfloor.updateModelMatrix();

    // Add the object to the renderer
    renderer.addObject(&cloth);
    // renderer.addObject(&mfloor);

    ////////////////// ADD INTEGRATOR & SIMULABLES ////////////////////
    mass_spring = new MassSpring(&integrator , &cloth, NODE_MASS, K_SPRING);

    InfPlane plane;
    plane.normal = vec3(0, 1, 0);
    plane.center = vec3(0, -1, 0);
    planeContact = new Contact(plane);
    // mass_spring->add_interaction(planeContact);


    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);

    return 0;
}

void pyResetSimulation(double k) {
    const double step = 0.5;
    SimpleMesh mesh;
    CreateGrid(mesh, N, M, step);
    manager.loadMesh("cloth", mesh);

    integrator.clear_simulables();
    delete mass_spring;
    mass_spring = new MassSpring(&integrator , &cloth, NODE_MASS, k);

    // mass_spring->add_interaction(planeContact);

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
}

int pymain() {
    // RENDER LOOP
    while (!renderer.windowShouldClose()){
        // Input
        renderer.cameraInput();

        // Simulation step
        {
            Clock timer("Simulation Step");
            integrator.integration_step();
        }

        // Render
        renderer.render();
    }
    Clock results("result");
    results.printClocks();
    delete mass_spring;
    delete planeContact;
    return 0;
}

void pyFillContainers() {
    integrator.fill_containers();
}

void pyRecieveDeltaV(Eigen::VectorXd delta_v) {
    integrator.reciveDeltaV(delta_v);
}

Eigen::SparseMatrix<double> pyGetEquationMatrix() {
    return integrator.getEquationMatrix();
}

Eigen::VectorXd pyGetEquationVector() {
    return integrator.getEquationVector();
}

Eigen::VectorXd pyGetForceVector() {
    return integrator.getForceVector();
}

Eigen::MatrixXd pyGetParametersJacobian() {
    return integrator.getParameterJacobian();
}

Eigen::SparseMatrix<double> pyGetMassMatrix() {
    return integrator.getMassMatrix();
}

Eigen::VectorXd pyGetPosition() {
    return integrator.x;
}

Eigen::VectorXd pyGetVelocity() {
    return integrator.v;
}

float pyGetTimeStep() {
    return integrator.getTimeStep();
}

Eigen::SparseMatrix<double> pyGetForcePositionJacobian() {
    return integrator.getForcePositionJacobian();
}

void pyRenderState() {
    renderer.render();
}

bool pyWindowShouldClose() {
    return renderer.windowShouldClose();
}

void pyCameraInput() {
    renderer.cameraInput();
}

PYBIND11_MODULE(symulathon, m) {
    m.doc() = "Simple simulation interface"; // optional module docstring

    m.def("initialize_scene", &initialize_scene, "Generates the context + the objects in the scene");

    m.def("restart_simulation", &pyResetSimulation, "Resets the simulables in the scene with new parameters");

    m.def("fill_containers", &pyFillContainers, "Fills the position, velocity, force and derivative containers");

    m.def("recieve_delta_v", &pyRecieveDeltaV, "Accepts an increment of velocity and updates the system accordingly");

    m.def("get_equation_matrix", &pyGetEquationMatrix, "Returns the current equation matrix. Needs a call to fill_containers first to have the current forces and derivatives.");

    m.def("get_equation_vector", &pyGetEquationVector, "Returns the current equation vector. Needs a call to fill_containers first to have the current forces and derivatives.");

    m.def("get_force_position_jacobian", &pyGetForcePositionJacobian, "Returns the current dfdx. Needs a call to fill_containers first to have the current forces and derivatives.");

    m.def("get_parameter_jacobian", &pyGetParametersJacobian, "Returns the current df/dp jacobian. Needs a call to fill_containers first to be updated.");

    m.def("get_mass_matrix", &pyGetMassMatrix, "Returns the mass matrix. (nDoF x nDoF dimensionality)");

    m.def("get_position", &pyGetPosition, "Returns the position vector in the current state");

    m.def("get_velocity", &pyGetVelocity, "Returns the velocity vector in the current state");

    m.def("get_force", &pyGetForceVector, "Returns the force vector in the current state. Needs to call fill_containers first.");

    m.def("get_time_step", &pyGetTimeStep, "Returns the timestep of the simultaion");

    m.def("render_state", &pyRenderState, "Render call");

    m.def("window_should_close", &pyWindowShouldClose, "Tells weather or not the simulation window should close");

    m.def("process_input", &pyCameraInput, "Reads and process the window's input");

    m.def("mainloop", &pymain, "The mainloop from main.cpp");
}
