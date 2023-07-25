#include "non_physical.hpp"
#include "integrator.hpp"
#include "vec3.hpp"

NonPhysical::NonPhysical(Object& obj)
    : render_object(obj)
{
    nDoF = 0;
    glm::vec3 center = obj.center();
    position = to_vec3(center);
}

NonPhysical::NonPhysical(Object& obj, Integrator& itg, vec3 (*trajectory_fun)(float))
    : render_object(obj)
{
    nDoF = 1;
    glm::vec3 center = obj.center();
    position = to_vec3(center);
    trajectory_function = trajectory_fun;
    itg.add_simulable(this);
}

void NonPhysical::update_state() {
    const double DeltaTime = integrator->getTimeStep();
    time += DeltaTime;
    vec3 new_position = trajectory_function(time);
    render_object.translation = new_position.to_glm();
    render_object.updateModelMatrix();
}

void NonPhysical::set_state() {
    time = integrator->x(index);
}

void NonPhysical::fill_containers() {
    integrator->x(index) = time;
    integrator->v(index) = 0;
}
