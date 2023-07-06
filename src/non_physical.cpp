#include "non_physical.hpp"
#include "vec3.hpp"

NonPhysical::NonPhysical(Object& obj)
    : render_object(obj)
{
    nDoF = 0;
    glm::vec3 center = obj.center();
    position = to_vec3(center);
}

void NonPhysical::update_state() {

}
