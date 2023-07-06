#include "collision_plane.hpp"
#include "contact.hpp"
#include "glm/ext/matrix_transform.hpp"
#include "glm/geometric.hpp"
#include "glm/matrix.hpp"
#include "mesh.h"
#include "non_physical.hpp"
#include "vec3.hpp"
#include <cmath>
#include <memory>

CollisionPlane::CollisionPlane(vec3 position, vec3 normal, Renderer& renderer) {
    CreateGrid(plane_mesh, 10, 10, 100);

    plane_shader = Shader(SHADER_PATH "/test.v0.vert",
                          SHADER_PATH "/normals.frag");

    plane_obj = Object(&plane_mesh, &plane_shader);

    const glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 axis = glm::cross(up, normal.to_glm());
    float angle = to_vec3(axis).length() / normal.length();

    glm::mat4 model = glm::mat4(1.0);
    plane_mesh.make_vertex_relative_to_center();
    model = glm::translate(model, position.to_glm());
    if (angle != 0.0f)
        model = glm::rotate(model, angle, glm::normalize(axis));
    plane_obj.model = model;
    plane_obj.inverse_model = glm::inverse(model);
    renderer.addObject(&plane_obj);

    nphys = std::make_unique<NonPhysical>(plane_obj);

    infplane_geometry = std::make_unique<InfPlane>(position, normal);

    contact = std::make_unique<Contact>(infplane_geometry.get());
}
