#include "collision_ball.hpp"
#include "mesh.h"
#include "non_physical.hpp"
#include "particle_system.hpp"
#include "shader_path.h"

CollisionBall::CollisionBall(vec3 position, double radius) {
    sphere_mesh.loadFromFile(TEXTURE_PATH"/sphere.obj");

    sphere_obj = Object(&sphere_mesh);

    sphere_obj.translation = position.to_glm();
    // Not full radius to avoid z fighting
    sphere_obj.scaling = glm::vec3(radius*0.95);
    sphere_obj.updateModelMatrix();

    nphys = std::make_unique<NonPhysical>(sphere_obj);

    sphere_contact_geometry = std::make_unique<Sphere>(position, radius);
    contact = std::make_unique<Contact>(sphere_contact_geometry.get());
}

CollisionBall::CollisionBall(vec3 position, double radius, Renderer& renderer) {
    sphere_mesh.loadFromFile(TEXTURE_PATH"/sphere.obj");

    sphere_shader = Shader(SHADER_PATH "/test.v0.vert",
                           SHADER_PATH "/normals.frag");

    sphere_obj = Object(&sphere_mesh, &sphere_shader);

    sphere_obj.translation = position.to_glm();
    sphere_obj.scaling = glm::vec3(radius);
    sphere_obj.updateModelMatrix();
    renderer.addObject(&sphere_obj);

    nphys = std::make_unique<NonPhysical>(sphere_obj);

    sphere_contact_geometry = std::make_unique<Sphere>(position, radius);
    contact = std::make_unique<Contact>(sphere_contact_geometry.get());
}
