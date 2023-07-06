#ifndef COLLISION_BALL_H_
#define COLLISION_BALL_H_

#include <memory>

#include "contact.hpp"
#include "mesh.h"
#include "non_physical.hpp"
#include "object.h"
#include "particle_system.hpp"
#include "renderer.h"
#include "shader.h"

class CollisionBall {
    public:
        CollisionBall(vec3 position, double radius, Renderer& renderer);

        inline Contact* getContact() { return contact.get(); }
    private:
        Shader sphere_shader;
        SimpleMesh sphere_mesh;
        Object sphere_obj;

        std::unique_ptr<Sphere> sphere_contact_geometry;
        std::unique_ptr<Contact> contact;
        std::unique_ptr<NonPhysical> nphys;
};

#endif // COLLISION_BALL_H_
