#ifndef COLLISION_PLANE_H_
#define COLLISION_PLANE_H_

#include "contact.hpp"
#include "mesh.h"
#include "non_physical.hpp"
#include "renderer.h"
#include "shader.h"
#include "vec3.hpp"
#include <memory>

class CollisionPlane {
    public:
        CollisionPlane(vec3 position, vec3 normal, Renderer& renderer);

        inline Contact* getContact() { return contact.get(); }

    private:
        Shader plane_shader;
        SimpleMesh plane_mesh;
        Object plane_obj;

        std::unique_ptr<NonPhysical> nphys;
        std::unique_ptr<InfPlane> infplane_geometry;
        std::unique_ptr<Contact> contact;
};

#endif // COLLISION_PLANE_H_
