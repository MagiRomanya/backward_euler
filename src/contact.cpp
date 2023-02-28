#include "contact.h"

float distance_point_sphere(const vec3& point, const Sphere& sphere){
    /* Distance between a point and a sphere.
     * The result is positive when the point is outside, zero when the point is on the surface,
     * and is negative when the point is inside the sphere. */

    const vec3 dv = point - sphere.center;
    return dv.length() - sphere.r;
}

float distance_point_inf_plane(const vec3& point, const InfPlane& plane){
    /* Distance between a point and a plane.
     * The result is positive when the point is in the region of space where the
     * normal of the plane points to the point, zero when the point is on the surface of the
     * plane, and it is negative if the normal points to the opposite direction of where the point is */

    return dot(plane.normal, plane.center - point);
}

float distance_point_fin_plane(const vec3& point, const FinPlane& plane, bool& valid){
    /* Returns the distance between a finite plane and a point.
     * The valid bool means weather or not the point is inside the finite
     * plane domain or, on the contrary, is outside of it. */

    const float d = dot(plane.normal, plane.center - point);
    const vec3& intersection = point - d * plane.normal;
    const vec3& tangent = normalize(cross(plane.up, plane.normal));
    const vec3& bitangent = normalize(cross(plane.normal, tangent));
    if (abs(dot(intersection - point, tangent)) > plane.radius) {
        valid = false;
        return 0.0f;
    }
    if (abs(dot(intersection - point, bitangent)) > plane.radius) {
        valid = false;
        return 0.0f;
    }
    valid = true;
    return d;
}

Contact::Contact(unsigned int particle, const Sphere& sphere){
    geometry_type = SPHERE;
    geometry = malloc(sizeof(Sphere));
    memcpy(geometry, (void*) &sphere, sizeof(Sphere));
}

Contact::Contact(unsigned int particle, const InfPlane& plane){
    geometry_type = INFPLANE;
    geometry = malloc(sizeof(InfPlane));
    memcpy(geometry, (void*) &plane, sizeof(InfPlane));
}

Contact::Contact(unsigned int particle, const FinPlane& plane){
    geometry_type = FINPLANE;
    geometry = malloc(sizeof(InfPlane));
    memcpy(geometry, (void*) &plane, sizeof(FinPlane));
}

Contact::~Contact(){
    // Destroy geometry
    free(geometry);
}
