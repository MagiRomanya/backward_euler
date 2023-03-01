#include "contact.h"
#include "particle_system.hpp"

double Sphere::distance_point(const vec3& point, bool& valid){
    /* Distance between a point and a sphere.
     * The result is positive when the point is outside, zero when the point is on the surface,
     * and is negative when the point is inside the sphere. */

    valid = true;
    const vec3 dv = point - center;
    return dv.length() - r;
}

vec3 Sphere::outward_direction(const vec3& point){
    return normalize(point - center);
}

double InfPlane::distance_point(const vec3& point, bool& valid){
    /* Distance between a point and a plane.
     * The result is positive when the point is in the region of space where the
     * normal of the plane points to the point, zero when the point is on the surface of the
     * plane, and it is negative if the normal points to the opposite direction of where the point is */
    valid = true;
    return dot(normal, center - point);
}

vec3 InfPlane::outward_direction(const vec3 &point){
    return normal;
}

double FinPlane::distance_point(const vec3& point,  bool& valid){
    /* Returns the distance between a finite plane and a point.
     * The valid bool means weather or not the point is inside the finite
     * plane domain or, on the contrary, is outside of it. */

    const double d = dot(normal, center - point);
    const vec3& intersection = point - d * normal;
    const vec3& tangent = normalize(cross(up, normal));
    const vec3& bitangent = normalize(cross(normal, tangent));
    if (abs(dot(intersection - point, tangent)) > radius) {
        valid = false;
        return 0.0f;
    }
    if (abs(dot(intersection - point, bitangent)) > radius) {
        valid = false;
        return 0.0f;
    }
    valid = true;
    return d;
}

vec3 FinPlane::outward_direction(const vec3 &point){
    return normal;
}

Contact::Contact(const Sphere& sphere){
    geometry_type = SPHERE;
    geometry = (ContactGeometry*) malloc(sizeof(Sphere));
    memcpy((void*) geometry, (void*) &sphere, sizeof(Sphere));
}

Contact::Contact(const InfPlane& plane){
    geometry_type = INFPLANE;
    geometry = (ContactGeometry*) malloc(sizeof(InfPlane));
    memcpy((void*) geometry, (void*) &plane, sizeof(InfPlane));
}

Contact::Contact(const FinPlane& plane){
    geometry_type = FINPLANE;
    geometry = (ContactGeometry*) malloc(sizeof(FinPlane));
    memcpy((void*) geometry, (void*) &plane, sizeof(FinPlane));
}

Contact::~Contact(){
    // Destroy geometry
    free(geometry);
}

void Contact::apply(Integrator &itg, ParticleSystem* sys) {
    bool valid;
    // std::cout << geometry->distance_point(sys->get_particle_position(19), valid) << std::endl;
    for (int i = 0; i < sys->get_n_particles(); i++) {
        ///////////// COLLISION DETECTION ///////////////
        bool valid;
        vec3 point = -sys->get_particle_position(i);
        double dist = geometry->distance_point(point, valid);

        if (!valid) continue;
        if (dist > 0) continue;
        if (sys->is_fixed(i)) continue;

        /////////////// COLLISION RESPONSE ////////////////
        vec3 direction = geometry->outward_direction(point);
        vec3 f = force(sys, direction, dist);
        itg.f0(sys->index + 3*i) = f.x();
        itg.f0(sys->index + 3*i+1) = f.y();
        itg.f0(sys->index + 3*i+2) = f.z();

        Eigen::Matrix3d df_dx = force_derivative(sys, direction, dist);
        typedef Eigen::Triplet<double> tri;
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++) {
                itg.add_df_dx_triplet(tri(sys->index + 3*i+j, sys->index + 3*i+k, df_dx(j,k)));
            }
        }
    }

}

vec3 Contact::force(ParticleSystem* sys, const vec3& direction, const double dist) {
    return - contact_stiffness * dist * direction;
}

Eigen::Matrix3d Contact::force_derivative(ParticleSystem* sys, const vec3& direction, const double dist) {
    return - contact_stiffness * Eigen::Matrix3d::Identity();
}
