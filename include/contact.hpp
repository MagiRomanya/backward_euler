#ifndef CONTACT_H_
#define CONTACT_H_

#include "interaction.hpp"
#include "vec3.hpp"

enum GEOMETRY_TYPE {
SPHERE = 0,
INFPLANE = 1,
FINPLANE = 2,
};

struct ContactGeometry {
    virtual double distance_point(const vec3& point, bool& valid) const = 0;
    virtual vec3 outward_direction(const vec3& point) const = 0;
};

struct Sphere : public ContactGeometry {
        Sphere(vec3 center, double r) : r(r), center(center) {}
        double r;
        vec3 center;
        double distance_point(const vec3& point, bool& valid) const override;
        vec3 outward_direction(const vec3& point) const override;
};

struct InfPlane : public ContactGeometry {
        InfPlane(vec3 position, vec3 normal) : center(position), normal(normal) {}
        vec3 normal;
        vec3 center;
        double distance_point(const vec3& point, bool& valid) const override;
        vec3 outward_direction(const vec3& point) const override;
};

struct FinPlane : public ContactGeometry {
        vec3 normal;
        vec3 center;
        vec3 up;
        double radius;
        double distance_point(const vec3& point, bool& valid) const override;
        vec3 outward_direction(const vec3& point) const override;
};

class Contact : public Interaction {
    public:
        /* Contact class does NOT own the ContactGeometry pointer */
        Contact(const Sphere* sphere);
        Contact(const InfPlane* plane);
        Contact(const FinPlane* plane);

        ~Contact();

        void apply(Integrator &itg, ParticleSystem* sys) override;

        inline void set_stiffness(double stiffness) { contact_stiffness = stiffness; }

    private:
        vec3 force(const vec3& direction, const double dist);

        Eigen::Matrix3d force_derivative(ParticleSystem* sys, const vec3& direction, const double dist);

        GEOMETRY_TYPE geometry_type;
        const ContactGeometry* geometry;
        double contact_stiffness = 1000.0f;
};
#endif // CONTACT_H_
