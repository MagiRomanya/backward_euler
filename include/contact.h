#ifndef CONTACT_H_
#define CONTACT_H_
#include <string.h>

#include "interaction.h"
#include "vec3.h"

enum GEOMETRY_TYPE {
SPHERE = 0,
INFPLANE = 1,
FINPLANE = 2,
};

struct ContactGeometry {
    virtual double distance_point(const vec3& point, bool& valid) { return 0.0; }
    virtual vec3 outward_direction(const vec3& point) {return vec3(0, 0, 0); }
};

struct Sphere : public ContactGeometry {
    double r;
    vec3 center;
    double distance_point(const vec3& point, bool& valid) override;
    vec3 outward_direction(const vec3& point) override;
};

struct InfPlane : public ContactGeometry {
    vec3 normal;
    vec3 center;
    double distance_point(const vec3& point, bool& valid) override;
    vec3 outward_direction(const vec3& point) override;
};

struct FinPlane : public ContactGeometry {
    vec3 normal;
    vec3 center;
    vec3 up;
    double radius;
    double distance_point(const vec3& point, bool& valid) override;
    vec3 outward_direction(const vec3& point) override;
};

class Contact : public Interaction {
    public:
        Contact(const Sphere& sphere);
        Contact(const InfPlane& plane);
        Contact(const FinPlane& plane);

        ~Contact();

        void apply(Integrator &itg, ParticleSystem* sys) override;

        inline void set_stiffness(double stiffness) { contact_stiffness = stiffness; }

    private:
        vec3 force(ParticleSystem* sys, const vec3& direction, const double dist);

        Eigen::Matrix3d force_derivative(ParticleSystem* sys, const vec3& direction, const double dist);

        GEOMETRY_TYPE geometry_type;
        ContactGeometry* geometry;
        double contact_stiffness = 100.0f;
};
#endif // CONTACT_H_
