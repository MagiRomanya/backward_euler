#ifndef CONTACT_H_
#define CONTACT_H_
#include <string.h>
#include "vec3.h"

enum GEOMETRY_TYPE {
SPHERE = 0,
INFPLANE = 1,
FINPLANE = 2,
};

struct Sphere{
    float r;
    vec3 center;
};

struct InfPlane{
    vec3 normal;
    vec3 center;
};

struct FinPlane{
    vec3 normal;
    vec3 center;
    vec3 up;
    float radius;
};

float distance_point_sphere(const vec3& point, const Sphere& sphere);

float distance_point_inf_plane(const vec3& point, const InfPlane& plane);

float distance_point_fin_plane(const vec3& point, const FinPlane& plane, bool& valid);

class Contact{
    public:
        Contact(unsigned int particle,const Sphere& sphere);
        Contact(unsigned int particle,const InfPlane& plane);
        Contact(unsigned int particle,const FinPlane& plane);

        ~Contact();

        GEOMETRY_TYPE geometry_type;
        void* geometry;
    private:
};
#endif // CONTACT_H_
