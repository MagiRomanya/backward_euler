#include "vec3.h"


void vec3::make_unit_vector() {
    double length = this->length();
    e[0] /= length;
    e[0] /= length;
    e[0] /= length;
}

double dot(const vec3 &v1, const vec3 &v2) {
    return v1.e[0] * v2.e[0] + v1.e[1] * v2.e[1] + v1.e[2] * v2.e[2];
}

vec3 cross(const vec3 &v1, const vec3 &v2) {
    return vec3(v1.e[1] * v2.e[2] - v1.e[2] * v2.e[1],
                v1.e[2] * v2.e[0] - v1.e[0] * v2.e[2],
                v1.e[0] * v2.e[1] - v1.e[1] * v2.e[0]);
}

// overloading operators

// vector and scalar operations
vec3 operator*(const vec3 &v, const double s) {
    return vec3(s * v.e[0], s * v.e[1], s * v.e[2]);
}

vec3 operator*(const double s, const vec3 &v) {
    return vec3(s * v.e[0], s * v.e[1], s * v.e[2]);
}

vec3 operator/(const vec3 &v, const double s) {
    return vec3(v.e[0] / s, v.e[1] / s, v.e[2] / s);
}

// vector vector operations
vec3 operator+(const vec3 &v1, const vec3 &v2) {
    return vec3(v1.e[0] + v2.e[0], v1.e[1] + v2.e[1], v1.e[2] + v2.e[2]);
}

vec3 operator-(const vec3 &v1, const vec3 &v2) {
    return vec3(v1.e[0] - v2.e[0], v1.e[1] - v2.e[1], v1.e[2] - v2.e[2]);
}

double operator*(const vec3 &v1, const vec3 &v2) { return dot(v1, v2); }

// printing a vec3

std::ostream& operator<<(std::ostream& os, const vec3& v){
os << "(" <<v.x() << ", " << v.y() << ", " << v.z() << ")";
return os;
}
