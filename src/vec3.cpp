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

vec3 normalize(const vec3& v){
    return v / v.length();
}

Eigen::Matrix3d outer_product(const vec3& v1, const vec3& v2){
    Eigen::Matrix3d result;
    result << v1.x() * v2.x(), v1.x() * v2.y(), v1.x() * v2.z(),
        v1.y() * v2.x(), v1.y() * v2.y(), v1.y() * v2.z(),
        v1.z() * v2.x(), v1.z() * v2.y(), v1.z() * v2.z();
    return result;
}

vec3 to_vec3(glm::vec3 &v){
    return vec3(v.x, v.y, v.z);
}
