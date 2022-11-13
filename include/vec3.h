#ifndef VEC3_H
#define vec3_h

#include <math.h>

class vec3 {
  public:
    // constructors

    vec3() {}
    vec3(double x, double y, double z) {
      e[0] = x;
      e[1] = y;
      e[2] = z;
    }
    inline double x() { return e[0]; }
    inline double y() { return e[1]; }
    inline double z() { return e[2]; }

    inline double length2() const { return e[0] * e[0] + e[1] * e[1] + e[2] * e[2]; }
    inline double length() const { return sqrt(this->length2()); }
    void make_unit_vector();

    // overloading
    vec3 operator-() { return vec3(-e[0], -e[1], -e[2]); }
    double operator[](int i) const { return e[i]; }

    vec3 operator+=(const vec3 &v) {
      e[0] += v.e[0];
      e[1] += v.e[1];
      e[2] += v.e[2];
      return *this;
    }

    vec3 operator-=(const vec3 &v) {
      e[0] -= v.e[0];
      e[1] -= v.e[1];
      e[2] -= v.e[2];
      return *this;
    }

    vec3 operator*=(const double s) {
      e[0] *= s;
      e[1] *= s;
      e[2] *= s;
      return *this;
    }

    vec3 operator/=(const double s) {
      e[0] /= s;
      e[1] /= s;
      e[2] /= s;
      return *this;
    }
    double e[3];
};

double dot(const vec3 &v1, const vec3 &v2);

vec3 cross(const vec3 &v1, const vec3 &v2);

// overloading operators

// vector and scalar operations
vec3 operator*(const vec3 &v, const double s);

vec3 operator*(const double s, const vec3 &v);

vec3 operator/(const vec3 &v, const double s);

// vector vector operations
vec3 operator+(const vec3 &v1, const vec3 &v2);

vec3 operator-(const vec3 &v1, const vec3 &v2);

double operator*(const vec3 &v1, const vec3 &v2);

#endif // vec3_h
