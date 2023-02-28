#include "spring.h"

void Spring::get_state(ParticleSystem &sys){
    unsigned int sys_index = sys.index;
    itg_i = sys.index + i*3;
    itg_j = sys.index + j*3;

    x1 = sys.get_particle_position(i);
    x2 = sys.get_particle_position(j);

    L = (x1 - x2).length();
}

vec3 Spring::force(const Integrator &igt) const {
    /* Computes the spring force */
    double f = -k * (1.0 - L0 / L);
    vec3 force = f * (x1 - x2);
    return force;
}

double Spring::energy(const Integrator &igt) const {
    /* Computes the spring's energy */
    double energy = 0.5 * k * (L - L0) * (L - L0);
    return energy;
}

Eigen::Matrix3d outer_product(const vec3& v1, const vec3& v2){
    Eigen::Matrix3d result;
    result << v1.x() * v2.x(), v1.x() * v2.y(), v1.x() * v2.z(),
        v1.y() * v2.x(), v1.y() * v2.y(), v1.y() * v2.z(),
        v1.z() * v2.x(), v1.z() * v2.y(), v1.z() * v2.z();
    return result;
}

Eigen::Matrix3d Spring::force_derivative(const System &s) const {
    /*Calculates the derivative of the force of a string with respect to position
     * df/dx */
    // u is the normalized vector between particles 1 and 2
    vec3 u = (x1 - x2)/L;
    // Initialize the derivative matrix
    Eigen::Matrix3d df_dx = (L - L0) * Eigen::Matrix3d::Identity();
    // The u · u.transpose() matrix
    Eigen::Matrix3d uut = outer_product(u, u);

    // Calculate the final derivative matrix
    df_dx = - k / L * (df_dx + L0 * uut);
    return df_dx; // 3x3 matrix
}

Eigen::Matrix3d Spring::force_derivative_finite(System &s) const {
    double epsilon = 0.01;
    vec3 current_force = force(s); // f(x)
    Eigen::Matrix3d df_dx;
    for (int a=0; a < 3; a++){
        for (int b=0; b < 3; b++){
            s.x(3*i + b) += epsilon;
            vec3 new_force = force(s); // f(x + epsilon)
            // df_dx = 1/eps * (f(x+epsilon) - f(x))
            df_dx(a,b) = 1.0/epsilon * (new_force[a] - current_force[a]);
            s.x(3*i + b) -= epsilon;
        }
    }
    return df_dx;
}

void Spring::add_energy(System &s) const {
    /* Adds the energy to the system */
    s.energy += energy(s);
}

void Spring::add_force(System &s) const {
    /* Adds the force to the system */
    vec3 calculated_force = force(s);
    if (!s.fixed[i]){
        s.f0(3 * i) += calculated_force.x();     // f0.x
        s.f0(3 * i + 1) += calculated_force.y(); // f0.y
        s.f0(3 * i + 2) += calculated_force.z(); // f0.z
    }

    // 3º Newton law: Equal and oposite reaction
    if (!s.fixed[j]){
        s.f0(3 * j) += -calculated_force.x();     // f0.x
        s.f0(3 * j + 1) += -calculated_force.y(); // f0.y
        s.f0(3 * j + 2) += -calculated_force.z(); // f0.z
    }
}

void Spring::add_derivative(System &s) const {
    /* Adds the force derivatives to the system */
    Eigen::Matrix3d df_dx = this->force_derivative(s);
    // Eigen::Matrix3d df_dx = this->force_derivative_finite(s);
    typedef Eigen::Triplet<double> tri;
    const int p1 = this->i;
    const int p2 = this->j;
    const double h2 = s.h * s.h;
    if (s.fixed[i] and s.fixed[p2]) return;
    if (s.fixed[i]){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, df_dx(j, k)));
                s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, -h2 * df_dx(j, k)));
            }
        }
        return;
    }
    if (s.fixed[p2]){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, df_dx(j, k)));
                s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, -h2 * df_dx(j, k)));
            }
        }
        return;
    }
    for (int j=0; j < 3; j++){
        for (int k=0; k < 3; k++){
            // The df_dx derivative
            s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, df_dx(j, k)));
            s.df_dx_triplets.push_back(tri(3 * p1 + j, 3 * p2 + k, -df_dx(j, k)));
            s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p1 + k, -df_dx(j, k)));
            s.df_dx_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, df_dx(j, k)));

            // The full equation matrix
            // In this equation we need
            // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;
            s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p1 + k, -h2 * df_dx(j, k)));
            s.equation_matrix_triplets.push_back(tri(3 * p1 + j, 3 * p2 + k, h2 * df_dx(j, k)));
            s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p1 + k, h2 * df_dx(j, k)));
            s.equation_matrix_triplets.push_back(tri(3 * p2 + j, 3 * p2 + k, -h2 * df_dx(j, k)));
        }
    }
}

void Spring::apply(System &s){
    /* Interaction's method to apply it's effect to the system */
    add_energy(s);
    add_force(s);
    add_derivative(s);
}
