#include "spring.hpp"

void Spring::get_state(ParticleSystem* sys){
    itg_i = sys->index + i*3;
    itg_j = sys->index + j*3;

    x1 = sys->get_particle_position(i);
    x2 = sys->get_particle_position(j);

    L = (x1 - x2).length();
}

vec3 Spring::force() const {
    /* Computes the spring force */
    double f = -k * (1.0 - L0 / L);
    vec3 force = f * (x1 - x2);
    return force;
}

double Spring::energy() const {
    /* Computes the spring's energy */
    double energy = 0.5 * k * (L - L0) * (L - L0);
    return energy;
}

Eigen::Matrix3d Spring::force_derivative() const {
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


void Spring::add_force(Integrator &itg, ParticleSystem* sys) const {
    /* Adds the force to the system */
    vec3 calculated_force = force();

    if (!sys->is_fixed(i)){
        itg.f0(itg_i) += calculated_force.x();     // f0.x
        itg.f0(itg_i + 1) += calculated_force.y(); // f0.y
        itg.f0(itg_i + 2) += calculated_force.z(); // f0.z
    }

    // 3º Newton law: Equal and oposite reaction
    if (!sys->is_fixed(j)){
        itg.f0(itg_j) += -calculated_force.x();     // f0.x
        itg.f0(itg_j + 1) += -calculated_force.y(); // f0.y
        itg.f0(itg_j + 2) += -calculated_force.z(); // f0.z
    }
}

void Spring::add_derivative(Integrator &itg, ParticleSystem* sys) const {
    /* Adds the force derivatives to the system */
    Eigen::Matrix3d df_dx = this->force_derivative();
    // Eigen::Matrix3d df_dx = this->force_derivative_finite(s);
    typedef Eigen::Triplet<double> tri;
    // const int p1 = this->i;
    // const int p2 = this->j;
    const double h2 = itg.getTimeStep() * itg.getTimeStep();

    if (sys->is_fixed(i) and sys->is_fixed(j)) return;

    if (sys->is_fixed(i)){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                itg.add_df_dx_triplet(tri(itg_j + j, itg_j + k, df_dx(j, k)));
                itg.add_equation_triplet(tri(itg_j + j, itg_j + k, -h2 * df_dx(j, k)));
            }
        }
        return;
    }
    if (sys->is_fixed(j)){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                itg.add_df_dx_triplet(tri(itg_i + j, itg_i + k, df_dx(j, k)));
                itg.add_equation_triplet(tri(itg_i + j, itg_i + k, -h2 * df_dx(j, k)));
            }
        }
        return;
    }
    for (int j=0; j < 3; j++){
        for (int k=0; k < 3; k++){
            // The df_dx derivative
            itg.add_df_dx_triplet(tri(itg_i + j, itg_i + k, df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_i + j, itg_j + k, -df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_j + j, itg_i + k, -df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_j + j, itg_j + k, df_dx(j, k)));

            // The full equation matrix
            // In this equation we need
            // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;
            itg.add_equation_triplet(tri(itg_i + j, itg_i + k, -h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_i + j, itg_j + k, h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_j + j, itg_i + k, h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_j + j, itg_j + k, -h2 * df_dx(j, k)));
        }
    }
}

void Spring::apply(Integrator &itg, ParticleSystem* sys){
    /* Interaction's method to apply it's effect to the system */
    get_state(sys);
    add_force(itg, sys);
    add_derivative(itg, sys);
}
