#include "twobody.hpp"

TwoBodyInteraction::TwoBodyInteraction(unsigned int pi, unsigned int pj) {
    p1 = pi;
    p2 = pj;
}

void TwoBodyInteraction::apply(Integrator &itg, ParticleSystem *sys) {
    get_state(sys);
    add_force(itg, sys);
    add_force_derivatives(itg, sys);
    add_parameters_derivative(itg, sys);
}

void TwoBodyInteraction::get_state(ParticleSystem* sys) {
    itg_p1 = sys->index + 3*p1;
    itg_p2 = sys->index + 3*p2;

    vec3 r1 = sys->get_particle_position(p1);
    vec3 r2 = sys->get_particle_position(p2);

    L = (r1 - r2).length();

    x1 = r1.x(); y1 = r1.y(); z1 = r1.z();
    x2 = r2.x(); y2 = r2.y(); z2 = r2.z();

    vec3 v1 = sys->get_particle_velocity(p1);
    vec3 v2 = sys->get_particle_velocity(p2);
    vx1 = v1.x(); vy1 = v1.y(); vz1 = v1.z();
    vx2 = v2.x(); vy2 = v2.y(); vz2 = v2.z();
}

void TwoBodyInteraction::add_force(Integrator &itg, ParticleSystem* sys) {
    /* Adds the force to the system */
    vec3 calculated_force = force();

    if (!sys->is_fixed(p1)){
        itg.f0(itg_p1) += calculated_force.x();     // f0.x
        itg.f0(itg_p1 + 1) += calculated_force.y(); // f0.y
        itg.f0(itg_p1 + 2) += calculated_force.z(); // f0.z
    }

    // 3º Newton law: Equal and oposite reaction
    if (!sys->is_fixed(p2)){
        itg.f0(itg_p2) += -calculated_force.x();     // f0.x
        itg.f0(itg_p2 + 1) += -calculated_force.y(); // f0.y
        itg.f0(itg_p2 + 2) += -calculated_force.z(); // f0.z
    }
}

void TwoBodyInteraction::add_force_derivatives(Integrator &itg, ParticleSystem* sys) {
    /* Adds the force derivatives to the system */
    Eigen::Matrix3d df_dx = force_position_derivative();
    Eigen::Matrix3d df_dv = force_velocity_derivative();
    // Eigen::Matrix3d df_dx = this->force_derivative_finite(s);
    typedef Eigen::Triplet<double> tri;
    const double h2 = itg.getTimeStep() * itg.getTimeStep();
    const double h = itg.getTimeStep();

    if (sys->is_fixed(p1) and sys->is_fixed(p2)) return;

    if (sys->is_fixed(p1)){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                itg.add_df_dx_triplet(tri(itg_p2 + j, itg_p2 + k, df_dx(j, k)));
                itg.add_df_dv_triplet(tri(itg_p2 + j, itg_p2 + k, df_dv(j, k)));
                itg.add_equation_triplet(tri(itg_p2 + j, itg_p2 + k, -h * df_dv(j, k) - h2 * df_dx(j, k)));
            }
        }
        return;
    }
    if (sys->is_fixed(p2)){
        for (int j=0; j < 3; j++){
            for (int k=0; k < 3; k++){
                itg.add_df_dx_triplet(tri(itg_p1 + j, itg_p1 + k, df_dx(j, k)));
                itg.add_df_dv_triplet(tri(itg_p1 + j, itg_p1 + k, df_dv(j, k)));
                itg.add_equation_triplet(tri(itg_p1 + j, itg_p1 + k, -h * df_dv(j, k) -h2 * df_dx(j, k)));
            }
        }
        return;
    }
    for (int j=0; j < 3; j++){
        for (int k=0; k < 3; k++){
            // The df_dx derivative
            itg.add_df_dx_triplet(tri(itg_p1 + j, itg_p1 + k, df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_p1 + j, itg_p2 + k, -df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_p2 + j, itg_p1 + k, -df_dx(j, k)));
            itg.add_df_dx_triplet(tri(itg_p2 + j, itg_p2 + k, df_dx(j, k)));

            // The df_dv derivative
            itg.add_df_dv_triplet(tri(itg_p1 + j, itg_p1 + k, df_dv(j, k)));
            itg.add_df_dv_triplet(tri(itg_p1 + j, itg_p2 + k, -df_dv(j, k)));
            itg.add_df_dv_triplet(tri(itg_p2 + j, itg_p1 + k, -df_dv(j, k)));
            itg.add_df_dv_triplet(tri(itg_p2 + j, itg_p2 + k, df_dv(j, k)));

            // The full equation matrix
            // In this equation we need
            // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;
            itg.add_equation_triplet(tri(itg_p1 + j, itg_p1 + k, -h * df_dv(j, k) -h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_p1 + j, itg_p2 + k, h * df_dv(j, k) + h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_p2 + j, itg_p1 + k, h * df_dv(j, k) + h2 * df_dx(j, k)));
            itg.add_equation_triplet(tri(itg_p2 + j, itg_p2 + k, -h * df_dv(j, k) -h2 * df_dx(j, k)));
        }
    }
}

void TwoBodyInteraction::add_parameters_derivative(Integrator &itg, ParticleSystem *sys) {
    typedef Eigen::Triplet<double> tri;

    Eigen::MatrixXd df_dp = force_parameters_derivative();

    for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int p = 0; p < parameter_indexs.size(); p++) {
            itg.add_to_df_dp_element(itg_p1 + i, parameter_indexs[p], df_dp(i, p));
            itg.add_to_df_dp_element(itg_p2 + i, parameter_indexs[p], -df_dp(i, p));
        }
    }
}
