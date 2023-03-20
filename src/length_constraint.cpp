#include "length_constraint.hpp"

void LengthConstraint::fill_containers() {
    vec3 constraint = p1 - p2;
    for (size_t i = 0; i < 3; i++){
        itg->constraint_value[index + i] = constraint.e[i];
    }

    Eigen::Matrix3d dcdxa = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d dcdxb = - Eigen::Matrix3d::Identity();

    Eigen::Matrix3d dcdtha = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d dcdthb = Eigen::Matrix3d::Identity();

    if (com1 != nullptr) {
        dcdtha = - skew(p1 - *com1);
    }

    if (com2 != nullptr) {
        dcdthb = skew(p2 - *com2);
    }
    for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j < 3; j++) {
            itg->add_constraint_jacobian_triplet(Eigen::Triplet<double>(jindex + i, i1 + j, dcdxa(i, j)));
            itg->add_constraint_jacobian_triplet(Eigen::Triplet<double>(jindex + i, i2 + j, dcdxb(i, j)));

            itg->add_constraint_jacobian_triplet(Eigen::Triplet<double>(i1 + j, jindex + i, dcdtha(i, j)));
            itg->add_constraint_jacobian_triplet(Eigen::Triplet<double>(i2 + j + 3, jindex + i, dcdthb(i, j)));
        }
    }
}
