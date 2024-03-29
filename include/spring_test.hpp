#ifndef TestingSpring_H_
#define TestingSpring_H_

#include <Eigen/Dense>
#include <vector>
#include "particle_system.hpp"
#include "twobody.hpp"
#include "vec3.h"
#include "math.h"

/* 
!!!!!!!!!!!!!!!!!!!
DO NOT EDIT THIS FILE
!!!!!!!!!!!!!!!!!!!
This file has been generated from the template <python_interaction.cpp> using a python script.
 */

class TestingSpring : public TwoBodyInteraction {
    public:
        TestingSpring(unsigned int pi, unsigned int pj, std::vector<double> inputParameters)
                                                    : TwoBodyInteraction(pi,pj)
        {
            n_parameters = 3;
            if (inputParameters.size() != n_parameters) {
                std::cout << "ERROR::TestingSpring::TestingSpring: Wrong number of initial parameters" << std::endl;
            }
            for (int i = 0; i < n_parameters; i++) {
                parameters[i]  = inputParameters[i];
            }
        }
    private:
        double parameters[3] = {0};

        double energy() const override {
            double E = (1.0/2.0)*(parameters[2])*(parameters[0])*pow(L - (parameters[1]), 2);
            return E;
        }

        vec3 force() const override {
            double f[3][1] = {{-(parameters[2])*(parameters[0])*(L - (parameters[1]))*(x1 - x2)/L}, {-(parameters[2])*(parameters[0])*(L - (parameters[1]))*(y1 - y2)/L}, {-(parameters[2])*(parameters[0])*(L - (parameters[1]))*(z1 - z2)/L}};
            return vec3(f[0][0], f[1][0], f[2][0]);
        }

        Eigen::Matrix3d force_position_derivative() const override {
            double df_dx[3][3] = {{-(parameters[2])*(parameters[0]) + (parameters[1])*(parameters[2])*(parameters[0])/L - (parameters[1])*(parameters[2])*(parameters[0])*pow(x1 - x2, 2)/pow(L, 3), -(parameters[1])*(parameters[2])*(parameters[0])*(x1 - x2)*(y1 - y2)/pow(L, 3), -(parameters[1])*(parameters[2])*(parameters[0])*(x1 - x2)*(z1 - z2)/pow(L, 3)}, {-(parameters[1])*(parameters[2])*(parameters[0])*(x1 - x2)*(y1 - y2)/pow(L, 3), -(parameters[2])*(parameters[0]) + (parameters[1])*(parameters[2])*(parameters[0])/L - (parameters[1])*(parameters[2])*(parameters[0])*pow(y1 - y2, 2)/pow(L, 3), -(parameters[1])*(parameters[2])*(parameters[0])*(y1 - y2)*(z1 - z2)/pow(L, 3)}, {-(parameters[1])*(parameters[2])*(parameters[0])*(x1 - x2)*(z1 - z2)/pow(L, 3), -(parameters[1])*(parameters[2])*(parameters[0])*(y1 - y2)*(z1 - z2)/pow(L, 3), -(parameters[2])*(parameters[0]) + (parameters[1])*(parameters[2])*(parameters[0])/L - (parameters[1])*(parameters[2])*(parameters[0])*pow(z1 - z2, 2)/pow(L, 3)}};
            Eigen::Matrix3d result;
            result << df_dx[0][0], df_dx[0][1], df_dx[0][2],
                      df_dx[1][0], df_dx[1][1], df_dx[1][2],
                      df_dx[2][0], df_dx[2][1], df_dx[2][2];
            return result;
        }

        virtual Eigen::Matrix3d force_velocity_derivative() const override {
            double df_dv[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            Eigen::Matrix3d result;
            result << df_dv[0][0], df_dv[0][1], df_dv[0][2],
                      df_dv[1][0], df_dv[1][1], df_dv[1][2],
                      df_dv[2][0], df_dv[2][1], df_dv[2][2];
            return result;
        }

        virtual Eigen::MatrixXd force_parameters_derivative() const override {
            Eigen::MatrixXd result(3, n_parameters);
            double df_dp[3][3] = {{-(parameters[2])*(L - (parameters[1]))*(x1 - x2)/L, (parameters[2])*(parameters[0])*(x1 - x2)/L, -(parameters[0])*(L - (parameters[1]))*(x1 - x2)/L}, {-(parameters[2])*(L - (parameters[1]))*(y1 - y2)/L, (parameters[2])*(parameters[0])*(y1 - y2)/L, -(parameters[0])*(L - (parameters[1]))*(y1 - y2)/L}, {-(parameters[2])*(L - (parameters[1]))*(z1 - z2)/L, (parameters[2])*(parameters[0])*(z1 - z2)/L, -(parameters[0])*(L - (parameters[1]))*(z1 - z2)/L}};
            for (unsigned int i = 0; i < 3; i++) {
                for (unsigned int j = 0; j < n_parameters; j++) {
                    result(i,j) = df_dp[i][j];
                }
            }
            return result;
        }
};

#endif // TestingSpring_H_
