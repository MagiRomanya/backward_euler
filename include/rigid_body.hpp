#ifndef RIGID_BODY_H_
#define RIGID_BODY_H_

#include "integrator.hpp"
#include "simulable.hpp"
#include "vec3.h"
#include "object.h"

class RigidBody : public Simulable {
    public:
        RigidBody (Integrator* itg, Object* obj, double mass) {
            nDoF = 6;
            integrator = itg;
            m_obj = obj;
            m_mass = mass;
            calculate_COM();
            calculate_intertia_tensor();
            itg->add_simulable(this);
        }

        void fill_containers() override;

        void update_state() override;

        void set_state() override;

    private:
        void calculate_COM();

        void update_mesh_COM(glm::vec3 vCOM);

        void calculate_intertia_tensor();

        void update_inertia_tensor();

        void update_rotation_tensor();

        void update_mesh_model();

        vec3 coriolis_torque();

        vec3 m_COM;

        // Center of mass position
        vec3 m_com_x;
        vec3 m_com_v;

        // Rotation angles
        vec3 m_theta;
        vec3 m_w;

        Object* m_obj;

        double m_mass;

        Eigen::Matrix3d m_inertia_tensor0;

        Eigen::Matrix3d m_inertia_tensor;

        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
};

#endif // RIGID_BODY_H_
