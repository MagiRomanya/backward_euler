#include "rigid_body.hpp"

void RigidBody::update_mesh_COM(glm::vec3 vCOM){
    /* Makes so that the mesh rotates arround it's COM */
    unsigned int n_vertex = m_obj->mesh->vertices.size();

    for (size_t i = 0; i < n_vertex; i++) {
        m_obj->mesh->vertices[i].Position -= vCOM;
    }
    m_obj->mesh->updateVAO();
}

void RigidBody::calculate_COM() {
    /* Calculates the center of mass of the object */
    unsigned int n_vertex = m_obj->mesh->vertices.size();

    m_COM = vec3(0.0, 0.0, 0.0);

    for (size_t i = 0; i < n_vertex; i++){
        glm::vec3 pos = m_obj->mesh->vertices[i].Position;
        m_COM += to_vec3(pos);
    }

    m_COM /= n_vertex;

    glm::vec3 lpos = glm::vec3(m_COM.x(), m_COM.y(), m_COM.z());
glm::vec3 wpos = m_obj->toWorld(lpos);

update_mesh_COM(lpos);


m_com_x = to_vec3(wpos);
m_com_v = vec3(0.0, 0.0, 0.0);
m_theta = vec3(0.0,0.0, 0.0);
m_w = vec3(0.01, 0.0, 0.0);
}

void RigidBody::calculate_intertia_tensor() {
    /* Calculates the tensor of inertia of the object */
    unsigned int n_vertex = m_obj->mesh->vertices.size();
    double u_mass = m_mass / n_vertex;

    m_inertia_tensor0 = Eigen::Matrix3d::Zero();

    for (size_t i = 0; i < n_vertex; i++){
        const vec3 pos = to_vec3(m_obj->mesh->vertices[i].Position) -  m_COM;
        m_inertia_tensor0 -= u_mass * skew(pos) * skew(pos);
    }
}


void RigidBody::fill_containers() {
    // MASS
    typedef Eigen::Triplet<double> tri;
    for (size_t i = 0; i < 3; i++){
        integrator->add_equation_triplet(tri(index + i, index + i, m_mass)); // Mass identity

        for (size_t j = 0; j < 3; j++){
            integrator->add_mass_triplet(tri(index + 3 + i, index + 3+ j, m_inertia_tensor(i, j))); // Inertia tensor
        }
    }

    // Positions and velocities
    for (size_t i = 0; i < 3; i++) {
        // COM position and velocity
        integrator->x[index + i] = m_com_x.e[i];
        integrator->v[index + i] = m_com_v.e[i];

        // Angular position and velocity
        integrator->x[index + 3 + i] = m_theta.e[i];
        integrator->v[index + 3 + i] = m_w.e[i];
    }
}

vec3 RigidBody::coriolis_torque() {
    return to_vec3(- skew(m_w) * m_inertia_tensor * to_eigen_vec3(m_w));
}

void RigidBody::update_inertia_tensor() {
    m_inertia_tensor = rotation_matrix * m_inertia_tensor0 * rotation_matrix.transpose();
}

void RigidBody::update_rotation_tensor() {
    rotation_matrix = (Eigen::Matrix3d::Identity() + skew(m_w)) * rotation_matrix;
}

void RigidBody::update_mesh_model() {
    glm::mat4 rb_translation = glm::mat4(1.0f);
    rb_translation = glm::translate(rb_translation, glm::vec3(m_com_x.x(), m_com_x.y(), m_com_x.z()));

    glm::mat4 rb_rotation = glm::mat4(1.0f);
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            // std::cout << rotation_matrix(i,j) << ", ";
            // GLM matrices are columns first then rows
            rb_rotation[j][i] = rotation_matrix(i, j);
        }
        // std::cout << std::endl;
    }
    // std::cout << std::endl;
    glm::mat4 rb_model = rb_translation * rb_rotation;
    m_obj->model = rb_model;
}

void RigidBody::update_state() {
    // Update velocities
    for (size_t i = 0; i < 3; i++) {
        m_com_v.e[i] += integrator->delta_v(index + i);
        m_w.e[i] += integrator->delta_v(index + 3 + i);
    }
    // Update positions
    m_com_x += integrator->getTimeStep() * m_com_v;
    m_theta += integrator->getTimeStep() * m_w;

    update_rotation_tensor();
    update_inertia_tensor();
    update_mesh_model();
}

void RigidBody::set_state() {
    for (size_t i = 0; i < 3; i++) {
        m_com_v.e[i] = integrator->v(index + i);
        m_w.e[i] = integrator->v(index + 3 + i);
        m_com_x.e[i] = integrator->x(index + i);
        m_theta.e[i] = integrator->x(index + 3 + i);
    }

    update_rotation_tensor();
    update_inertia_tensor();
    update_mesh_model();

}
