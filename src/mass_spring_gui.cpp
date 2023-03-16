#include "mass_spring_gui.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "vec3.h"

void MassSpringGUI::draw() {

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::Begin("Mass Spring Parameters");
    {
        static float xyz[] = {0.0f, 0.0f, 0.0f};

        bool changed = false;
        const float speed = 0.01f;

        changed = ImGui::DragFloat3("xyz", xyz, speed);

        if (changed){
            std::vector<int> fixed = sys->get_fixed_particles();
            vec3 pos = vec3(xyz[0], xyz[1], xyz[2]);
            for (size_t i = 0; i < fixed.size(); i++){
                vec3 new_pos = pos + sys->get_particle_position(fixed[i]);
                sys->set_particle_position(fixed[i], new_pos);
            }

        }
    }
    ImGui::End();
}
