#ifndef MASS_SPRING_GUI_H_
#define MASS_SPRING_GUI_H_

#include "gui_element.hpp"
#include "mass_spring.hpp"
#include "renderer.h"

class MassSpringGUI : public GUI_element {
    public:
        MassSpringGUI(MassSpring* sys, Renderer* renderer) : sys(sys), renderer(renderer) {
            renderer->add_GUI_element(this);
        }

        void draw() override;

    private:
        MassSpring* sys;
        Renderer* renderer;

};

#endif // MASS_SPRING_GUI_H_
