#ifndef FIXER_H_
#define FIXER_H_

#include <glm/glm.hpp>

#include "object.h"
#include "system.h"
#include "intersection.h"

class Fixer{
    public:
        Object* obj;
        System* s;
        Fixer(Object* obj, System* s){
            this->obj = obj;
            this->s = s;
        }

        void fix(){
            /* Fixes the particles inside of the fixer's mesh */
            if (s->num != s->mesh.vertices.size()){
                std::cout << "FIXER::ERROR: System's mesh vertices and number of particles does not match" << std::endl;
            }
            for (int i = 0; i < s->num; i++){
                glm::vec3& pos = s->mesh.vertices[i].Position;

                if (is_inside(*obj, pos)){
                    s->fix_particle(i);
                };
            }
        }
};

#endif // FIXER_H_
