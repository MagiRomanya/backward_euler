#ifndef INTERACTION_H
#define INTERACTION_H

class System;

class Interaction{
/* Abstract class which represents any interaction that can apply force to a particle
 * Examples: springs, gravity, friction, etc  */
    public:
        virtual void apply(System &s) {}
        virtual void render(System &s) {}
};


#endif // INTERACTION_H
