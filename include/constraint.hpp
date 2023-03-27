#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

class Integrator;

class Constraint {
    public:
        unsigned int nConstraints;

        unsigned int index; // index of the constraint vector

        unsigned int jindex; // index of the constraint jacobian in the full matrix

        Integrator* itg;

        virtual void fill_containers() = 0;

};

#endif // CONSTRAINT_H_
