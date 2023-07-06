#ifndef LENGTH_CONSTRAINT_H_
#define LENGTH_CONSTRAINT_H_

#include "integrator.hpp"
#include "constraint.hpp"
#include "vec3.hpp"

class LengthConstraint : public Constraint {
    public:
        LengthConstraint(Integrator* itg, vec3& p1, vec3& p2, int i1, int i2, vec3* com1 = nullptr, vec3* com2 = nullptr)
            : p1(p1), p2(p2), com1(com1), com2(com2)
        {
            this->i1 = i1;
            this->i2 = i2;
            this->itg = itg;
            itg->add_constraint(this);
        }

        void fill_containers() override;

    private:
        vec3& p1;
        vec3& p2;
        vec3* com1;
        vec3* com2;
        unsigned int i1;
        unsigned int i2;
};

#endif // LENGTH_CONSTRAINT_H_
