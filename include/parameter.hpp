#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <cstddef>
#include <differentiable_manager.hpp>
#include <iostream>

class Parameter {
    public:
        /* Constructor for differentiable parameters. */
        Parameter(DifferentiableManager* diff, double value) {
            isDifferentiable = true;
            this->diff = diff;
            this->value = value;
            index = diff->add_parameter(value);
        }

        /* Constructor for non differentiable paramters. */
        Parameter(double value) {
            isDifferentiable = false;
            this->value = value;
        }

        inline double getValue() const {
            if (isDifferentiable)
                return diff->get_parameter(index);
            return value;
        }

        inline bool isDiff() const { return isDifferentiable; }

        inline size_t getIndex() const {
            if (isDifferentiable)
                return index;
            std::cerr << "ERROR::PARAMETER::GETINDEX: Tried to access non differentiable index." << std::endl;
            exit(-1);
        }

        inline void update(double value) {
            this->value = value;
            if (!isDifferentiable) return;
            diff->set_parameter(index, value);
        }

    private:
        bool isDifferentiable;
        size_t index;
        double value;
        DifferentiableManager* diff;
};

#endif // PARAMETER_H_
