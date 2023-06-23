#ifndef DIFFERENTIABLE_MANAGER_H_
#define DIFFERENTIABLE_MANAGER_H_

#include <vector>
#include <iostream>

class DifferentiableManager {
    /* Class which stores and handles differentiable parameters only. */
    public:
        ~DifferentiableManager();

        int add_parameter(double p);

        inline double get_parameter(size_t i) const { return parameters[i]; }
        inline void set_parameter(size_t i, double value) { parameters[i] = value; }
        inline int get_size() const { return parameters.size(); }
        inline void clear() { parameters.clear(); }

        void print();

        inline std::vector<double> get_parameters() { return parameters; }

    private:
        std::vector<double> parameters;
};

#endif // DIFFERENTIABLE_MANAGER_H_
