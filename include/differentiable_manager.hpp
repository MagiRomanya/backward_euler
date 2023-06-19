#ifndef DIFFERENTIABLE_MANAGER_H_
#define DIFFERENTIABLE_MANAGER_H_

#include <vector>
#include <iostream>

class DifferentiableManager {
    public:
        DifferentiableManager() {}
        ~DifferentiableManager() {
            // std::cout << "Number of differentiable parameters: " << parameters.size() << std::endl;
        }

        inline int add_parameter(double p) {
            int index = parameters.size();
            parameters.push_back(p);
            return index;
        }
        inline double get_parameter(size_t i) const { return parameters[i]; }
        inline void set_parameter(size_t i, double value) { parameters[i] = value; }
        inline int get_size() const { return parameters.size(); }
        inline void clear() { parameters.clear(); }

    private:
        std::vector<double> parameters;
};

#endif // DIFFERENTIABLE_MANAGER_H_
