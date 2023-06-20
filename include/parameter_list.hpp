#ifndef PARAMETER_LIST_H_
#define PARAMETER_LIST_H_

#include <vector>
#include <parameter.hpp>

class ParameterList {
    /* Simple class to handle a set of paramters.
     * It is useful to manage differentiable and non differentiable paramters at the same time,
     * making it seemless to work with either kind. */

    public:
        void addParameter(DifferentiableManager* diff, double value);

        void addParameter(double value);

        std::vector<size_t> getIndexVector() const;

        inline unsigned int getTotalParameters() { return total_parameters; }
        inline unsigned int getDiffParameters() { return diff_parameters; }
        inline unsigned int getNonDiffParameters() { return non_diff_parameters; }
        inline void updateParameter(size_t index, double value) { parameters.at(index).update(value); }

        double operator[](int index) const;

    private:
        std::vector<Parameter> parameters;

        unsigned int total_parameters = 0;
        unsigned int diff_parameters = 0;
        unsigned int non_diff_parameters = 0;
};

#endif // PARAMETER_LIST_H_
