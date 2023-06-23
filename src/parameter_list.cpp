#include "parameter_list.hpp"
#include <vector>

double ParameterList::operator[](int index) const {
    if (index >= parameters.size()) {
        std::cerr << "ERROR::PARAMETER_LIST: Trying to access parameter " << index << " out of bounds" << std::endl;
        exit(-1);
    }
    return parameters[index].getValue();
}

void ParameterList::addParameter(DifferentiableManager *diff, double value) {
    parameters.push_back(Parameter(diff, value));
    total_parameters++;
    diff_parameters++;
}

void ParameterList::addParameter(double value) {
    parameters.push_back(Parameter(value));
    total_parameters++;
    non_diff_parameters++;
}


std::vector<size_t> ParameterList::getIndexVector() const {
    std::vector<size_t> indices;
    for (int i = 0; i < parameters.size(); i++) {
        if (parameters[i].isDiff())
            indices.push_back(parameters[i].getIndex());
    }
    return indices;
}
