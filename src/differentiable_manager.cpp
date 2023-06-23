#include "differentiable_manager.hpp"

DifferentiableManager::~DifferentiableManager() {
    // print();
    // std::cout << "Number of differentiable parameters: " << parameters.size() << std::endl;
}


int DifferentiableManager::add_parameter(double p) {
    int index = parameters.size();
    parameters.push_back(p);
    return index;
}

void DifferentiableManager::print() {
    std::cout << parameters[0];
    for (int i = 1; i < parameters.size(); i++) {
        std::cout << ", " << parameters[i];
    }
    std::cout << std::endl;
}
