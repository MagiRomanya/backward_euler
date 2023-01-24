#include <iostream>
#include "mesh.h"
#include <vector>


int main(int argc, char *argv[]) {

    SimpleMesh m;

    CreateGrid(m, 2, 2, 1);

    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    m.boundary(internalEdges, externalEdges);

    std::cout << "Internal edges: " << std::endl;
    for (int i = 0; i < internalEdges.size(); i+=2){
        Edge& e1 = internalEdges[i];
        Edge& e2 = internalEdges[i+1];
        std::cout << e1.a <<", "<< e1.b<<", "  << e1.opposite << "\t\t"<< e2.a <<", " << e2.b <<", " << e2.opposite << std::endl;
    }


    std::cout << "External edges: " << std::endl;
    for (int i = 0; i < externalEdges.size(); i++){
        Edge& e = externalEdges[i];
        std::cout << e.a << ", " << e.b << std::endl;
    }

    std::cout << "Internal edges size: " << internalEdges.size() << std::endl;
    std::cout << "External edges size: " << externalEdges.size() << std::endl;
    std::cout << "Total size: " << externalEdges.size() + internalEdges.size() << std::endl;

    return 0;
}
