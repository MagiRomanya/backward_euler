#include "spring_counter.hpp"
#include "pysimulation.hpp"
#include "mesh.h"

std::vector<int> count_spring(SimpleMesh& mesh) {
    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    mesh.boundary(internalEdges, externalEdges);

    int nFlex = internalEdges.size() / 2.0 + externalEdges.size();
    int nBend = internalEdges.size() / 2.0;

    return {nFlex, nBend};
}

std::vector<int> count_spring() {
    SimpleMesh mesh;
    CreateGrid(mesh, N, M, 1);
    return count_spring(mesh);
}
