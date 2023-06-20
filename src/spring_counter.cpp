#include "spring_counter.hpp"

std::vector<int> count_spring(const SimpleMesh& mesh) {
    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    mesh.boundary(internalEdges, externalEdges);

    int nFlex = internalEdges.size() + externalEdges.size();
    int nBend = internalEdges.size();

    return {nFlex, nBend};
}
