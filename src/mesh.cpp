#include "mesh.h"
#include <unordered_map>
#include "math.h"

// Have a way to hash the Edge class
namespace std{
  template<>
  struct hash<Edge>{
    unsigned int operator()(const Edge& key) const{
      return 100000 * key.a + key.b;
    }
  };
}

bool operator==(const Edge& e1, const Edge& e2){
    return (e1.a == e2.a) and (e1.b == e2.b);
}

void SimpleMesh::boundary(std::vector<Edge> &internalEdges, std::vector<Edge> &externalEdges) const {
    /* Fills the internal edges and external edges vectors. In the internal edges vectors, we order the vector
     * by puting a semi-edge and its inverse next to each other. */

    std::vector<Edge> edges;
    edges.reserve(2 * 3 * triangles.size()); // In the worst case scenario all the mesh will be boudary
    std::unordered_map<Edge, int> in_vector;

    unsigned int externalSize = 0;
    unsigned int internalSize = 0;

    for (size_t i=0; i < triangles.size(); i++){
        const Triangle &t = triangles[i];

        // Edge a, b
        // If the inverse edge has already been added, it adds the current edge next to it
        Edge e = Edge(t.a, t.b, t.c);
        if (in_vector.count(e.reversed())){
            int inverse_edge_index = in_vector[e.reversed()];
            edges[inverse_edge_index + 1] = e;
            internalSize+=2;
        }
        else{
            in_vector[e] = edges.size();
            edges.push_back(e);
            edges.push_back(Edge(-1,-1,-1)); // Add a dummy edge to the list for later removal if it's not overwritten
        }

        // Edge c, a
        e = Edge(t.c, t.a, t.b);
        if (in_vector.count(e.reversed())){
            int inverse_edge_index = in_vector[e.reversed()];
            edges[inverse_edge_index + 1] = e;
            internalSize+=2;
        }
        else{
            in_vector[e] = edges.size();
            edges.push_back(e);
            edges.push_back(Edge(-1,-1,-1));
        }

        // Edge b, c
        e = Edge(t.b, t.c, t.a);
        if (in_vector.count(e.reversed())){
            int inverse_edge_index = in_vector[e.reversed()];
            edges[inverse_edge_index + 1] = e;
            internalSize+=2;
        }
        else{
            in_vector[e] = edges.size();
            edges.push_back(e);
            edges.push_back(Edge(-1,-1,-1));
        }
    }

    externalSize = 3 * triangles.size() - internalSize;

    externalEdges.reserve(externalSize);
    internalEdges.reserve(internalSize);

    // Now we have to remove the dummy edges created by the edge of the mesh where no inverse exist ( the boundary )
    for (size_t i=0; i < edges.size(); i+=2){
        if (edges[i+1].a < 0){ // If the 2nd edge is dummy we have found a external edge with no inverse
            externalEdges.push_back(edges[i]);
        }
        else{
            internalEdges.push_back(edges[i]);
            internalEdges.push_back(edges[i+1]);
        }
    }
}

void CreateGrid(SimpleMesh &m, int nY, int nZ, double step){
    /* Creates a grid mesh in the x plane */

    m.vertices.resize(nY * nZ);
    m.triangles.reserve(2*(nY-1)*(nZ-1));
    // First create the vertices of the mesh
    for (unsigned int j = 0; j < nY; j++){
        for (unsigned int k = 0; k < nZ; k++){
            Vertex v;
            // v.Position = glm::vec3(0.0, step * j, step * k);
            // NOTE uncomment to make the cloth fall from flat to vertical
            v.Position = glm::vec3(step*k, step * j, 0.0);
            m.vertices[nZ*j + k] = v;
        }
    }

    // Next we triangulate the mesh
    for (unsigned int j = 0; j < nY-1; j++){
        for (unsigned int k = 0; k < nZ-1; k++){
            // 2 triangles per cell
            Triangle t1;
            Triangle t2;
            t1.a = nZ*j + k;
            t1.b = nZ*j + (k+1);
            t1.c = nZ*(j+1) + k;
            m.triangles.push_back(t1);

            t2.a = nZ*(j+1) + (k+1);
            t2.b = nZ*(j+1) + k;
            t2.c = nZ*j + (k+1);
            m.triangles.push_back(t2);
        }
    }
}

double SimpleMesh::distance2(int i, int j) const {
    glm::vec3 d = vertices[i].Position - vertices[j].Position;
    return d.x*d.x + d.y*d.y + d.z*d.z;
}

double SimpleMesh::distance(int i, int j) const {
    return sqrt(distance2(i, j));
}
