#ifndef EDGE_H
#define EDGE_H

class Edge{
    public:
        int a, b;

        int opposite;
        // Constructor
        Edge(int a, int b, int opposite)
            : a(a), b(b), opposite(opposite) {}

        Edge reversed() { return Edge(b, a, -1); }
};

bool operator==(const Edge& e1, const Edge& e2);

#endif
