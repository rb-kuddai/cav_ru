#include <stdlib.h>
#include <assert.h>

#include "Geometry.h"

Vertex::Vertex() {}
Vertex::Vertex( Vector3 pos )
    : position(pos)
    , normal(Vector3::Zero()) {}

Vertex::Vertex( Vector3 pos, Vector3 norm )
    : position(pos)
    , normal(norm) {}

Mesh::Mesh()
    : m_vertices(NULL)
    , m_triangles(NULL)
    , m_num_vertices(0)
    , m_num_triangles(0) {}

Mesh::~Mesh() {
    delete[] m_vertices;
    delete[] m_triangles;
}

int Mesh::NumVertices() {
    return m_num_vertices;
}

int Mesh::NumTriangles() {
    return m_num_triangles;
}

int Mesh::GetIndex(int i) {
    return m_triangles[i];
}

Vertex Mesh::GetVertex(int i) {
    return m_vertices[i];
}
