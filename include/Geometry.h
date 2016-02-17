#ifndef GEOMETRY_H
#define GEOMETRY_H

#pragma once

#include <string>

#include "Vector.h"

class Vertex {
    public:
        Vector3 position;
        Vector3 normal;
        
        Vector3 weight_ids;
        Vector3 weight_amounts;
        
        Vertex();
        Vertex(Vector3 pos);
        Vertex(Vector3 pos, Vector3 norm);
};

class Mesh {

    public:
        Mesh();
        ~Mesh();
        
        int NumVertices();
        int NumTriangles();
        
        int GetIndex(int i);
        Vertex GetVertex(int i);
        
        Vertex* m_vertices;
        int* m_triangles;
        
        int m_num_vertices;
        int m_num_triangles;
	
};

#endif