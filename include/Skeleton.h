#ifndef SKELETON_H
#define SKELETON_H

#pragma once

#include <string>

#include "Matrix.h"
#include "Vector.h"

struct Joint {
    int id;
    int parent_id;
    Vector3 position;
    Matrix_4x4 rotation;

    Joint();
    Joint(int id, int parent_id);
};

class Skeleton {

    public:
        Skeleton();
        ~Skeleton();
        
        int NumJoints();
        Joint GetJoint(int i);
        void SetJoint(int i, Joint j);
        
        Skeleton* Copy();

        Matrix_4x4 JointTransform(int i);

        Joint* m_joints;
        int m_num_joints;
        
};

#endif