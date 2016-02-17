#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>

#include <math.h>
#include <stdlib.h>

#include "Skeleton.h"

Joint::Joint()
    : id(0)
    , parent_id(-1)
    , position(Vector3::Zero())
    , rotation(Matrix_4x4::Id()) {}

Joint::Joint(int id, int parent_id)
    : id(id)
    , parent_id(parent_id) {}

Skeleton::Skeleton()
    : m_joints(NULL)
    , m_num_joints(0) {}

Skeleton::~Skeleton() {
    delete[] m_joints;
}

int Skeleton::NumJoints() {
    return m_num_joints;
}

Joint Skeleton::GetJoint(int i) {
    return m_joints[i];
}

void Skeleton::SetJoint(int i, Joint j) {
    m_joints[i] = j;
}

/*
** TODO: Implement. This method must return the global transform of a joint
*/
Matrix_4x4 Skeleton::JointTransform(int i) {
	Joint joint = this->m_joints;
}

Skeleton* Skeleton::Copy() {

    Skeleton* copy = new Skeleton();
    copy->m_num_joints = m_num_joints;
    copy->m_joints = new Joint[copy->m_num_joints];

    for(int i = 0; i < copy->m_num_joints; i++) {
        copy->SetJoint(i, m_joints[i]);
    }

    return copy;
}
