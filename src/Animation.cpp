#include "Animation.h"

#include <math.h>
#include <stdio.h>

Animation::Animation()
    : m_num_frames(0)
    , m_frames(NULL) {}

Animation::~Animation() {

    for (int i = 0; i < m_num_frames; i++) {
        delete m_frames[i];
    }

    delete[] m_frames;
}

void Animation::AddFrame(Skeleton* frame) {
    m_num_frames++;
    Skeleton** newframes = new Skeleton*[m_num_frames];

    for(int i = 0; i < m_num_frames-1; i++) {
        newframes[i] = m_frames[i];
    }

    newframes[m_num_frames-1] = frame->Copy();

    delete[] m_frames;
    m_frames = newframes;
}

Skeleton* Animation::GetFrame(int i) {
    return m_frames[i];
}

int Animation::NumFrames() {
    return m_num_frames;
}
