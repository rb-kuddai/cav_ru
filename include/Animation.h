#ifndef ANIMATION_H
#define ANIMATION_H

#include "Skeleton.h"

class Animation {

    public:
        Animation();
        ~Animation();
        
        void AddFrame(Skeleton* frame);
        Skeleton* GetFrame(int i);
        int NumFrames();
        
        private:
        
        int m_num_frames;
        Skeleton** m_frames;
};

#endif