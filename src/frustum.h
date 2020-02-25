#ifndef FRUSTUM_H_
#define FRUSTUM_H_

#include "magic_math.h"

typedef struct
{
    Mat4 transform;
    float fov;
    float aspect;
    float near_plane;
    float far_plane;
} Frustum;

#endif /* end of include guard: FRUSTUM_H_ */
