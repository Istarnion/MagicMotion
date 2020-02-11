#ifndef FRUSTUM_H_
#define FRUSTUM_H_

#include "magic_math.h"

typedef struct
{
    V3 position;
    float pitch, yaw, roll;
    float fov;
    float aspect;
    float near_plane;
    float far_plane;
} Frustum;

#endif /* end of include guard: FRUSTUM_H_ */
