#ifndef MAGIC_MOTION_H_
#define MAGIC_MOTION_H_

#include "magic_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef V3 SpatialPoint;

typedef union
{
    uint32_t color;
    struct
    {
        uint8_t _padding, r, g, b;
    };
} ColorPoint;

void MagicMotion_Initialize(void);
void MagicMotion_Finalize(void);

void MagicMotion_CaptureFrame(void);

unsigned int MagicMotion_GetCloudSize(void);
SpatialPoint *MagicMotion_GetPositions(void);
Color *MagicMotion_GetColors(void);

#ifdef __cplusplus
}
#endif
#endif /* end of include guard: MAGIC_MOTION_H_ */

