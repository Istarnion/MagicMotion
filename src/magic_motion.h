#ifndef MAGIC_MOTION_H_
#define MAGIC_MOTION_H_

#include "magic_math.h"
#include "frustum.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// This is quite arbitrary, but kept low to save memory and startup time
#define MAX_SENSORS 4

typedef enum
{
    TAG_CAMERA_0,
    TAG_CAMERA_1,
    TAG_CAMERA_2,
    TAG_CAMERA_3
} MagicMotionTag;

typedef union
{
    uint32_t color;
    struct
    {
        uint8_t _padding, r, g, b;
    };
} Color;

typedef struct
{
    int hitbox;
    bool enter;
} MagicMotionHitboxEvent;

void MagicMotion_Initialize(void);
void MagicMotion_Finalize(void);

unsigned int MagicMotion_GetNumCameras(void);
const Frustum *MagicMotion_GetCameraFrustums(void);
void MagicMotion_SetCameraPosition(unsigned int camera_index, V3 position);
void MagicMotion_SetCameraRotation(unsigned int camera_index, float pitch, float yaw, float roll);

void MagicMotion_CaptureFrame(void);

void MagicMotion_GetColorImageResolution(unsigned int camera_index, int *width, int *height);
void MagicMotion_GetDepthImageResolution(unsigned int camera_index, int *width, int *height);

const Color *MagicMotion_GetColorImage(unsigned int camera_index);
const float *MagicMotion_GetDepthImage(unsigned int camera_index);

unsigned int MagicMotion_GetCloudSize(void);
V3 *MagicMotion_GetPositions(void);
Color *MagicMotion_GetColors(void);
MagicMotionTag *MagicMotion_GetTags(void);

int MagicMotion_RegisterHitbox(V3 pos, V3 size);
void MagicMotion_UpdateHitbox(int hitbox, V3 pos, V3 size);
MagicMotionHitboxEvent *MagicMotion_QueryHitboxes(int *num_events);

#ifdef __cplusplus
}
#endif
#endif /* end of include guard: MAGIC_MOTION_H_ */
