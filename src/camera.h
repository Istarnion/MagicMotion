#ifndef CAMERA_H_
#define CAMERA_H_

/*
 * This is a VIRTUAL camera, for viewing the scene, NOT a representation
 * of a real physical camera.
 */

#include "magic_math.h"

typedef struct
{
    V3 position;
    float pitch, yaw;
} Camera;

Mat4 CameraGetViewMatrix();

V3 CameraForward(Camera *cam);
void MoveCamera(Camera *cam, V3 v); // Move relative to the cameras local space
void RotateCamera(Camera *cam, float yaw, float pitch);
void CameraLookAt(Camera *cam, V3 target);

#endif /* end of include guard: CAMERA_H_ */

