#include "camera.h"

Mat4
CameraGetViewMatrix(Camera *cam)
{
    V3 target = AddV3(cam->position, CameraForward(cam));
    Mat4 result = LookAtMat4(cam->position, target, (V3){ 0, 1, 0 });
    return result;
}

V3
CameraForward(Camera *cam)
{
    Mat4 rotation = RotateMat4(cam->pitch, cam->yaw, 0);
    V3 result = (V3){ rotation.f02, rotation.f12, rotation.f22 };
    return result;
}

void
MoveCamera(Camera *cam, V3 v)
{
    V3 p = cam->position;
    V3 f = CameraForward(cam);
    V3 u = (V3){ 0, 1, 0 };
    V3 r = CrossV3(f, u);

    V3 parts[] = {
        ScaleV3(r, v.x),
        ScaleV3(u, v.y),
        ScaleV3(f, v.z)
    };

    cam->position = AddV3(p, SumV3(parts, 3));
}

void
RotateCamera(Camera *cam, float y, float p)
{
    cam->pitch = Clamp(cam->pitch + p, M_PI/-2.0f, M_PI/2.0f);
    cam->yaw += y;
}

void
CameraLookAt(Camera *cam, V3 target)
{
    // TODO(istarnion): Implement!
}

