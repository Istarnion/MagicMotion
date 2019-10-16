#include "camera.h"

Mat4
CameraGetViewMatrix(Camera *cam)
{
    //Mat4 result = LookAtMat4(cam->position, AddV3(cam->position, cam->forward), cam->up);
    Mat4 result = TranslationMat4(NegateV3(cam->position));
    return result;
}

void
MoveCamera(Camera *cam, V3 v)
{
    V3 p = cam->position;
    V3 parts[] = {
        ScaleV3(cam->right, v.x),
        ScaleV3(cam->up, v.y),
        ScaleV3(cam->forward, v.z)
    };

    cam->position = AddV3(p, SumV3(parts, 3));
}

void
RotateCamera(Camera *cam, float y, float p)
{
    cam->pitch = Clamp(cam->pitch+p, -RADIANS(89.0f), RADIANS(89.0f));
    cam->yaw += y;

    V3 front;
    front.x = cos(cam->pitch) * cos(cam->yaw);
    front.y = sin(cam->pitch);
    front.z = cos(cam->pitch) * sin(cam->yaw);
    cam->forward = NormalizeV3(SubV3(AddV3(front, cam->position), cam->position));

    cam->right = CrossV3(cam->forward, cam->up);
}

void
CameraLookAt(Camera *cam, V3 target)
{
    cam->forward = NormalizeV3(SubV3(target, cam->position));
    cam->right = CrossV3(cam->forward, cam->up);
    cam->up = CrossV3(cam->right, cam->forward);
}

