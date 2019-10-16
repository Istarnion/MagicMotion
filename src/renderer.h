#ifndef RENDERER_H_
#define RENDERER_H_

#include "frustum.h"

void RendererInit(const char *title, int width, int height);
void RendererQuit(void);
void RendererClear(void);
void RendererDisplay(void);
void RendererUpdateSize(void);
void RendererGetSize(int *width, int *height);
void RendererSetViewMatrix(Mat4 v);
void RendererSetProjectionMatrix(Mat4 p);

void RenderCube(V3 center, V3 size);
void RenderFrustum(const Frustum *frustum);

#endif /* end of include guard: RENDERER_H_ */
