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
Mat4 *RendererGetViewMatrix(void);
Mat4 *RendererGetProjectionMatrix(void);

void *RendererCreateTexture(const void *pixels, int width, int height);
void RendererDestroyTexture(void *texture);

void RenderWireCube(V3 center, V3 size);
void RenderCube(V3 center, V3 size);
void RenderCubes(V3 *centers, V3 *colors, size_t num_cubes);
void RenderPointCloud(V3 *points, V3 *colors, size_t num_points);
void RenderFrustum(const Frustum *frustum);
void RenderFullscreenQuad(void);

#endif /* end of include guard: RENDERER_H_ */

