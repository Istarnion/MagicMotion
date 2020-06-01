#ifndef VIDEO_RECORDER_H_
#define VIDEO_RECORDER_H_

#include "magic_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VideoRecorder VideoRecorder;

VideoRecorder *StartVideoRecording(const char *cloud_file, const char *video_file);
void StopRecording(VideoRecorder *recorder);
void WriteCloudFrame(VideoRecorder *recorder, size_t n_points, const V3 *xyz, const Color *rgb, const MagicMotionTag *tags);
void AddVideoFrame(VideoRecorder *recorder, size_t color_w, size_t color_h, size_t depth_w, size_t depth_h, const Color *colors, const float *depths);

#ifdef __cplusplus
} // extern "C"
#endif
#endif /* end of include guard: VIDEO_RECORDER_H_ */

