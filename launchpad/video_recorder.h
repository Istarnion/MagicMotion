#ifndef VIDEO_RECORDER_H_
#define VIDEO_RECORDER_H_

#include "magic_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VideoRecorder VideoRecorder;

VideoRecorder *StartVideoRecording(const char *file);
void StopRecording(VideoRecorder *recorder);
void WriteVideoFrame(VideoRecorder *recorder, size_t n_points, V3 *xyz, Color *rgb);

#ifdef __cplusplus
} // extern "C"
#endif
#endif /* end of include guard: VIDEO_RECORDER_H_ */

