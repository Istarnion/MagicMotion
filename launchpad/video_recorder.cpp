#include "magic_motion.h" // Color
#include <stdio.h>

#ifdef __cplusplus__
extern "C" {
#endif

typedef struct VideoRecorder
{
    FILE *file;
    size_t frame_count;
} VideoRecorder;

#define MAX_RECORDERS 8
static VideoRecorder recorders[MAX_RECORDERS];

VideoRecorder *
StartVideoRecording(const char *file)
{
    VideoRecorder *result = NULL;

    for(int i=0; i<MAX_RECORDERS; ++i)
    {
        VideoRecorder *recorder = &recorders[i];
        if(recorder->file == NULL)
        {
            result = recorder;
            result->frame_count = 0;
            result->file = fopen(file, "w");
            if(result->file == NULL || ferror(result->file))
            {
                result->file = NULL;
                result = NULL;
            }

            break;
        }
    }

    return result;
}

void
StopRecording(VideoRecorder *recorder)
{
    fprintf(recorder->file, "nframes %d\n", recorder->frame_count);
    fclose(recorder->file);
    recorder->file = NULL;
    recorder->frame_count = 0;
}

void
WriteVideoFrame(VideoRecorder *recorder, size_t n_points, V3 *xyz, Color *rgb)
{
    ++recorder->frame_count;
    fprintf(recorder->file, "npoints %d\n", n_points);
    for(size_t i=0; i<n_points; ++i)
    {
        fprintf(recorder->file, "%f %f %f #%06X\n",
                n_points, xyz.x, xyz.y, xyz.z, rgb.color);
    }
}

#ifdef __cplusplus__
} // extern "C"
#endif

