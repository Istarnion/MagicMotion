#include "magic_motion.h" // Color, MagiMotionTag
#include <stdio.h>

#define MINIZ_NO_STDIO
#define MINIZ_NO_TIME
#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_ARCHIVE_WRITING_APIS
#define MINIZ_NO_ZLIB_APIS
#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#include "miniz.c"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VideoRecorder
{
    FILE *cloud_file;
    FILE *video_file;
    size_t frame_count;
} VideoRecorder;

#define MAX_RECORDERS 8
static VideoRecorder recorders[MAX_RECORDERS];

VideoRecorder *
StartVideoRecording(const char *cloud_file, const char *video_file)
{
    VideoRecorder *result = NULL;

    for(int i=0; i<MAX_RECORDERS; ++i)
    {
        VideoRecorder *recorder = &recorders[i];
        if(recorder->cloud_file == NULL || recorder->video_file == NULL)
        {
            result = recorder;
            result->frame_count = 0;
            result->cloud_file = fopen(cloud_file, "wb");
            result->video_file = fopen(video_file, "wb");
            if(result->cloud_file == NULL || ferror(result->cloud_file) ||
               result->video_file == NULL || ferror(result->video_file))
            {
                result->cloud_file = NULL;
                result->video_file = NULL;
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
    SDL_assert(recorder);

    fprintf(recorder->cloud_file, "%zu\n", recorder->frame_count);
    fprintf(recorder->video_file, "%zu\n", recorder->frame_count);
    fclose(recorder->cloud_file);
    fclose(recorder->video_file);
    recorder->cloud_file = NULL;
    recorder->video_file = NULL;
    recorder->frame_count = 0;
}

static void
CompressAndWriteData(FILE *f, const void *data, size_t size)
{
    size_t compressed_size;
    void *compressed_data = tdefl_compress_mem_to_heap(data, size, &compressed_size, 0);
    fwrite(&compressed_size, sizeof(size_t), 1, f);
    fwrite(compressed_data, 1, compressed_size, f);
}

void
WriteVideoFrame(VideoRecorder *recorder, size_t n_points, const V3 *xyz, const Color *rgb, const MagicMotionTag *tags)
{
    ++recorder->frame_count;
    fprintf(recorder->cloud_file, "frame %zu %zu\n", recorder->frame_count, n_points);
    CompressAndWriteData(recorder->cloud_file, xyz, n_points*sizeof(V3));
    CompressAndWriteData(recorder->cloud_file, rgb, n_points*sizeof(Color));
    CompressAndWriteData(recorder->cloud_file, tags, n_points*sizeof(MagicMotionTag));
    putc('\n', recorder->cloud_file);
}

void
AddVideoFrame(VideoRecorder *recorder, size_t color_w, size_t color_h, size_t depth_w, size_t depth_h, const Color *colors, const float *depths)
{
    fprintf(recorder->video_file, "frame %zu\n", recorder->frame_count);
    fprintf(recorder->video_file, "color %zux%zu\n", color_w, color_h);
    CompressAndWriteData(recorder->video_file, colors, color_w*color_h*sizeof(Color));
    fprintf(recorder->video_file, "depth %zux%zu\n", depth_w, depth_h);
    CompressAndWriteData(recorder->video_file, depths, depth_w*depth_h*sizeof(float));
    putc('\n', recorder->video_file);
}

#ifdef __cplusplus
} // extern "C"
#endif

