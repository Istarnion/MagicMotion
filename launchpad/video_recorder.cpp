#include "magic_motion.h" // Color
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
            result->file = fopen(file, "wb");
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
    fwrite(&recorder->frame_count, sizeof(size_t), 1);
    fclose(recorder->file);
    recorder->file = NULL;
    recorder->frame_count = 0;
}

static void
CompressAndWriteData(FILE *f, void *data, size_t size)
{
    size_t compressed_size;
    void *compressed_data = tdefl_compress_mem_to_heap(data, size, &compressed_size, 0);
    fwrite(&compressed_size, sizeof(size_t), 1);
    fwrite(compressed_data, 1, compressed_size);
}

void
WriteVideoFrame(VideoRecorder *recorder, size_t n_points, V3 *xyz, Color *rgb)
{
    ++recorder->frame_count;
    CompressAndWriteData(recorder->file, xyz, n_points*sizeof(V3));
    CompressAndWriteData(recorder->file, rgb, n_points*sizeof(Color));
}

#ifdef __cplusplus__
} // extern "C"
#endif

