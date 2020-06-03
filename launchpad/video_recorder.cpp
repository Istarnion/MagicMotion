#include "magic_motion.h" // MagiMotionTag
#include "sensor_interface.h" // ColorPixel
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
StartVideoRecording(const char *cloud_file, const char *video_file, const size_t num_sensors, const SensorInfo *sensors)
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

    fprintf(result->video_file, "%zu sensors\n", num_sensors);
    for(int i=0; i<num_sensors; ++i)
    {
        const SensorInfo *s = &sensors[i];
        fprintf(result->video_file, "%s %s %s\n",
                s->vendor, s->name, s->serial);
        fprintf(result->video_file, "%d %d %f\n",
                s->color_stream_info.width, s->color_stream_info.height,
                s->color_stream_info.fov);
        fprintf(result->video_file, "%d %d %f %f %f\n",
                s->depth_stream_info.width, s->depth_stream_info.height,
                s->depth_stream_info.fov,
                s->depth_stream_info.min_depth,
                s->depth_stream_info.max_depth);
    }

    return result;
}

void
StopRecording(VideoRecorder *recorder)
{
    SDL_assert(recorder);

    printf("Writing %zu as the %zu last bytes\n", recorder->frame_count, sizeof(size_t));
    fwrite(&recorder->frame_count, sizeof(size_t), 1, recorder->cloud_file);
    fwrite(&recorder->frame_count, sizeof(size_t), 1, recorder->video_file);
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
    printf("Frame was compressed from %zu to %zu (%.02f%%)\n", size, compressed_size, ((float)compressed_size/(float)size)*100);
    fwrite(&compressed_size, sizeof(size_t), 1, f);
    fwrite(compressed_data, 1, compressed_size, f);
    free(compressed_data);
}

void
WriteVideoFrame(VideoRecorder *recorder, size_t n_points, const V3 *xyz, const ColorPixel *rgb, const MagicMotionTag *tags)
{
    ++recorder->frame_count;
    fprintf(recorder->cloud_file, "frame %zu %zu\n", recorder->frame_count, n_points);
    CompressAndWriteData(recorder->cloud_file, xyz, n_points*sizeof(V3));
    CompressAndWriteData(recorder->cloud_file, rgb, n_points*sizeof(ColorPixel));
    CompressAndWriteData(recorder->cloud_file, tags, n_points*sizeof(MagicMotionTag));
    putc('\n', recorder->cloud_file);
}

void
AddVideoFrame(VideoRecorder *recorder, size_t color_w, size_t color_h, size_t depth_w, size_t depth_h, const ColorPixel *colors, const float *depths)
{
    fprintf(recorder->video_file, "frame %zu\n", recorder->frame_count);
    fprintf(recorder->video_file, "color\n");
    CompressAndWriteData(recorder->video_file, colors, color_w*color_h*sizeof(ColorPixel));
    fprintf(recorder->video_file, "\ndepth\n");
    CompressAndWriteData(recorder->video_file, depths, depth_w*depth_h*sizeof(float));
    putc('\n', recorder->video_file);
}

#ifdef __cplusplus
} // extern "C"
#endif

