#include "magic_motion.h" // MagiMotionTag
#include "sensor_interface.h" // ColorPixel
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>

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

typedef struct
{
    void *data;
    size_t n_bytes;
    FILE *fd;
} QueuedBuffer;

#define QUEUE_LENGTH 1024
#define N_CONSUMERS 1

typedef struct VideoRecorder
{
    FILE *cloud_file;
    FILE *video_file;
    size_t frame_count;

    volatile bool running;

    QueuedBuffer buffer_queue[QUEUE_LENGTH];
    size_t producer_index;
    size_t consumer_index;
    size_t buffer_count;
    pthread_mutex_t lock;
    pthread_mutex_t file_lock;
    sem_t *empty;
    sem_t *full;

    pthread_t consumers[N_CONSUMERS];
} VideoRecorder;

// Why support multiple recorders?
#define MAX_RECORDERS 4
static VideoRecorder recorders[MAX_RECORDERS];

static void *
_ConsumerThread(void *userdata)
{
    VideoRecorder *recorder = (VideoRecorder *)userdata;

    struct timespec waittime = {0};
    waittime.tv_sec = 1;
    while(recorder->running)
    {
        if(sem_timedwait(recorder->full, &waittime))
        {
            continue;
        }

        pthread_mutex_lock(&recorder->lock);

        QueuedBuffer buffer = recorder->buffer_queue[recorder->consumer_index];

        ++recorder->consumer_index;
        if(recorder->consumer_index >= QUEUE_LENGTH)
        {
            recorder->consumer_index = 0;
        }

        --recorder->buffer_count;

        pthread_mutex_unlock(&recorder->lock);
        sem_post(recorder->empty);

        pthread_mutex_lock(&recorder->file_lock);
        fwrite(buffer.data, 1, buffer.n_bytes, buffer.fd);
        pthread_mutex_unlock(&recorder->file_lock);
        free(buffer.data);
    }

    return NULL;
}

void
_WriteBuffer(VideoRecorder *recorder, const void *data, size_t n_bytes, FILE *fd)
{
    QueuedBuffer buffer = {0};
    {
        buffer.data = malloc(n_bytes);
        memcpy(buffer.data, data, n_bytes);
        buffer.n_bytes = n_bytes;
        buffer.fd = fd;
    }

    sem_wait(recorder->empty);
    pthread_mutex_lock(&recorder->lock);

    recorder->buffer_queue[recorder->producer_index] = buffer;
    ++recorder->producer_index;
    if(recorder->producer_index >= QUEUE_LENGTH)
    {
        recorder->producer_index = 0;
    }

    ++recorder->buffer_count;

    pthread_mutex_unlock(&recorder->lock);
    sem_post(recorder->full);
}

void
_WriteString(VideoRecorder *recorder, const char *s, FILE *fd)
{
    size_t len = strlen(s);
    _WriteBuffer(recorder, s, len, fd);
}

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
            memset(result, 0, sizeof(VideoRecorder));
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

    result->running = true;

    pthread_mutex_init(&result->lock, NULL);
    pthread_mutex_init(&result->file_lock, NULL);
    result->full = sem_open("full", O_CREAT, 0600, 0);
    result->empty = sem_open("empty", O_CREAT, 0600, QUEUE_LENGTH);

    for(int i=0; i<N_CONSUMERS; ++i)
    {
        pthread_create(&result->consumers[i], NULL, _ConsumerThread, result);
    }

    char header[1024] = {0};
    size_t header_offset = 0;
    sprintf(header, "%zu sensors\n", num_sensors);
    header_offset = strlen(header);
    for(int i=0; i<num_sensors; ++i)
    {
        const SensorInfo *s = &sensors[i];
        sprintf(header+header_offset, "%s %s %s\n",
                s->vendor, s->name, s->serial);
        header_offset = strlen(header);
        sprintf(header+header_offset, "%d %d %f\n",
                s->color_stream_info.width, s->color_stream_info.height,
                s->color_stream_info.fov);
        header_offset = strlen(header);
        sprintf(header+header_offset, "%d %d %f %f %f\n",
                s->depth_stream_info.width, s->depth_stream_info.height,
                s->depth_stream_info.fov,
                s->depth_stream_info.min_depth,
                s->depth_stream_info.max_depth);
        header_offset = strlen(header);
    }

    _WriteString(result, header, result->video_file);

    return result;
}

void
StopRecording(VideoRecorder *recorder)
{
    SDL_assert(recorder);

    printf("Writing %zu as the %zu last bytes\n", recorder->frame_count, sizeof(size_t));
    fwrite(&recorder->frame_count, sizeof(size_t), 1, recorder->cloud_file);
    fwrite(&recorder->frame_count, sizeof(size_t), 1, recorder->video_file);

    recorder->running = false;
    for(int i=0; i<N_CONSUMERS; ++i)
    {
        pthread_join(recorder->consumers[i], NULL);
    }

    sem_close(recorder->full);
    sem_close(recorder->empty);
    pthread_mutex_destroy(&recorder->lock);

    fclose(recorder->cloud_file);
    fclose(recorder->video_file);

    memset(recorder, 0, sizeof(VideoRecorder));
}

static void
CompressAndWriteData(VideoRecorder *recorder, FILE *f, const void *data, size_t size)
{
    size_t compressed_size;
    void *compressed_data = tdefl_compress_mem_to_heap(data, size, &compressed_size, 0);
    printf("Frame was compressed from %zu to %zu (%.02f%%)\n", size, compressed_size, ((float)compressed_size/(float)size)*100);
    _WriteBuffer(recorder, &compressed_size, sizeof(size_t), f);
    _WriteBuffer(recorder, compressed_data, compressed_size, f);
    free(compressed_data);
}

void
WriteVideoFrame(VideoRecorder *recorder, size_t n_points, const V3 *xyz, const ColorPixel *rgb, const MagicMotionTag *tags)
{
    ++recorder->frame_count;
    char header[128] = {0};
    sprintf(header, "frame %zu %zu\n", recorder->frame_count, n_points);
    _WriteString(recorder, header, recorder->cloud_file);
    CompressAndWriteData(recorder, recorder->cloud_file, xyz, n_points*sizeof(V3));
    CompressAndWriteData(recorder, recorder->cloud_file, rgb, n_points*sizeof(ColorPixel));
    CompressAndWriteData(recorder, recorder->cloud_file, tags, n_points*sizeof(MagicMotionTag));
    _WriteString(recorder, "\n", recorder->cloud_file);
}

void
AddVideoFrame(VideoRecorder *recorder, size_t color_w, size_t color_h, size_t depth_w, size_t depth_h, const ColorPixel *colors, const float *depths)
{
    char header[128] = {0};
    sprintf(header, "frame %zu\ncolor\n", recorder->frame_count);
    _WriteString(recorder, header, recorder->video_file);
    CompressAndWriteData(recorder, recorder->video_file, colors, color_w*color_h*sizeof(ColorPixel));
    memset(header, 0, 128);
    sprintf(header, "\ndepth\n");
    _WriteString(recorder, header, recorder->video_file);
    CompressAndWriteData(recorder, recorder->video_file, depths, depth_w*depth_h*sizeof(float));
    _WriteString(recorder, "\n", recorder->video_file);
}

#ifdef __cplusplus
} // extern "C"
#endif

