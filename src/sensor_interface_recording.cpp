#include "sensor_interface.h"

#include "utils.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define MINIZ_NO_STDIO
#define MINIZ_NO_TIME
#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_ARCHIVE_WRITING_APIS
#define MINIZ_NO_ZLIB_APIS
#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#include "miniz.c"

struct _sensor;

typedef struct _sensor
{
    ColorPixel *color_frame;
    DepthPixel *depth_frame;

    size_t *color_frame_offsets;
    size_t *depth_frame_offsets;
} Sensor;

static struct
{
    FILE *video_file;
    size_t num_sensors;
    size_t frame_index;
    size_t num_frames;

    Sensor sensors[8];
    SensorInfo sensor_infos[8];
} _interface;

void
InitializeSensorInterface(void)
{
    puts("Initializing the Recording Interface..");

    // TODO(istarnion): Don't hard code this. But don't change the signature of the function neither
    _interface.video_file = fopen("test_recording_video.vid", "rb");
    assert(_interface.video_file);

    // Find the number of frames (the last four bytes)
    _interface.num_frames = 0;
    fseek(_interface.video_file, -sizeof(size_t), SEEK_END);
    fread(&_interface.num_frames, sizeof(size_t), 1, _interface.video_file);

    printf("Num frames: %zu\n", _interface.num_frames);
    assert(_interface.num_frames < 20000);

    rewind(_interface.video_file);

    // Find number of sensors
    fscanf(_interface.video_file, "%zu sensors\n", &_interface.num_sensors);
    printf("Num sensors: %zu\n", _interface.num_sensors);
    assert(_interface.num_sensors <= 8);

    for(int i=0; i<_interface.num_sensors; ++i)
    {
        SensorInfo *info = &_interface.sensor_infos[i];
        info->sensor_data = &_interface.sensors[i];
        strncpy(info->URI, "REC", 128);
        fscanf(_interface.video_file, "%s %s %s\n",
                info->vendor, info->name, info->serial);
        fscanf(_interface.video_file, "%d %d %f\n",
                &info->color_stream_info.width, &info->color_stream_info.height,
                &info->color_stream_info.fov);
        fscanf(_interface.video_file, "%d %d %f %f %f\n",
                &info->depth_stream_info.width, &info->depth_stream_info.height,
                &info->depth_stream_info.fov,
                &info->depth_stream_info.min_depth, &info->depth_stream_info.max_depth);

        info->color_stream_info.aspect_ratio = (float)info->color_stream_info.width /
                                               (float)info->color_stream_info.height;
        info->depth_stream_info.aspect_ratio = (float)info->depth_stream_info.width /
                                               (float)info->depth_stream_info.height;

        Sensor *sensor = &_interface.sensors[i];
        sensor->color_frame = (ColorPixel *)calloc(info->color_stream_info.width*info->color_stream_info.height, sizeof(ColorPixel));
        sensor->depth_frame = (DepthPixel *)calloc(info->depth_stream_info.width*info->depth_stream_info.height, sizeof(DepthPixel));

        sensor->color_frame_offsets = (size_t *)calloc(_interface.num_frames, sizeof(size_t));
        sensor->depth_frame_offsets = (size_t *)calloc(_interface.num_frames, sizeof(size_t));

        printf("%s %s (%s):\n\tColor: %dx%d, fov: %f\n\tDepth: %dx%d, fov: %f, min: %f, max: %f\n",
               info->vendor, info->name, info->serial,
               info->color_stream_info.width, info->color_stream_info.height, info->color_stream_info.fov,
               info->depth_stream_info.width, info->depth_stream_info.height, info->depth_stream_info.fov,
               info->depth_stream_info.min_depth, info->depth_stream_info.max_depth);
    }

    for(int i=0; i<_interface.num_frames; ++i)
    {
        for(int j=0; j<_interface.num_sensors; ++j)
        {
            Sensor *sensor = &_interface.sensors[j];

            size_t frame_index;
            fscanf(_interface.video_file, "frame %zu\n", &frame_index);
            assert(frame_index == (i+1));
            char frame_type[64] = {0};
            fgets(frame_type, 64, _interface.video_file);
            assert(strcmp(frame_type, "color\n") == 0);
            sensor->color_frame_offsets[i] = ftell(_interface.video_file);

            size_t compressed_size = 0;
            fread(&compressed_size, sizeof(size_t), 1, _interface.video_file);
            fseek(_interface.video_file, compressed_size+1, SEEK_CUR); // Skip compressed data and following newline

            memset(frame_type, 0, 64);
            fgets(frame_type, 64, _interface.video_file);
            assert(strcmp(frame_type, "depth\n") == 0);
            sensor->depth_frame_offsets[i] = ftell(_interface.video_file);
            fread(&compressed_size, sizeof(size_t), 1, _interface.video_file);
            fseek(_interface.video_file, compressed_size+1, SEEK_CUR); // Skip compressed data and following newline
        }
    }

    puts("Done.");
}

void
FinalizeSensorInterface(void)
{
    puts("Shutting down the Recording Interface.");

    for(int i=0; i<_interface.num_sensors; ++i)
    {
        Sensor *sensor = &_interface.sensors[i];
        free(sensor->color_frame);
        free(sensor->depth_frame);
        free(sensor->color_frame_offsets);
        free(sensor->depth_frame_offsets);
    }

    fclose(_interface.video_file);
    memset(&_interface, 0, sizeof(_interface));

    puts("Done.");
}

int
PollSensorList(SensorInfo *sensor_list, int max_sensors)
{
    int num_sensors = MIN(max_sensors, _interface.num_sensors);

    for(int i=0; i<num_sensors; ++i)
    {
        memcpy(&sensor_list[i], &_interface.sensor_infos[i], sizeof(SensorInfo));
    }

    return num_sensors;
}

int
SensorInitialize(SensorInfo *sensor, bool enable_color, bool enable_depth)
{
    return 0;
}

void
SensorFinalize(SensorInfo *sensor)
{
    // Ignore
}

ColorPixel *
GetSensorColorFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;

    const size_t buffer_size = sensor->color_stream_info.width * sensor->color_stream_info.height * sizeof(ColorPixel);
    uint8_t *compressed_buffer = (uint8_t *)malloc(buffer_size);
    fseek(_interface.video_file, s->color_frame_offsets[_interface.frame_index], SEEK_SET);
    size_t compressed_size = 0;
    fread(&compressed_size, sizeof(size_t), 1, _interface.video_file);
    fread(compressed_buffer, 1, compressed_size, _interface.video_file);

    size_t bytes_written = tinfl_decompress_mem_to_mem(s->color_frame, buffer_size, compressed_buffer, compressed_size, 0);
    assert(bytes_written == buffer_size);
    free(compressed_buffer);

    return s->color_frame;
}

DepthPixel *
GetSensorDepthFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;

    const size_t buffer_size = sensor->depth_stream_info.width * sensor->depth_stream_info.height * sizeof(DepthPixel);
    uint8_t *compressed_buffer = (uint8_t *)malloc(buffer_size);
    fseek(_interface.video_file, s->depth_frame_offsets[_interface.frame_index], SEEK_SET);
    size_t compressed_size = 0;
    fread(&compressed_size, sizeof(size_t), 1, _interface.video_file);
    fread(compressed_buffer, 1, compressed_size, _interface.video_file);

    size_t bytes_written = tinfl_decompress_mem_to_mem(s->depth_frame, buffer_size, compressed_buffer, compressed_size, 0);
    assert(bytes_written == buffer_size);
    free(compressed_buffer);

    // Increment frame_index
    ++_interface.frame_index;
    if(_interface.frame_index >= _interface.num_frames)
    {
        _interface.frame_index = 0;
    }

    return s->depth_frame;
}

