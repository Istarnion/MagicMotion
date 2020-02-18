#include "sensor_interface.h"

#include "librealsense2/rs.h"
#include "librealsense2/h/rs_pipeline.h"
#include "librealsense2/h/rs_config.h"
#include "utils.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct _sensor
{
    rs2_pipeline *pipe;

    float units_to_mm;
    
    ColorPixel *color_frame;
    DepthPixel *depth_frame;
} Sensor;

static rs2_context *_rs_context;
static unsigned int _color_frame_index = 0;
static unsigned int _depth_frame_index = 0;

static rs2_frame *_latest_frame;

static void
_report_error(const rs2_error *err, const char *file, int line)
{
    fprintf(stderr, "libRealSense error at %s:%d\n\t%s(%s)\n\t%s\n",
            file, line,
            rs2_get_failed_function(err), rs2_get_failed_args(err),
            rs2_get_error_message(err));
}

#define report_error(err) _report_error((err), __FILE__, __LINE__)

// Get units per meter for the device
static float
_get_depth_scale(const rs2_device* dev)
{
    rs2_error* e = NULL;
    rs2_sensor_list* sensor_list = rs2_query_sensors(dev, &e);
    if(e) report_error(e);

    int num_of_sensors = rs2_get_sensors_count(sensor_list, &e);
    if(e) report_error(e);

    float depth_scale = 0;
    int is_depth_sensor_found = 0;
    for (int i = 0; i < num_of_sensors; ++i)
    {
        rs2_sensor* sensor = rs2_create_sensor(sensor_list, i, &e);
        if(e) report_error(e);

        // Check if the given sensor can be extended to depth sensor interface
        is_depth_sensor_found = rs2_is_sensor_extendable_to(sensor, RS2_EXTENSION_DEPTH_SENSOR, &e);
        if(e) report_error(e);

        if (1 == is_depth_sensor_found)
        {
            depth_scale = rs2_get_option((const rs2_options*)sensor, RS2_OPTION_DEPTH_UNITS, &e);
            if(e) report_error(e);
            rs2_delete_sensor(sensor);
            break;
        }
        rs2_delete_sensor(sensor);
    }
    rs2_delete_sensor_list(sensor_list);

    if (0 == is_depth_sensor_found)
    {
        printf("Depth sensor not found!\n");
        exit(EXIT_FAILURE);
    }

    return depth_scale;
}

void
InitializeSensorInterface(void)
{
    puts("Initializing libRealSense..");

    rs2_error *err = NULL;
    _rs_context = rs2_create_context(RS2_API_VERSION, &err);

    if(err)
    {
        report_error(err);
    }
    else
    {
        puts("Done.");
    }
}

void
FinalizeSensorInterface(void)
{
    puts("Shutting down libRealSense..");
    if(_latest_frame != NULL) rs2_release_frame(_latest_frame);
    rs2_delete_context(_rs_context);
    puts("Done.");
}

int
PollSensorList(SensorInfo *sensor_list, int max_sensors)
{
    rs2_error *err = NULL;
    rs2_device_list *devices = rs2_query_devices(_rs_context, &err);
    if(err) report_error(err);

    int device_count = MIN(rs2_get_device_count(devices, &err), max_sensors);
    if(err) report_error(err);

    for(int i=0; i<MIN(device_count, max_sensors); ++i)
    {
        SensorInfo *info = &sensor_list[i];

        rs2_device *device = rs2_create_device(devices, i, &err);
        if(err)
        {
            report_error(err);
            continue;
        }

        strcpy(info->vendor, "Intel Corporation");
        strncpy(info->name, rs2_get_device_info(device, RS2_CAMERA_INFO_NAME, &err), 128);
        strncpy(info->URI, rs2_get_device_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER, &err), 128);

        rs2_delete_device(device);
    }

    rs2_delete_device_list(devices);

    return device_count;
}

int
SensorInitialize(SensorInfo *sensor, bool enable_color, bool enable_depth)
{
    printf("Initializing sensor %s (%s)\n", sensor->name, sensor->URI);
    Sensor *s = (Sensor *)malloc(sizeof(Sensor));
    
    rs2_error *err = NULL;
    rs2_config *cfg = rs2_create_config(&err);
    if(err)
    {
        report_error(err);
        return -1;
    }
    
    rs2_config_enable_device(cfg, sensor->URI, &err);
    if(err)
    {
        report_error(err);
        return -2;
    }
    
    rs2_config_enable_all_stream(cfg, &err);
    if(err) report_error(err);
    
    s->pipe = rs2_create_pipeline(_rs_context, &err);
    if(err)
    {
        report_error(err);
        return -3;
    }

    rs2_pipeline_profile *pipe_profile = rs2_pipeline_start_with_config(s->pipe, cfg, &err);
    if(err)
    {
        report_error(err);
        return -4;
    }
    
    rs2_stream_profile_list *streams = rs2_pipeline_profile_get_streams(pipe_profile, &err);
    if(err)
    {
        report_error(err);
        return -5;
    }
    
    int stream_profile_count = rs2_get_stream_profiles_count(streams, &err);
    for(int i=0; i<stream_profile_count; ++i)
    {
        const rs2_stream_profile *stream_profile = rs2_get_stream_profile(streams, i, &err);
        if(err)
        {
            report_error(err);
            return -6;
        }
        
        rs2_stream stream;
        rs2_format format;
        int index;
        int uid;
        int framerate;
        rs2_get_stream_profile_data(stream_profile, &stream, &format, &index, &uid, &framerate, &err);
        if(err)
        {
            report_error(err);
            return -7;
        }
        int width, height;
        rs2_get_video_stream_resolution(stream_profile, &width, &height, &err);
        if(err)
        {
            report_error(err);
            return -8;
        }
        
        if(enable_depth &&
           format == RS2_FORMAT_Z16 &&
           s->depth_frame != NULL)
        {
            // First depth stream
            s->depth_frame = (DepthPixel *)calloc(width*height, sizeof(DepthPixel));
            
            sensor->depth_stream_info.aspect_ratio = (float)width / (float)height;
            sensor->depth_stream_info.fov = 1;
            sensor->depth_stream_info.min_depth = 0.1f;
            sensor->depth_stream_info.max_depth = 5000.0f;
            sensor->depth_stream_info.width = width;
            sensor->depth_stream_info.height = height;
            
            float depth_scale = _get_depth_scale(rs2_pipeline_profile_get_device(pipe_profile, &err));
            if(err)
            {
                report_error(err);
                return -9;
            }

            s->units_to_mm = 1.0f / (depth_scale * 1000.0f);
            
            printf("Initialized depth stream %s:\n\tResolution: %d x %d\n\tFramerate: %d\n\tUnits/m: %f\n",
                   rs2_stream_to_string(stream), width, height, framerate, depth_scale);
        }
        else if(enable_color &&
                format == RS2_FORMAT_RGB8 &&
                s->color_frame != NULL)
        {
            // First color stream
            s->color_frame = (ColorPixel *)calloc(width*height, sizeof(ColorPixel));
            
            sensor->color_stream_info.aspect_ratio = (float)width / (float)height;
            sensor->color_stream_info.fov = 1;
            sensor->color_stream_info.width = width;
            sensor->color_stream_info.height = height;
            
            printf("Initialized color stream %s:\n\tResolution: %d x %d\n\tFramerate: %d\n",
                   rs2_stream_to_string(stream), width, height, framerate);
        }
        else
        {
            printf("Ignoring stream %s:\n\tResolution: %d x %d\n\tFramerate: %d\n",
                   rs2_stream_to_string(stream), width, height, framerate);
        }
    }
    
    rs2_delete_stream_profiles_list(streams);

    sensor->sensor_data = s;
    return 0;
}

void
SensorFinalize(SensorInfo *sensor)
{
    if(sensor->sensor_data)
    {
        printf("Finalizing sensor %s(%s)\n", sensor->name, sensor->URI);
        
        rs2_pipeline_stop(sensor->sensor_data->pipe, NULL);
        rs2_delete_pipeline(sensor->sensor_data->pipe);
        
        free(sensor->sensor_data->color_frame);
        free(sensor->sensor_data->depth_frame);
        
        free(sensor->sensor_data);
        sensor->sensor_data = NULL;
    }
}

ColorPixel *
GetSensorColorFrame(SensorInfo *sensor)
{
    rs2_error *err = NULL;
    if(_color_frame_index >= _depth_frame_index)
    {
        ++_color_frame_index;
        if(_latest_frame) rs2_release_frame(_latest_frame);
        _latest_frame = rs2_pipeline_wait_for_frames(sensor->sensor_data->pipe, 10000, &err);
        if(err)
        {
            report_error(err);
            return NULL;
        }
    }
    else
    {
        _color_frame_index = _depth_frame_index;
    }
    
    int num_frames = rs2_embedded_frames_count(_latest_frame, &err);
    
    for(int i=num_frames-1; i>=0; --i)
    {
        rs2_frame *frame = rs2_extract_frame(_latest_frame, i, &err);
        rs2_format format;
        const rs2_stream_profile *profile = rs2_get_frame_stream_profile(frame, &err);
        rs2_get_stream_profile_data(profile, NULL, &format, NULL, NULL, NULL, &err);
        
        if(format == RS2_FORMAT_RGB8)
        {
            rs2_release_frame(frame);
            continue;
        }
        
        const uint8_t *rgb_data = (const uint8_t *)rs2_get_frame_data(frame, &err);
        int num_pixels = sensor->color_stream_info.width * sensor->color_stream_info.height;
        for(int j=0; j<num_pixels; ++j)
        {
            ColorPixel pixel;
            pixel.r = *rgb_data++;
            pixel.g = *rgb_data++;
            pixel.b = *rgb_data++;
            sensor->sensor_data->color_frame[j] = pixel;
        }
        
        rs2_release_frame(frame);
        break;
    }
    
    return sensor->sensor_data->color_frame;
}

DepthPixel *
GetSensorDepthFrame(SensorInfo *sensor)
{
    rs2_error *err = NULL;
    if(_depth_frame_index >= _color_frame_index)
    {
        ++_depth_frame_index;
        if(_latest_frame != NULL) rs2_release_frame(_latest_frame);
        _latest_frame = rs2_pipeline_wait_for_frames(sensor->sensor_data->pipe, 10000, &err);
        if(err)
        {
            report_error(err);
            return NULL;
        }
    }
    else
    {
        _depth_frame_index = _color_frame_index;
    }

    int num_frames = rs2_embedded_frames_count(_latest_frame, &err);
    
    for(int i=num_frames-1; i>=0; --i)
    {
        rs2_frame *frame = rs2_extract_frame(_latest_frame, i, &err);
        
        if(0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &err))
        {
            rs2_release_frame(frame);
            continue;
        }
        
        const uint16_t *depth_data = (const uint16_t *)rs2_get_frame_data(frame, &err);
        if(err)
        {
            report_error(err);
        }
        else
        {
            int num_pixels = sensor->depth_stream_info.width * sensor->depth_stream_info.height;
            float total_depth = 0;
            
            for(int j=0; j<num_pixels; ++j)
            {
                DepthPixel depth = (float)depth_data[j] * sensor->sensor_data->units_to_mm;
                sensor->sensor_data->depth_frame[j] = (DepthPixel)depth;
                total_depth += depth;
            }
        }
        
        rs2_release_frame(frame);
        break;
    }
    
    return sensor->sensor_data->depth_frame;
}
