#include "sensor_interface.h"

#include "OpenNI.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct _sensor
{
    openni::Device oni_device;

    openni::VideoStream color_stream;
    openni::VideoFrameRef color_frame_ref;
    ColorPixel *color_frame;

    openni::VideoStream depth_stream;
    openni::VideoFrameRef depth_frame_ref;
    DepthPixel *depth_frame;
} Sensor;

void
InitializeSensorInterface(void)
{
    puts("Initializing OpenNI2..");
    openni::Status rc = openni::OpenNI::initialize();
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                rc, __FILE__, __LINE__,
                openni::OpenNI::getExtendedError());
    }

    puts("Done.");
}

void
FinalizeSensorInterface(void)
{
    puts("Shutting down OpenNI2..");
    openni::OpenNI::shutdown();
    puts("Done.");
}

int
PollSensorList(SensorInfo *sensor_list, int max_sensors)
{
    openni::Array<openni::DeviceInfo> device_list;
    openni::OpenNI::enumerateDevices(&device_list);

    int num_sensors = MIN(device_list.getSize(), max_sensors);
    for(int i=0; i<num_sensors; ++i)
    {
        openni::DeviceInfo device = device_list[i];

        strncpy(sensor_list[i].name, device.getName(), sizeof(sensor_list[i].name));
        if(!sensor_list[i].name[0]) strcpy(sensor_list[i].name, "[UNKNOWN NAME]");
        sensor_list[i].name[sizeof(sensor_list[i].name)-1] = '\0';

        strncpy(sensor_list[i].URI, device.getUri(), sizeof(sensor_list[i].URI));
        if(!sensor_list[i].URI[0]) strcpy(sensor_list[i].name, "[UNKNOWN URI]");
        sensor_list[i].URI[sizeof(sensor_list[i].URI)-1] = '\0';

        strncpy(sensor_list[i].vendor, device.getVendor(), sizeof(sensor_list[i].vendor));
        if(!sensor_list[i].vendor[0]) strcpy(sensor_list[i].name, "[UNKNOWN VENDOR]");
        sensor_list[i].vendor[sizeof(sensor_list[i].vendor)-1] = '\0';
    }

    return num_sensors;
}

int
SensorInitialize(SensorInfo *sensor, bool enable_color, bool enable_depth)
{
    Sensor *s = new Sensor;
    assert(s);
    sensor->sensor_data = s;

    openni::Status rc;
    rc = s->oni_device.open(sensor->URI);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                rc, __FILE__, __LINE__,
                openni::OpenNI::getExtendedError());
        SensorFinalize(sensor);
        return -1;
    }

    puts("Successfully initialized the device");

    if(enable_color)
    {
        rc = s->color_stream.create(s->oni_device, openni::SENSOR_COLOR);
        if(rc != openni::STATUS_OK)
        {
            fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                    rc, __FILE__, __LINE__,
                    openni::OpenNI::getExtendedError());
            SensorFinalize(sensor);
            return -2;
        }

        rc = s->color_stream.start();
        if(rc != openni::STATUS_OK)
        {
            fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                    rc, __FILE__, __LINE__,
                    openni::OpenNI::getExtendedError());
            SensorFinalize(sensor);
            return -3;
        }

        const openni::VideoMode &vmode = s->color_stream.getVideoMode();
        sensor->color_stream_info.width = vmode.getResolutionX();
        sensor->color_stream_info.height = vmode.getResolutionY();
        sensor->color_stream_info.fov = s->color_stream.getHorizontalFieldOfView();
        float vfov = s->color_stream.getVerticalFieldOfView();
        sensor->color_stream_info.aspect_ratio = sensor->color_stream_info.fov / vfov;

        s->color_frame = (ColorPixel *)calloc(sensor->color_stream_info.width * sensor->color_stream_info.height,
                                              sizeof(ColorPixel));
    }
    else
    {
        s->color_frame = NULL;
    }

    if(enable_depth)
    {
        rc = s->depth_stream.create(s->oni_device, openni::SENSOR_DEPTH);
        if(rc != openni::STATUS_OK)
        {
            fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                    rc, __FILE__, __LINE__,
                    openni::OpenNI::getExtendedError());
            SensorFinalize(sensor);
            return -4;
        }

        rc = s->depth_stream.start();
        if(rc != openni::STATUS_OK)
        {
            fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                    rc, __FILE__, __LINE__,
                    openni::OpenNI::getExtendedError());
            SensorFinalize(sensor);
            return -5;
        }

        if(enable_color && s->oni_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        {
            s->oni_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        }

        const openni::VideoMode &vmode = s->depth_stream.getVideoMode();
        sensor->depth_stream_info.width = vmode.getResolutionX();
        sensor->depth_stream_info.height = vmode.getResolutionY();
        sensor->depth_stream_info.fov = s->depth_stream.getHorizontalFieldOfView();
        float vfov = s->depth_stream.getVerticalFieldOfView();
        sensor->depth_stream_info.aspect_ratio = sensor->depth_stream_info.fov / vfov;
        sensor->depth_stream_info.min_depth = s->depth_stream.getMinPixelValue();
        sensor->depth_stream_info.max_depth = s->depth_stream.getMaxPixelValue();

        s->depth_frame = (DepthPixel *)calloc(sensor->depth_stream_info.width * sensor->depth_stream_info.height,
                                              sizeof(DepthPixel));
    }
    else
    {
        s->depth_frame = NULL;
    }

    return 0;
}

void
SensorFinalize(SensorInfo *sensor)
{
    if(sensor->sensor_data)
    {
        if(sensor->sensor_data->color_frame)
        {
            sensor->sensor_data->color_stream.destroy();
            free(sensor->sensor_data->color_frame);
        }

        if(sensor->sensor_data->color_frame)
        {
            sensor->sensor_data->depth_stream.destroy();
            free(sensor->sensor_data->depth_frame);
        }

        delete sensor->sensor_data;
        sensor->sensor_data = NULL;
    }
}

ColorPixel *
GetSensorColorFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;

    openni::Status rc;
    rc = s->color_stream.readFrame(&s->color_frame_ref);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                rc, __FILE__, __LINE__,
                openni::OpenNI::getExtendedError());
        SensorFinalize(sensor);
        assert(false);
    }

    assert(sensor->color_stream_info.width == s->color_frame_ref.getWidth() &&
           sensor->color_stream_info.height == s->color_frame_ref.getHeight());

    const openni::RGB888Pixel *color_data =
        (const openni::RGB888Pixel *)s->color_frame_ref.getData();

    if(color_data)
    {
        ColorPixel *pixel = s->color_frame;
        int num_pixels = sensor->color_stream_info.width * sensor->color_stream_info.height;
        for(int i = 0; i < num_pixels; ++i)
        {
            openni::RGB888Pixel color = *(color_data++);
            pixel->r = (unsigned char)color.r;
            pixel->g = (unsigned char)color.g;
            pixel->b = (unsigned char)color.b;
            ++pixel;
        }
    }
    else
    {
        fprintf(stderr, "Got frame with no data.\n");
    }

    s->color_frame_ref.release();

    return s->color_frame;
}

DepthPixel *
GetSensorDepthFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;

    openni::Status rc;
    rc = s->depth_stream.readFrame(&s->depth_frame_ref);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                rc, __FILE__, __LINE__,
                openni::OpenNI::getExtendedError());
        SensorFinalize(sensor);
        assert(false);
    }

    assert(sensor->depth_stream_info.width == s->depth_frame_ref.getWidth() &&
           sensor->depth_stream_info.height == s->depth_frame_ref.getHeight());

    const openni::DepthPixel *depth_data =
        (const openni::DepthPixel *)s->depth_frame_ref.getData();

    if(depth_data)
    {
        DepthPixel *pixel = s->depth_frame;
        int num_pixels = sensor->depth_stream_info.width * sensor->depth_stream_info.height;
        for(int i = 0; i < num_pixels; ++i)
        {
            *pixel = (float)*depth_data;
            ++depth_data;
            ++pixel;
        }
    }
    else
    {
        fprintf(stderr, "Got frame with no data.\n");
    }

    s->depth_frame_ref.release();

    return s->depth_frame;
}

