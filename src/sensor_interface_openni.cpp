#include "sensor_interface.h"

#include "OpenNI.h"
#include "utils.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

struct _sensor;

struct OpenNIFrameListener : public openni::VideoStream::NewFrameListener
{
    const char *name;
    int width, height;
    bool is_depth;
    void *data;
    openni::VideoFrameRef frame_ref;

    virtual void onNewFrame(openni::VideoStream &video_stream)
    {
        openni::Status rc;
        rc = video_stream.readFrame(&frame_ref);
        if(rc != openni::STATUS_OK)
        {
            fprintf(stderr, "OpenNI2 Error %d at %s:%d\n%s\n",
                    rc, __FILE__, __LINE__,
                    openni::OpenNI::getExtendedError());
            assert(false);
        }

#if 0
        assert(width == frame_ref.getWidth() &&
               height == frame_ref.getHeight());
#else
        if(width != frame_ref.getWidth() || height != frame_ref.getHeight())
        {
            printf("WARNING: %s got %s frame (%d x %d), but expected (%d x %d)\n",
                   name, is_depth ? "depth" : "color",
                   frame_ref.getWidth(), frame_ref.getHeight(),
                   width, height);
            return;
        }
#endif

        const void *frame = frame_ref.getData();
        if(frame)
        {
            if(is_depth)
            {
                DepthPixel *pixel = (DepthPixel *)data;
                // openni::DepthPixel is a 16 bit unsigned integer (uint16_t)
                const openni::DepthPixel *src = (const openni::DepthPixel *)frame;
                const int num_pixels = width * height;
                for(int i = 0; i < num_pixels; ++i)
                {
                    *pixel = (float)*src;
                    ++src;
                    ++pixel;
                }
            }
            else
            {
                memcpy(data, frame, width*height*sizeof(ColorPixel));
            }
        }
        else
        {
            fprintf(stderr, "%s got %s frame with no data\n",
                    name, is_depth ? "depth" : "color");
        }

        frame_ref.release();

#if 0 // Debugging
        openni::VideoMode mode = video_stream.getVideoMode();
        int w = mode.getResolutionX();
        int h = mode.getResolutionY();
        int fps = mode.getFps();
        printf("%s got %s frame from video stream (%d x %d) @ %d FPS\n",
               name, is_depth ? "depth" : "color", w, h, fps);
#endif
    }
};

typedef struct _sensor
{
    openni::Device oni_device;

    openni::VideoStream color_stream;
    openni::VideoFrameRef color_frame_ref;
    ColorPixel *color_frame;
    OpenNIFrameListener *color_frame_listener;

    openni::VideoStream depth_stream;
    openni::VideoFrameRef depth_frame_ref;
    DepthPixel *depth_frame;
    OpenNIFrameListener *depth_frame_listener;
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

        memset(sensor_list[i].serial, 0, sizeof(sensor_list[i].serial));

        sensor_list[i].sensor_data = NULL;
    }

    return num_sensors;
}

int
SensorInitialize(SensorInfo *sensor, bool enable_color, bool enable_depth)
{
    Sensor *s = new Sensor;
    assert(s);
    sensor->sensor_data = s;

    printf("Initializing sensor at %s\n", sensor->URI);

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

    int size = sizeof(sensor->serial);
    s->oni_device.getProperty(openni::DEVICE_PROPERTY_SERIAL_NUMBER, sensor->serial, &size);
    printf("Serial number: %s\n", sensor->serial);

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

        s->color_frame_listener = new OpenNIFrameListener();
        s->color_frame_listener->name = sensor->URI;
        s->color_frame_listener->width = vmode.getResolutionX();
        s->color_frame_listener->height = vmode.getResolutionY();
        s->color_frame_listener->is_depth = false;
        s->color_frame_listener->data = s->color_frame;
        s->color_stream.addNewFrameListener(s->color_frame_listener);
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

        // Try to enable image registration mode if supported
        if(enable_color && s->oni_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        {
            printf("Enabling image registration mode on %s (%s)\n", sensor->name, sensor->serial);
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

        s->depth_frame_listener = new OpenNIFrameListener();
        s->depth_frame_listener->name = sensor->URI;
        s->depth_frame_listener->width = vmode.getResolutionX();
        s->depth_frame_listener->height = vmode.getResolutionY();
        s->depth_frame_listener->is_depth = true;
        s->depth_frame_listener->data = s->depth_frame;
        s->depth_stream.addNewFrameListener(s->depth_frame_listener);
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
        printf("Finalizing sensor %s\n", sensor->serial);
        Sensor *s = sensor->sensor_data;

        if(s->color_frame)
        {
            s->color_stream.stop();
            s->color_stream.removeNewFrameListener(s->color_frame_listener);
            delete s->color_frame_listener;
            s->color_frame_listener = NULL;
            s->color_stream.destroy();
            free(sensor->sensor_data->color_frame);
        }

        if(s->depth_frame)
        {
            s->depth_stream.stop();
            s->depth_stream.removeNewFrameListener(s->depth_frame_listener);
            delete s->depth_frame_listener;
            s->depth_frame_listener = NULL;
            s->depth_stream.destroy();
            free(s->depth_frame);
        }

        delete s;
        sensor->sensor_data = NULL;
    }
}

ColorPixel *
GetSensorColorFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;
    return s->color_frame;
}

DepthPixel *
GetSensorDepthFrame(SensorInfo *sensor)
{
    Sensor *s = sensor->sensor_data;
    return s->depth_frame;
}

