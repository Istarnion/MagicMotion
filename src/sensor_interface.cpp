
#include "sensor_interface.h"

#include "OpenNI.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static openni::Device oni_device;
static openni::VideoStream color_stream;
static openni::VideoFrameRef color_frame;
static openni::VideoStream depth_stream;
static openni::VideoFrameRef depth_frame;

Frame output_frame;

int
SensorInitialize(void)
{
    openni::Status rc;
    rc = openni::OpenNI::initialize();
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        return -1;
    }

    rc = oni_device.open(openni::ANY_DEVICE);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        return -2;
    }

    rc = color_stream.create(oni_device, openni::SENSOR_COLOR);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        return -3;
    }

    rc = color_stream.start();
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        return -4;
    }

    rc = depth_stream.create(oni_device, openni::SENSOR_DEPTH);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        return -4;
    }

    rc = depth_stream.start();
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        return -5;
    }

    const openni::VideoMode &vmode = depth_stream.getVideoMode();
    output_frame.width = vmode.getResolutionX();
    output_frame.height = vmode.getResolutionY();

    size_t buffer_size =
        output_frame.width * output_frame.height * sizeof(Pixel);
    output_frame.pixels = (Pixel *)malloc(buffer_size);

    return 0;
}

void
SensorFinalize(void)
{
    puts("Shutting down OpenNI..");
    free(output_frame.pixels);
    output_frame.pixels = NULL;

    color_stream.destroy();
    depth_stream.destroy();
    oni_device.close();

    openni::OpenNI::shutdown();
}

void
GetSensorResolution(int *width, int *height)
{
    *width = output_frame.width;
    *height = output_frame.height;
}

float
GetSensorFOV(void)
{
    float fov = depth_stream.getHorizontalFieldOfView();
    return fov;
}

float
GetSensorAspectRatio(void)
{
    float h = depth_stream.getHorizontalFieldOfView();
    float v = depth_stream.getVerticalFieldOfView();
    return h / v;
}

Frame
GetSensorFrame(bool patch, bool record)
{
    openni::Status rc;

    rc = color_stream.readFrame(&color_frame);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        assert(false);
    }

    rc = depth_stream.readFrame(&depth_frame);
    if(rc != openni::STATUS_OK)
    {
        fprintf(stderr, "OpenNI2 Error %d\n%s\n", rc,
                openni::OpenNI::getExtendedError());
        SensorFinalize();
        assert(false);
    }

    assert(output_frame.width == color_frame.getWidth() &&
           output_frame.height == color_frame.getHeight());
    assert(output_frame.width == depth_frame.getWidth() &&
           output_frame.height == depth_frame.getHeight());

    const openni::RGB888Pixel *color_data =
        (const openni::RGB888Pixel *)color_frame.getData();
    const openni::DepthPixel *depth_data =
        (const openni::DepthPixel *)depth_frame.getData();

    /*
    if(record)
    {
        push_frame((RGBPixel *)color_data, depth_data);
    }
    */

    if(color_data && depth_data)
    {
        Pixel *pixel = output_frame.pixels;
        for(int i = 0; i < output_frame.width * output_frame.height; ++i)
        {
            openni::RGB888Pixel color = *(color_data++);
            pixel->r = (float)color.r / 255.0f;
            pixel->g = (float)color.g / 255.0f;
            pixel->b = (float)color.b / 255.0f;
            float depth = (float)*(depth_data++);
            if(!patch || depth > 0.0f)
            {
                pixel->depth = depth;
            }
            ++pixel;
        }
    }
    else
    {
        fprintf(stderr, "Got frame with no data.\n");
    }

    color_frame.release();
    depth_frame.release();

    return output_frame;
}

