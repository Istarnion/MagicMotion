#include "scene_video.h"

#include "sensor_interface.h"
#include "renderer.h"
#include "ui.h"

namespace video
{
    typedef enum
    {
        VIDEO_RGB,
        VIDEO_DEPTH,
        VIDEO_DEPTH_COLORIZED
    } VideoMode;

    static SensorInfo sensor;
    static GLuint color_texture;
    static GLuint depth_texture;
    static VideoMode video_mode;
    static ColorPixel *depth_pixels;

    bool
    SceneInit(void)
    {
        int num_sensors = PollSensorList(&sensor, 1);
        if(num_sensors > 0)
        {
            int rc = SensorInitialize(&sensor, true, true);
            if(rc)
            {
                fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor.vendor, sensor.name, sensor.URI);
                return false;
            }
        }
        else
        {
            return false;
        }

        glGenTextures(1, &color_texture);
        glBindTexture(GL_TEXTURE_2D, color_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                     sensor.color_stream_info.width,
                     sensor.color_stream_info.height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

        glGenTextures(1, &depth_texture);
        glBindTexture(GL_TEXTURE_2D, depth_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                     sensor.depth_stream_info.width,
                     sensor.depth_stream_info.height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

        depth_pixels = (ColorPixel *)malloc(sensor.depth_stream_info.width *
                                            sensor.depth_stream_info.height *
                                            sizeof(ColorPixel));
        return true;
    }

    void
    SceneUpdate(void)
    {
        UpdateProjectionMatrix();

        if(ImGui::Begin("Video Mode"))
        {
            ImGui::RadioButton("RGB", (int *)&video_mode, VIDEO_RGB);
            ImGui::RadioButton("Depth", (int *)&video_mode, VIDEO_DEPTH);
            ImGui::RadioButton("Depth Colorized", (int *)&video_mode, VIDEO_DEPTH_COLORIZED);
        }

        ImGui::End();

        if(video_mode == VIDEO_RGB)
        {
            ColorPixel *color_frame = GetSensorColorFrame(&sensor);
            glBindTexture(GL_TEXTURE_2D, color_texture);
            glTexSubImage2D(GL_TEXTURE_2D, 0,
                            0, 0,
                            sensor.color_stream_info.width,
                            sensor.color_stream_info.height,
                            GL_RGB, GL_UNSIGNED_BYTE,
                            color_frame);
        }
        else
        {

            DepthPixel *depth_frame = GetSensorDepthFrame(&sensor);

            glBindTexture(GL_TEXTURE_2D, depth_texture);
            // Normalize the values:
            size_t num_pixels = sensor.depth_stream_info.width * sensor.depth_stream_info.height;
            float denominator = sensor.depth_stream_info.max_depth - sensor.depth_stream_info.min_depth;
            if(video_mode == VIDEO_DEPTH)
            {
                for(size_t i=0; i<num_pixels; ++i)
                {
                    float value = (depth_frame[i] - sensor.depth_stream_info.min_depth) / denominator;
                    if(value < 0) value = 0;
                    depth_pixels[i].r = (unsigned char)(value * 0xFF);
                    depth_pixels[i].g = (unsigned char)(value * 0xFF);
                    depth_pixels[i].b = (unsigned char)(value * 0xFF);
                }
            }
            else if(video_mode == VIDEO_DEPTH_COLORIZED)
            {
                static ColorPixel gradient[] = {
                    { 0x77, 0x00, 0x00 },
                    { 0xFF, 0x00, 0x00 },
                    { 0xF0, 0xF0, 0x00 },
                    { 0x00, 0xFF, 0x00 },
                    { 0x00, 0x00, 0xFF },
                    { 0x00, 0x00, 0x77 },
                    { 0x00, 0x00, 0x00 }
                };

                for(size_t i=0; i<num_pixels; ++i)
                {
                    float value = (depth_frame[i] - sensor.depth_stream_info.min_depth) / denominator;
                    if(value > 0)
                    {
                        value *= sizeof(gradient) / sizeof(ColorPixel);
                        ColorPixel a = gradient[(int)value];
                        ColorPixel b = gradient[(int)value+1];
                        value -= (int)value;

                        depth_pixels[i].r = (unsigned char)(a.r*(1.0f-value) + b.r*value);
                        depth_pixels[i].g = (unsigned char)(a.g*(1.0f-value) + b.g*value);
                        depth_pixels[i].b = (unsigned char)(a.b*(1.0f-value) + b.b*value);
                    }
                    else
                    {
                        depth_pixels[i] = (ColorPixel){ 0, 0, 0 };
                    }
                }
            }

            glTexSubImage2D(GL_TEXTURE_2D, 0,
                            0, 0,
                            sensor.depth_stream_info.width,
                            sensor.depth_stream_info.height,
                            GL_RGB, GL_UNSIGNED_BYTE,
                            depth_pixels);
        }

        RenderFullscreenQuad();
    }

    void
    SceneEnd(void)
    {
        SensorFinalize(&sensor);

        glBindTexture(GL_TEXTURE_2D, 0);
        glDeleteTextures(1, &color_texture);
        glDeleteTextures(1, &depth_texture);

        free(depth_pixels);
    }
}

Scene
GetVideoScene(void)
{
    Scene result;
    result.Init = &video::SceneInit;
    result.Update = &video::SceneUpdate;
    result.End = &video::SceneEnd;

    return result;
}

