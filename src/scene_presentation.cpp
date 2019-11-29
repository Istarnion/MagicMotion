#include "scene_presentation.h"

#include <SDL.h>
#include "sensor_interface.h"
#include "renderer.h"
#include "ui.h"

typedef enum
{
    PRESENTATION_SLIDES,
    PRESENTATION_VIDEO,
    PRESENTATION_POINT_CLOUD
} PresentationMode;

typedef struct
{
    SensorInfo sensor;
    Frustum frustum;
    bool show_frustum;
    V3 *point_cloud;
} PresentationSensorRenderData;

static PresentationSensorRenderData active_sensor;
static Camera presentation_cam;
static PresentationMode presentation_mode = PRESENTATION_SLIDES;
static GLuint video_texture;

#define NUM_SLIDES 6
static GLuint slides[NUM_SLIDES];

static float tween_t;
static int slide_index;

#define LERP(a, b, t) ((a)*(1.0f-(t)) + (b)*(t))

bool
PresentationSceneInit(void)
{
    presentation_cam.pitch = 0;
    presentation_cam.yaw = 0;
    presentation_cam.position = MakeV3(0, 0, 0);
    /*
    presentation_cam.pitch = 0.187f;
    presentation_cam.yaw = 0.113f;
    presentation_cam.position = MakeV3(-500, 1000, -1300);
    // */
    CameraLookAt(&presentation_cam, MakeV3(0, 0, 0));

    SensorInfo *sensor = &active_sensor.sensor;
    int num_active_sensors = PollSensorList(sensor, 1);
    if(num_active_sensors > 0)
    {
        int rc = SensorInitialize(sensor, true, true);
        if(rc)
        {
            fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
            return false;
        }

        active_sensor.frustum = (Frustum){
            .position = (V3){ 0, 0, 0 },
            .pitch = 0,
            .yaw = 0,
            .roll = 0,
            .fov = sensor->depth_stream_info.fov,
            .aspect = sensor->depth_stream_info.aspect_ratio,
            .near_plane = sensor->depth_stream_info.min_depth,
            .far_plane = sensor->depth_stream_info.max_depth
        };

        active_sensor.point_cloud = (V3 *)calloc(sensor->depth_stream_info.width * sensor->depth_stream_info.height,
                                                     sizeof(V3));
    }
    else
    {
        return false;
    }

    glGenTextures(1, &video_texture);
    glBindTexture(GL_TEXTURE_2D, video_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 active_sensor.sensor.color_stream_info.width,
                 active_sensor.sensor.color_stream_info.height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    const char *slide_files[NUM_SLIDES] = {
        "slides/slide1.png",
        "slides/slide2.png",
        "slides/slide3.png",
        "slides/slide4.png",
        "slides/slide5.png",
        "slides/slide6.png"
    };

    glGenTextures(NUM_SLIDES, slides);
    for(size_t i=0; i<NUM_SLIDES; ++i)
    {
        Image image;
        LoadImage(&image, slide_files[i]);

        glBindTexture(GL_TEXTURE_2D, slides[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                     image.width, image.height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, image.pixels);

        FreeImage(&image);
    }

    return true;
}

void
PresentationSceneUpdate(void)
{
    _UpdateProjectionMatrix();

    /*
    if(ImGui::Begin("Sensor"))
    {
        ImGui::InputFloat3("Position", (float *)&active_sensor.frustum.position);
        ImGui::SliderFloat("Pitch", &active_sensor.frustum.pitch, 0, 2.0f*M_PI);
        ImGui::SliderFloat("Yaw", &active_sensor.frustum.yaw, 0, 2.0f*M_PI);
        ImGui::SliderFloat("Roll", &active_sensor.frustum.roll, 0, 2.0f*M_PI);
        ImGui::Checkbox("Frustum", &active_sensor.show_frustum);
    }

    ImGui::End();

    if(ImGui::Begin("Camera"))
    {
        ImGui::InputFloat3("Position", (float *)&presentation_cam.position);
        ImGui::SliderFloat("Pitch", &presentation_cam.pitch, 0, 2.0f*M_PI);
        ImGui::SliderFloat("Yaw", &presentation_cam.yaw, 0, 2.0f*M_PI);
    }

    ImGui::End();
    // */

    InputState *input = Input();
    if(input->right == PRESSED)
    {
        if(presentation_mode == PRESENTATION_SLIDES)
        {
            ++slide_index;
            if(slide_index >= NUM_SLIDES)
            {
                presentation_mode = PRESENTATION_VIDEO;
            }
        }
        else
        {
            presentation_mode = PRESENTATION_POINT_CLOUD;
            tween_t = 0;
        }
    }
    else if(input->left == PRESSED)
    {
        if(presentation_mode == PRESENTATION_SLIDES && slide_index > 0)
        {
            --slide_index;
        }
        else if(presentation_mode == PRESENTATION_VIDEO)
        {
            slide_index = 0;
            presentation_mode = PRESENTATION_SLIDES;
        }
        else if(presentation_mode == PRESENTATION_POINT_CLOUD)
        {
            presentation_mode = PRESENTATION_VIDEO;
        }
    }

    RendererSetViewMatrix(CameraGetViewMatrix(&presentation_cam));

    switch(presentation_mode)
    {
        case PRESENTATION_SLIDES:
        {
            glBindTexture(GL_TEXTURE_2D, slides[slide_index]);
            RenderFullscreenQuad();
            break;
        }
        case PRESENTATION_VIDEO:
        {
            ColorPixel *color_frame = GetSensorColorFrame(&active_sensor.sensor);
            glBindTexture(GL_TEXTURE_2D, video_texture);
            glTexSubImage2D(GL_TEXTURE_2D, 0,
                            0, 0,
                            active_sensor.sensor.color_stream_info.width,
                            active_sensor.sensor.color_stream_info.height,
                            GL_RGB, GL_UNSIGNED_BYTE,
                            color_frame);
            RenderFullscreenQuad();
            break;
        }
        case PRESENTATION_POINT_CLOUD:
        {
            presentation_cam.yaw = LERP(0, 0.2, tween_t);
            presentation_cam.position = MakeV3(LERP(0, -1500, tween_t),
                                               0,
                                               LERP(0, -500, tween_t));

            RendererSetViewMatrix(CameraGetViewMatrix(&presentation_cam));
            tween_t = MIN(1.0f, tween_t + (1.0f/30.0f));

            DepthPixel *depth_frame = GetSensorDepthFrame(&active_sensor.sensor);

            const size_t w = active_sensor.sensor.depth_stream_info.width;
            const size_t h = active_sensor.sensor.depth_stream_info.height;
            const float fov = active_sensor.sensor.depth_stream_info.fov;
            const float aspect = active_sensor.sensor.depth_stream_info.aspect_ratio;

            size_t num_points = 0;
            for(size_t y=0; y<h; ++y)
            for(size_t x=0; x<w; ++x)
            {
                float depth = depth_frame[(w-x-1)+y*w];
                if(depth > 0.0f)
                {
                    float pos_x = tanf((((float)x / (float)w)-0.5f)*fov) * depth;
                    float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;
                    V3 point = (V3){ pos_x, pos_y, depth };
                    active_sensor.point_cloud[num_points++] = point;
                }
            }

            RenderCubes(active_sensor.point_cloud, num_points,
                        active_sensor.frustum.position,
                        (V3){ active_sensor.frustum.pitch,
                              active_sensor.frustum.yaw,
                              active_sensor.frustum.roll },
                        (V3){ 1, 1, 1 });

            if(active_sensor.show_frustum)
            {
                RenderFrustum(&active_sensor.frustum);
            }

            break;
        }
    }
}

void
PresentationSceneEnd(void)
{
    SensorFinalize(&active_sensor.sensor);
    free(active_sensor.point_cloud);

    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &video_texture);
    glDeleteTextures(NUM_SLIDES, slides);

    FinalizeSensorInterface();
}

Scene
GetPresentationScene(void)
{
    Scene result;
    result.Init = &PresentationSceneInit;
    result.Update = &PresentationSceneUpdate;
    result.End = &PresentationSceneEnd;

    return result;
}

