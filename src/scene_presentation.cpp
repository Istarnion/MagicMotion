#include "scene_presentation.h"

#include <SDL.h>
#include "sensor_interface.h"
#include "renderer.h"
#include "ui.h"
#include "sensor_serialization.h"

typedef struct
{
    SensorInfo sensor;
    Frustum frustum;
    bool show_frustum;
    V3 *point_cloud;
} PresentationSensorRenderData;

static PresentationSensorRenderData active_sensor;
static Camera presentation_cam;

bool
PresentationSceneInit(void)
{
    presentation_cam.pitch = 0;
    presentation_cam.yaw = 0;
    presentation_cam.position = MakeV3(0, 0, 0);
    CameraLookAt(&presentation_cam, MakeV3(0, 0, 0));

    SerializedSensor serialized_sensors[4];
    int num_serialized_sensors = LoadSensors(serialized_sensors, 4);

    SensorInfo *sensor = &active_sensor.sensor;
    int num_active_sensors = PollSensorList(sensor, 1);
    if(num_active_sensors > 0)
    {
        int rc = SensorInitialize(sensor, false, true);
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

        for(int j=0; j<num_serialized_sensors; ++j)
        {
            if(strcmp(sensor->URI, serialized_sensors[j].URI) == 0)
            {
                printf("Loading data for URI %s\n", sensor->URI);
                active_sensor.frustum.position   = serialized_sensors[j].frustum.position;
                active_sensor.frustum.pitch      = serialized_sensors[j].frustum.pitch;
                active_sensor.frustum.yaw        = serialized_sensors[j].frustum.yaw;
                active_sensor.frustum.roll       = serialized_sensors[j].frustum.roll;
                break;
            }
        }
    }
    else
    {
        return false;
    }

    for(int i=0; i<num_serialized_sensors; ++i)
    {
        free(serialized_sensors[i].URI);
    }

    return true;
}

void
PresentationSceneUpdate(void)
{
    _UpdateProjectionMatrix();

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

    InputState *input = Input();

    V3 camera_movement = MakeV3(0, 0, 0);
    camera_movement.x = (!!(bool)input->right) - (!!(bool)input->left);
    camera_movement.y = (!!(bool)input->up) - (!!(bool)input->down);
    camera_movement.z = (!!(bool)input->forward) - (!!(bool)input->back);

    if(MagnitudeSquaredV3(camera_movement) > 0)
    {
        camera_movement = ScaleV3(NormalizeV3(camera_movement), 100);
        MoveCamera(&presentation_cam, camera_movement);
    }

    if(input->left_mouse_button)
    {
        SDL_SetRelativeMouseMode(SDL_TRUE);
        RotateCamera(&presentation_cam, -Input()->mouse_delta.x, Input()->mouse_delta.y);
    }
    else
    {
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }

    RendererSetViewMatrix(CameraGetViewMatrix(&presentation_cam));

    V3 color = (V3){ 1, 1, 1 };

    DepthPixel *depth_frame = GetSensorDepthFrame(&active_sensor.sensor);

    const size_t w = active_sensor.sensor.depth_stream_info.width;
    const size_t h = active_sensor.sensor.depth_stream_info.height;
    const float fov = active_sensor.sensor.depth_stream_info.fov;
    const float aspect = active_sensor.sensor.depth_stream_info.aspect_ratio;

    size_t num_points = 0;
    for(size_t y=0; y<h; ++y)
    for(size_t x=0; x<w; ++x)
    {
        float depth = depth_frame[x+y*w];
        if(depth > 0.0f)
        {
            float pos_x = tanf((((float)x / (float)w)-0.5f)*fov) * depth;
            float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;
            V3 point = (V3){ pos_x, pos_y, depth };
            active_sensor.point_cloud[num_points++] = point;
        }
    }

    RenderCubes(active_sensor.point_cloud, num_points, active_sensor.frustum.position,
                (V3){ active_sensor.frustum.pitch, active_sensor.frustum.yaw, active_sensor.frustum.roll },
                color);

    if(active_sensor.show_frustum)
    {
        RenderFrustum(&active_sensor.frustum);
    }
}

void
PresentationSceneEnd(void)
{
    for(int i=0; i<num_active_sensors; ++i)
    {
        SaveSensor(active_sensor.sensor.URI, &active_sensors[i].frustum);
        SensorFinalize(&active_sensor.sensor);
        free(active_sensor.point_cloud);
    }

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

