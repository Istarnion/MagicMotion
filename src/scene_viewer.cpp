#include "scene_viewer.h"

#include <SDL.h>
#include "sensor_interface.h"
#include "renderer.h"
#include "ui.h"
#include "sensor_serialization.h"
#include "utils.h"

namespace viewer
{
    typedef struct
    {
        SensorInfo *sensor;
        Frustum frustum;
        bool colorize;
        bool show_frustum;
        V3 *point_cloud;
    } SensorRenderData;

    static const V3 colors[] = {
        (V3){ 1, 0, 0 },
        (V3){ 0, 1, 0 },
        (V3){ 0, 0, 1 }
    };

    static int num_active_sensors;
    static SensorInfo sensor_list[MAX_SENSORS];
    static SensorRenderData active_sensors[MAX_SENSORS];

    static Camera cam;

    bool
    SceneInit(void)
    {
        cam.pitch = 0;
        cam.yaw = 0;
        cam.position = MakeV3(0, 0, 0);
        CameraLookAt(&cam, MakeV3(0, 0, 0));

        // We don't support plug and play of sensors
        // because it does not seem to be supported by
        // OpenNI2. This means the number of sensors will
        // be fixed at startup.

        SerializedSensor serialized_sensors[MAX_SENSORS];
        int num_serialized_sensors = LoadSensors(serialized_sensors, MAX_SENSORS);

        num_active_sensors = PollSensorList(sensor_list, MAX_SENSORS);
        for(int i=0; i<num_active_sensors; ++i)
        {
            SensorInfo *sensor = &sensor_list[i];
            active_sensors[i].sensor = sensor;
            int rc = SensorInitialize(sensor, false, true);
            if(rc)
            {
                fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
                return false;
            }

            active_sensors[i].frustum = (Frustum){
                .position = (V3){ 0, 0, 0 },
                .pitch = 0,
                .yaw = 0,
                .roll = 0,
                .fov = sensor->depth_stream_info.fov,
                .aspect = sensor->depth_stream_info.aspect_ratio,
                .near_plane = sensor->depth_stream_info.min_depth,
                .far_plane = sensor->depth_stream_info.max_depth
            };

            active_sensors[i].point_cloud = (V3 *)calloc(sensor->depth_stream_info.width * sensor->depth_stream_info.height,
                                                         sizeof(V3));

            active_sensors[i].colorize = false;

            for(int j=0; j<num_serialized_sensors; ++j)
            {
                if(strcmp(sensor->URI, serialized_sensors[j].URI) == 0)
                {
                    printf("Loading data for URI %s\n", sensor->URI);
                    active_sensors[i].frustum.position   = serialized_sensors[j].frustum.position;
                    active_sensors[i].frustum.pitch      = serialized_sensors[j].frustum.pitch;
                    active_sensors[i].frustum.yaw        = serialized_sensors[j].frustum.yaw;
                    active_sensors[i].frustum.roll       = serialized_sensors[j].frustum.roll;
                    break;
                }
            }
        }

        for(int i=0; i<num_serialized_sensors; ++i)
        {
            free(serialized_sensors[i].URI);
        }

        return true;
    }

    void
    SceneUpdate(void)
    {
        UpdateProjectionMatrix();

        if(ImGui::Begin("Sensors"))
        {
            for(int i=0; i<num_active_sensors; ++i)
            {
                ImGui::PushID(i);

                SensorRenderData *s = &active_sensors[i];
                if(ImGui::CollapsingHeader(s->sensor->name))
                {
                    ImGui::InputFloat3("Position", (float *)&s->frustum.position);
                    ImGui::SliderFloat("Pitch", &s->frustum.pitch, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Yaw", &s->frustum.yaw, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Roll", &s->frustum.roll, 0, 2.0f*M_PI);
                    ImGui::Checkbox("Colorize", &s->colorize);
                    ImGui::Checkbox("Frustum", &s->show_frustum);
                }

                ImGui::PopID();
            }
        }

        ImGui::End();

        InputState *input = Input();

        FPSCamera(input, &cam);

        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

        for(int i=0; i<num_active_sensors; ++i)
        {
            SensorRenderData *s = &active_sensors[i];
            V3 color = s->colorize ? colors[i % 3] : MakeV3(1, 1, 1);

            DepthPixel *depth_frame = GetSensorDepthFrame(s->sensor);

            const size_t w = s->sensor->depth_stream_info.width;
            const size_t h = s->sensor->depth_stream_info.height;
            const float fov = s->sensor->depth_stream_info.fov;
            const float aspect = s->sensor->depth_stream_info.aspect_ratio;

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
                    s->point_cloud[num_points++] = point;
                }
            }


            RenderCubes(s->point_cloud, num_points, s->frustum.position,
                        (V3){ s->frustum.pitch, s->frustum.yaw, s->frustum.roll },
                        color);

            if(s->show_frustum)
            {
                RenderFrustum(&s->frustum);
            }
        }
    }

    void
    SceneEnd(void)
    {
        for(int i=0; i<num_active_sensors; ++i)
        {
            SaveSensor(active_sensors[i].sensor->URI, &active_sensors[i].frustum);
            SensorFinalize(active_sensors[i].sensor);
            free(active_sensors[i].point_cloud);
        }
    }
}

Scene
GetViewerScene(void)
{
    Scene result;
    result.Init = &viewer::SceneInit;
    result.Update = &viewer::SceneUpdate;
    result.End = &viewer::SceneEnd;

    return result;
}

