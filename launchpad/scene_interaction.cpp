#include "scene_interaction.h"

#include "sensor_interface.h"
#include "renderer.h"
#include "octree.h"
#include "ui.h"
#include "utils.h"

namespace interaction
{
    typedef struct
    {
        SensorInfo *sensor;
        Frustum frustum;
        bool show_frustum;
        V3 *point_cloud;
    } SensorRenderData;

    static int num_active_sensors;
    static SensorInfo sensor_list[MAX_SENSORS];
    static SensorRenderData active_sensors[MAX_SENSORS];

    static Camera cam;

    static bool cube_touched_last_frame = false;
    static Octree octree;

    static V3 cloud_colors[] = {
        { 1.0, 1.0, 1.0 },
        { 1.0, 0.5, 0.5 },
        { 0.5, 1.0, 0.5 },
        { 0.5, 0.5, 1.0 }
    };
    static int cloud_color_index = 0;

    bool
    SceneInit(void)
    {
        cam.pitch = 0.187f;
        cam.yaw = 0.113f;
        cam.position = MakeV3(-500, 1000, -1300);
        CameraLookAt(&cam, MakeV3(0, 0, 0));

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

        InputState *input = Input();
        FPSCamera(input, &cam);

        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

        int max_total_point_cloud_size = 0;
        for(int i=0; i<num_active_sensors; ++i)
        {
            SensorRenderData *s = &active_sensors[i];
            max_total_point_cloud_size += s->sensor->depth_stream_info.width *
                                          s->sensor->depth_stream_info.height;
        }

        ResetOctree(&octree, max_total_point_cloud_size, 1000000);

        static V3 cube_center = (V3){ 0, 0, 2500 };
        static V3 cube_size = (V3){ 250, 250, 250 };
        bool cube_touched = false;

        for(int i=0; i<num_active_sensors; ++i)
        {
            SensorRenderData *s = &active_sensors[i];

            DepthPixel *depth_frame = GetSensorDepthFrame(s->sensor);

            const size_t w = s->sensor->depth_stream_info.width;
            const size_t h = s->sensor->depth_stream_info.height;
            const float fov = s->sensor->depth_stream_info.fov;
            const float aspect = s->sensor->depth_stream_info.aspect_ratio;

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
                    s->point_cloud[num_points++] = point;

                }
            }

            AddPointsToOctree(s->point_cloud, num_points, &octree);

            RenderCubes(s->point_cloud, num_points,
                        s->frustum.position,
                        (V3){ s->frustum.pitch,
                              s->frustum.yaw,
                              s->frustum.roll },
                        cloud_colors[cloud_color_index]);

            if(s->show_frustum)
            {
                RenderFrustum(&s->frustum);
            }
        }

        RenderWireCube(cube_center, cube_size);

        cube_touched = CheckBoxCollision(&octree, cube_center, cube_size);
        if(cube_touched && !cube_touched_last_frame)
        {
            cloud_color_index = (cloud_color_index + 1) % 4;
        }

        cube_touched_last_frame = cube_touched;
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
GetInteractionScene(void)
{
    Scene result;
    result.Init = &interaction::SceneInit;
    result.Update = &interaction::SceneUpdate;
    result.End = &interaction::SceneEnd;

    return result;
}

