#include "scene_viewer.h"

#include <SDL.h>
#include "magic_motion.h"
#include "renderer.h"
#include "ui.h"
#include "utils.h"
#include "config.h"

namespace viewer
{
    #define MAX_BOXES 128
    #define MAX_PARTICLES 128

    typedef struct
    {
        Frustum frustum;
        bool colorize;
        bool show_frustum;
        V3 *point_cloud;
        V3 *colors;
    } SensorRenderData;

    typedef struct
    {
        V3 position;
        V3 size;
    } Box;

    typedef struct
    {
        V3 position;
        V3 velocity;
        float time_to_live;
    } Particle;

    static const V3 colors[] = {
        (V3){ 1, 0, 0 },
        (V3){ 0, 1, 0 },
        (V3){ 0, 0, 1 }
    };

    static int num_active_sensors;
    static SensorRenderData active_sensors[MAX_SENSORS];
    static Box boxes[MAX_BOXES];
    static int num_boxes;

    static Particle particles[MAX_PARTICLES];

    static Camera cam;

    void
    ParticleBurst(V3 origin)
    {
        int start_index = 0;
        for(int i=0; i<MAX_PARTICLES; ++i)
        {
            if(particles[i].time_to_live <= 0)
            {
                start_index = i;
                break;
            }
        }

        for(int i=start_index; i<start_index+16; ++i)
        {
            int index = i % MAX_PARTICLES;
            V3 dir = NormalizeV3(SubV3(cam.position, origin));
            dir.x += ((float)rand() / (float)RAND_MAX) - 0.5f;
            dir.y += ((float)rand() / (float)RAND_MAX) - 0.5f;
            dir.z *= 2.0f;
            particles[index].position = origin;
            particles[index].time_to_live = 2.0f;
            particles[index].velocity = ScaleV3(dir, 10.0f);
        }
    }

    bool
    SceneInit(void)
    {
        const Config *config = GetConfig();

        cam.pitch = 0;
        cam.yaw = 3.1415f;
        cam.position = MakeV3(0, 0, 0);
        CameraLookAt(&cam, MakeV3(0, 0, 0));

        num_active_sensors = MagicMotion_GetNumCameras();
        const Frustum *frustums = MagicMotion_GetCameraFrustums();
        for(int i=0; i<num_active_sensors; ++i)
        {
            active_sensors[i].frustum = frustums[i];
        }

        FILE *f = fopen(config->box_file, "r");
        if(f)
        {
            fscanf(f, "%d\n", &num_boxes);
            if(num_boxes > MAX_BOXES) num_boxes = MAX_BOXES;

            for(int i=0; i<num_boxes; ++i)
            {
                fscanf(f, "%f,%f,%f|%f,%f,%f\n",
                       &boxes[i].position.x, &boxes[i].position.y, &boxes[i].position.z,
                       &boxes[i].size.x, &boxes[i].size.y, &boxes[i].size.z);
            }
        }

        return true;
    }

    void
    SceneUpdate(float dt)
    {
        UpdateProjectionMatrix();

        if(ImGui::Begin("Sensors"))
        {
            for(int i=0; i<num_active_sensors; ++i)
            {
                ImGui::PushID(i);

                SensorRenderData *s = &active_sensors[i];
                if(ImGui::CollapsingHeader("Camera"))
                {
                    ImGui::InputFloat3("Position", (float *)&s->frustum.position);
                    ImGui::SliderFloat("Pitch", &s->frustum.pitch, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Yaw", &s->frustum.yaw, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Roll", &s->frustum.roll, 0, 2.0f*M_PI);
                    ImGui::Checkbox("Colorize", &s->colorize);
                    ImGui::Checkbox("Frustum", &s->show_frustum);

                    MagicMotion_SetCameraPosition(i, s->frustum.position);
                    MagicMotion_SetCameraRotation(i, s->frustum.pitch, s->frustum.yaw, s->frustum.roll);
                }

                ImGui::PopID();
            }
        }

        ImGui::End();

        if(ImGui::Begin("Boxes"))
        {
            for(int i=0; i<num_boxes; ++i)
            {
                ImGui::PushID(i);

                ImGui::InputFloat3("Position", (float *)&boxes[i].position);
                ImGui::InputFloat3("Size", (float *)&boxes[i].size);
                if(ImGui::Button("Mark"))
                {
                    ParticleBurst(boxes[i].position);
                }

                ImGui::Spacing();

                ImGui::PopID();
            }

            if(ImGui::Button("+"))
            {
                Box b = (Box){ (V3){ 0, 0, 0 }, (V3){ 1, 1, 1 } };
                MagicMotion_RegisterHitbox(b.position, b.size);
                boxes[num_boxes++] = b;
            }
        }

        ImGui::End();

        InputState *input = Input();

        FPSCamera(input, &cam, dt);

        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

        RenderCube((V3){ 0, 0, 0 }, (V3){ 1, 1, 1 });

        V3 *positions = MagicMotion_GetPositions();
        Color *colors = MagicMotion_GetColors();
        unsigned int point_cloud_size = MagicMotion_GetCloudSize();

        V3 fcolors[point_cloud_size];
        for(size_t i=0; i<point_cloud_size; ++i)
        {
            fcolors[i] = (V3){ colors[i].r / 255.0f, colors[i].g / 255.0f, colors[i].b / 255.0f };
        }

        RenderPointCloud(positions, fcolors, point_cloud_size, (V3){ 0, 0, 0 }, (V3){ 0, 0, 0 });

        for(int i=0; i<num_active_sensors; ++i)
        {
            if(active_sensors[i].show_frustum)
            {
                RenderFrustum(&active_sensors[i].frustum);
            }
        }

        for(int i=0; i<num_boxes; ++i)
        {
            RenderWireCube(boxes[i].position, boxes[i].size);
        }

        int num_hitbox_events = 0;
        MagicMotionHitboxEvent *events = MagicMotion_QueryHitboxes(&num_hitbox_events);
        for(int i=0; i<num_hitbox_events; ++i)
        {
            printf("Collision %s on hitbox %d\n", (events[i].enter?"enter":"exit"), events[i].hitbox);
            if(events[i].enter)
            {
                ParticleBurst(boxes[events[i].hitbox].position);
            }
        }

        for(int i=0; i<MAX_PARTICLES; ++i)
        {
            if(particles[i].time_to_live > 0)
            {
                particles[i].velocity = AddV3(particles[i].velocity, ScaleV3((V3){ 0, -20.0f, 0 }, dt));
                particles[i].position = AddV3(particles[i].position, ScaleV3(particles[i].velocity, dt));
                particles[i].time_to_live -= dt;
                RenderCube(particles[i].position, (V3){ 0.1f, 0.1f, 0.1f });
            }
        }
    }

    void
    SceneEnd(void)
    {
        FILE *f = fopen("boxes.ser", "w");
        if(f)
        {
            fprintf(f, "%d\n", num_boxes);

            for(int i=0; i<num_boxes; ++i)
            {
                fprintf(f, "%f,%f,%f|%f,%f,%f\n",
                        boxes[i].position.x, boxes[i].position.y, boxes[i].position.z,
                        boxes[i].size.x,     boxes[i].size.y,     boxes[i].size.z);
            }
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

