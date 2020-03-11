#include "scene_viewer.h"

#include <SDL.h>
#include "magic_motion.h"
#include "renderer.h"
#include "ui.h"
#include "utils.h"
#include "config.h"
#include "video_recorder.h"

namespace viewer
{
    #define MAX_PARTICLES 128

    typedef enum
    {
        MANIPULATE_POSITION,
        MANIPULATE_ROTATION
    } SensorManipulationMode;

    typedef struct
    {
        char name[128];
        SensorManipulationMode manipulation_mode;
        Frustum frustum;
        bool colorize;
        bool show_frustum;
        V3 *point_cloud;
        V3 *colors;
    } SensorRenderData;

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
    static int selected_sensor;
    static ImGuizmo::MODE gizmo_mode;

    static Particle particles[MAX_PARTICLES];

    static Camera cam;

    static VideoRecorder *video_recorder;

    enum UIMode
    {
        UI_CAMERA
    };

    static struct
    {
        UIMode mode;
        bool video_window_open;
        char recording_filename[128];
        bool is_recording;

        bool render_point_cloud;
        bool render_voxels;
        bool render_voxel_bounds;
    } UI;

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
        cam.yaw = 0;
        cam.position = MakeV3(0, 0, 0);
        CameraLookAt(&cam, MakeV3(0, 0, 0));

        num_active_sensors = MagicMotion_GetNumCameras();
        const Frustum *frustums = MagicMotion_GetCameraFrustums();
        for(int i=0; i<num_active_sensors; ++i)
        {
            active_sensors[i].frustum = frustums[i];
            const char *serial = MagicMotion_GetCameraSerialNumber(i);
            const char *nick;

            // EXAMPLE
            /*
            if(strcmp(serial, "18110830920") == 0)
            {
                nick = "Fremst";
            }
            else
            {
                nick = "Astra";
            }

            snprintf(active_sensors[i].name, 128, "%s (%s)",
                     nick, serial);
            */
        }

        selected_sensor = 0;
        gizmo_mode = ImGuizmo::LOCAL;

        strcpy(UI.recording_filename, "recording.vid");
        UI.render_voxels = true;
        UI.render_voxel_bounds = true;

        return true;
    }

    void
    SceneUpdate(float dt)
    {
        UpdateProjectionMatrix();

        InputState *input = Input();
        FPSCamera(input, &cam, dt);
        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

        ImGui::BeginMainMenuBar();
        if(ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Point Cloud", NULL, &UI.render_point_cloud);
            ImGui::MenuItem("Voxels", NULL, &UI.render_voxels);
            ImGui::MenuItem("Voxel Bounds", NULL, &UI.render_voxel_bounds);

            ImGui::EndMenu();
        }

        if(ImGui::BeginMenu("Tools"))
        {
            if(ImGui::MenuItem("Camera Mode", NULL, UI.mode == UI_CAMERA))
            {
                UI.mode = UI_CAMERA;
            }

            ImGui::MenuItem("Video Recording", NULL, &UI.video_window_open);

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();

        Mat4 *view = RendererGetViewMatrix();
        Mat4 *proj = RendererGetProjectionMatrix();

        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        // Edit sensor stuff:
        if(UI.mode == UI_CAMERA)
        {
            if(ImGui::Begin("Scene"))
            {
                for(int i=0; i<num_active_sensors; ++i)
                {
                    ImGui::PushID(i);
                    ImGui::RadioButton(active_sensors[i].name, &selected_sensor, i);
                    ImGui::PopID();
                }
            }

            ImGui::End();

            SensorRenderData *s = &active_sensors[selected_sensor];
            if(ImGui::Begin("Inspector"))
            {
                ImGuizmo::OPERATION gizmo_operation;

                float pos[3];
                float rotation[3];
                float scale[3];

                ImGuizmo::DecomposeMatrixToComponents(s->frustum.transform.v,
                                                      pos, rotation, scale);

                if(s->manipulation_mode == MANIPULATE_POSITION)
                {
                    gizmo_operation = ImGuizmo::TRANSLATE;
                }
                else if(s->manipulation_mode == MANIPULATE_ROTATION)
                {
                    gizmo_operation = ImGuizmo::ROTATE;
                }

                ImGui::InputFloat3("Position", pos);
                ImGui::InputFloat3("Rotation", rotation);
                ImGui::RadioButton("Translate",
                                   (int *)&s->manipulation_mode,
                                   MANIPULATE_POSITION);

                ImGui::SameLine();

                ImGui::RadioButton("Rotate",
                                   (int *)&s->manipulation_mode,
                                   MANIPULATE_ROTATION);

                ImGui::RadioButton("Local", (int *)&gizmo_mode,
                                            (int)ImGuizmo::LOCAL);
                ImGui::SameLine();
                ImGui::RadioButton("World", (int *)&gizmo_mode,
                                            (int)ImGuizmo::WORLD);

                ImGui::Checkbox("Frustum", &s->show_frustum);

                ImGuizmo::RecomposeMatrixFromComponents(pos, rotation, scale,
                                                        s->frustum.transform.v);

                ImGuizmo::Manipulate(view->v, proj->v,
                                     gizmo_operation, gizmo_mode,
                                     s->frustum.transform.v);

                MagicMotion_SetCameraTransform(selected_sensor, s->frustum.transform);
            }

            ImGui::End();
        }

        if(UI.video_window_open)
        {
            ImGui::Begin("Video Recording", &UI.video_window_open);

            ImGui::InputText("File", UI.recording_filename, 128);

            if(!UI.is_recording)
            {
                if(ImGui::Button("Start recording"))
                {
                    video_recorder = StartVideoRecording(UI.recording_filename);
                    UI.is_recording = true;
                }
            }
            else
            {
                if(ImGui::Button("Stop recording"))
                {
                    StopRecording(video_recorder);
                    video_recorder = NULL;
                    UI.is_recording = false;
                }
            }

            ImGui::End();
        }

        RenderCube((V3){ 0, 0, 0 }, (V3){ 1, 1, 1 });

        V3 *positions = MagicMotion_GetPositions();
        Color *colors = MagicMotion_GetColors();
        unsigned int point_cloud_size = MagicMotion_GetCloudSize();

        if(UI.render_point_cloud)
        {
            V3 fcolors[point_cloud_size]; // NOTE(istarnion): Allocating this on the stack is risky!
            for(size_t i=0; i<point_cloud_size; ++i)
            {
                fcolors[i] = (V3){
                    colors[i].r / 255.0f,
                    colors[i].g / 255.0f,
                    colors[i].b / 255.0f
                };
            }

            RenderPointCloud(positions, fcolors, point_cloud_size);
        }

        if(UI.is_recording)
        {
            WriteVideoFrame(video_recorder, point_cloud_size, positions, colors);
        }


        if(UI.render_voxels)
        {
            Voxel *voxels = MagicMotion_GetVoxels();
            V3 voxel_centers[256];
            V3 voxel_colors[256];
            int voxel_index = 0;
            for(int z=0; z<NUM_VOXELS_Z; ++z)
            {
                for(int y=0; y<NUM_VOXELS_Y; ++y)
                {
                    for(int x=0; x<NUM_VOXELS_X; ++x)
                    {
                        const int i = x+y*NUM_VOXELS_X+z*NUM_VOXELS_X*NUM_VOXELS_Y;
                        if(voxels[i].point_count > 8)
                        {
                            Color c = voxels[i].color;
                            voxel_colors[voxel_index] = (V3){
                                c.r / 255.0f,
                                c.g / 255.0f,
                                c.b / 255.0f
                            };

                            voxel_centers[voxel_index++] = (V3){
                                (x - NUM_VOXELS_X/2) * VOXEL_SIZE,
                                (y - NUM_VOXELS_Y/2) * VOXEL_SIZE,
                                (z - NUM_VOXELS_Z/2) * VOXEL_SIZE
                            };

                            if(voxel_index >= 256)
                            {
                                RenderCubes(voxel_centers, voxel_colors, 256);
                                voxel_index = 0;
                            }
                        }
                    }
                }
            }

            if(voxel_index > 0)
            {
                RenderCubes(voxel_centers, voxel_colors, voxel_index);
            }
        }

        if(UI.render_voxel_bounds)
        {
            RenderWireCube((V3){ 0, 0, 0 },
                           (V3){ BOUNDING_BOX_X, BOUNDING_BOX_Y, BOUNDING_BOX_Z });
        }

        for(int i=0; i<num_active_sensors; ++i)
        {
            if(active_sensors[i].show_frustum)
            {
                RenderFrustum(&active_sensors[i].frustum);
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

