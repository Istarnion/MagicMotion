#include <SDL.h>
#include "ui.h"
#include "renderer.cpp"
#include "frustum.h"
#include "files.cpp"
#include "input.cpp"
#include "camera.cpp"
#include "sensor_interface.cpp"

typedef struct
{
    SensorInfo *sensor;
    Frustum frustum;
    V3 *point_cloud;
} SensorRenderData;

#define MAX_SENSORS 4

static const V3 colors[] = {
    (V3){ 1, 0, 0 },
    (V3){ 0, 1, 0 },
    (V3){ 0, 0, 1 }
};

static void _UpdateProjectionMatrix(void);

int main(int num_args, char *args[])
{
    bool running = true;

    SDL_Init(SDL_INIT_EVERYTHING);
    InitializeSensorInterface();

    RendererInit("Code", 800, 600);

    Camera cam;
    cam.pitch = 0;
    cam.yaw = 0;
    cam.position = MakeV3(0, 0, 0);
    CameraLookAt(&cam, MakeV3(0, 0, 0));

    _UpdateProjectionMatrix();

    // We don't support plug and play of sensors
    // because it does not seem to be supported by
    // OpenNI2. This means the number of sensors will
    // be fixed at startup.
    SensorInfo sensor_list[MAX_SENSORS];
    SensorRenderData active_sensors[MAX_SENSORS];

    int num_active_sensors = PollSensorList(sensor_list, MAX_SENSORS);
    for(int i=0; i<num_active_sensors; ++i)
    {
        SensorInfo *sensor = &sensor_list[i];
        active_sensors[i].sensor = sensor;
        int rc = SensorInitialize(sensor, false, true);
        if(rc)
        {
            fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
            running = false;
            break;
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
    }

    while(running)
    {
        InputNewFrame();
        SDL_Event e;
        while(SDL_PollEvent(&e))
        {
            ImGui_ImplSDL2_ProcessEvent(&e);
            ImGuiIO &io = ImGui::GetIO();

            switch(e.type)
            {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_WINDOWEVENT:
                    if(e.window.event == SDL_WINDOWEVENT_RESIZED ||
                       e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
                    {
                        RendererUpdateSize();
                        _UpdateProjectionMatrix();
                    }
                    break;
                case SDL_KEYDOWN:
                case SDL_KEYUP:
                    if(!io.WantCaptureKeyboard)
                    {
                        InputKeyEvent(e.type == SDL_KEYDOWN, e.key.keysym.scancode);
                    }
                    break;
                case SDL_MOUSEMOTION:
                    if(!io.WantCaptureMouse)
                    {
                        int width, height;
                        RendererGetSize(&width, &height);
                        float w = 2.0f / width;
                        float h = -2.0f / height;

                        InputMouseMotion(e.motion.x * w - 1.0f,    e.motion.y * h - 1.0f,
                                         e.motion.xrel / (float)width,
                                         e.motion.yrel / (float)height);
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                case SDL_MOUSEBUTTONUP:
                    if(!io.WantCaptureMouse)
                    {
                        InputMousePress(e.type == SDL_MOUSEBUTTONDOWN, e.button.button);
                    }
                    break;
                case SDL_MOUSEWHEEL:
                    if(!io.WantCaptureMouse)
                    {
                        int scroll = e.wheel.y;
                        InputMouseWheel(scroll);
                    }
                    break;
                default:
                    break;
            }
        }

        RendererClear();

        ImGui::Begin("Camera controll");
        ImGui::InputFloat3("Camera position", (float *)&cam.position);
        ImGui::InputFloat("Camera pitch", &cam.pitch);
        ImGui::InputFloat("Camera yaw", &cam.yaw);
        ImGui::End();

        if(ImGui::Begin("Sensors"))
        {
            for(int i=0; i<num_active_sensors; ++i)
            {
                SensorRenderData *s = &active_sensors[i];
                if(ImGui::CollapsingHeader(s->sensor->name))
                {
                    ImGui::PushID(i);
                    ImGui::InputFloat3("Position", (float *)&s->frustum.position);
                    ImGui::SliderFloat("Pitch", &s->frustum.pitch, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Yaw", &s->frustum.yaw, 0, 2.0f*M_PI);
                    ImGui::SliderFloat("Roll", &s->frustum.roll, 0, 2.0f*M_PI);
                    ImGui::PopID();
                }
            }
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
            MoveCamera(&cam, camera_movement);
        }

        if(input->left_mouse_button)
        {
            SDL_SetRelativeMouseMode(SDL_TRUE);
            RotateCamera(&cam, -Input()->mouse_delta.x, Input()->mouse_delta.y);
        }
        else
        {
            SDL_SetRelativeMouseMode(SDL_FALSE);
        }

        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

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
                float depth = depth_frame[x+y*w];
                if(depth > 0.0f)
                {
                    float pos_x = tanf((((float)x / (float)w)-0.5f)*fov) * depth;
                    float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;
                    s->point_cloud[num_points++] = (V3){ pos_x, pos_y, depth };
                }
            }

            RenderCubes(s->point_cloud, num_points, s->frustum.position,
                        (V3){ s->frustum.pitch, s->frustum.yaw, s->frustum.roll },
                        colors[i % 3]);
            RenderFrustum(&s->frustum);
        }

        RendererDisplay();
    }

    RendererQuit();

    for(int i=0; i<num_active_sensors; ++i)
    {
        SensorFinalize(active_sensors[i].sensor);
        free(active_sensors[i].point_cloud);
    }

    FinalizeSensorInterface();

    SDL_Quit();
    return 0;
}

static void
_UpdateProjectionMatrix()
{
    int w, h;
    RendererGetSize(&w, &h);
    float aspect = (float)w / (float)h;

    RendererSetProjectionMatrix(PerspectiveMat4(aspect, 45.0f, 10.0f, 100000.0f));
}

