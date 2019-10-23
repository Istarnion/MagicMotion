#include <SDL.h>
#include "ui.h"
#include "renderer.cpp"
#include "frustum.h"
#include "files.cpp"
#include "input.cpp"
#include "camera.cpp"
#include "sensor_interface.cpp"

static void _UpdateProjectionMatrix(void);

int main(int num_args, char *args[])
{
    SDL_Init(SDL_INIT_EVERYTHING);
    SensorInitialize();

    RendererInit("Code", 800, 600);

    Frustum f;
    f.position = MakeV3(0, 0, 0);
    f.pitch = f.yaw = f.roll = 0;
    f.fov = GetSensorFOV();
    f.aspect = GetSensorAspectRatio();
    f.near_plane = 300;
    f.far_plane = 5000;

    Camera cam;
    cam.pitch = 0;
    cam.yaw = 0;
    cam.position = MakeV3(0, 0, 0);
    CameraLookAt(&cam, MakeV3(0, 0, 0));

    _UpdateProjectionMatrix();

    bool running = true;
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
        ImGui::SliderFloat("Frustum pitch", &f.pitch, 0, 2.0f*M_PI);
        ImGui::SliderFloat("Frustum yaw", &f.yaw, 0, 2.0f*M_PI);
        ImGui::SliderFloat("Frustum roll", &f.roll, 0, 2.0f*M_PI);
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

        RenderFrustum(&f);

        Frame frame = GetSensorFrame(false, false);
        const size_t half_width = frame.width/2;
        const size_t half_height = frame.height/2;
        V3 points[frame.width * frame.height];
        size_t num_points = 0;
        for(size_t y=0; y<frame.height; ++y)
        for(size_t x=0; x<frame.width; ++x)
        {
            float depth = frame.pixels[x+y*frame.width].depth;
            if(depth > 0.0f)
            {
                float pos_x = tanf((((float)x / (float)frame.width)-0.5f)*f.fov) * depth;
                float pos_y = tanf((0.5f-((float)y / (float)frame.height))*(f.fov/f.aspect)) * depth;
                points[num_points++] = (V3){ pos_x, pos_y, depth };
            }
        }

        RenderCubes(points, num_points, f.position, (V3){ f.pitch, f.yaw, f.roll });

        RendererDisplay();
    }

    RendererQuit();
    SensorFinalize();
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

