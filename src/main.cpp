#include <SDL.h>
#include "ui.h"
#include "renderer.cpp"
#include "frustum.h"
#include "files.cpp"
#include "input.cpp"
#include "camera.cpp"

static void _UpdateProjectionMatrix(void);

int main(int num_args, char *args[])
{
    SDL_Init(SDL_INIT_EVERYTHING);

    RendererInit("Code", 800, 600);

    Frustum f;
    f.position = MakeV3(0, 0, 0);
    f.pitch = RADIANS(90);
    f.yaw = f.roll = 0;
    f.fov = RADIANS(60);
    f.aspect = 1;
    f.near_plane = 300;
    f.far_plane = 5000;

    Camera cam;
    cam.pitch = 0;
    cam.yaw = M_PI;
    cam.position = MakeV3(0, 1000, 7500);
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
        ImGui::InputFloat("Frustim pitch", &f.pitch);
        ImGui::InputFloat("Frustim yaw", &f.yaw);
        ImGui::InputFloat("Frustim roll", &f.roll);
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

        // RenderCube(MakeV3(    0,  0,        0), MakeV3(100, 100, 100));
        RenderCube(MakeV3(    0,  0,     5000), MakeV3(100, 100, 100));
        RenderCube(MakeV3( 5000,  0,        0), MakeV3(100, 100, 100));
        RenderCube(MakeV3(    0,  0,    -5000), MakeV3(100, 100, 100));
        RenderCube(MakeV3(-5000,  0,        0), MakeV3(100, 100, 100));
        RenderCube(MakeV3(0,      5000,     0), MakeV3(100, 100, 100));
        RenderCube(MakeV3(0,     -5000,     0), MakeV3(100, 100, 100));

        RendererDisplay();
    }

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
    // RendererSetProjectionMatrix(OrthographicMat4(-1000, 1000, -1000*aspect, 1000*aspect, 100, 20000));
}

