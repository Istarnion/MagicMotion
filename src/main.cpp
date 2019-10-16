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
    f.position = MakeV3(0, 5000, 0);
    f.forward = MakeV3(0, -1, 0);
    f.up = MakeV3(0, 0, 1);
    f.fov = 60;
    f.aspect = 1;
    f.near_plane = 300;
    f.far_plane = 5000;

    Camera cam;
    cam.position = MakeV3(0, 500, 7500);
    cam.forward = MakeV3(0, 0, 1);
    cam.right = MakeV3(1, 0, 0);
    cam.up = MakeV3(0, 1, 0);
    cam.yaw = cam.pitch = 0;
    //CameraLookAt(&cam, MakeV3(0, 2500, 0));

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
                                         e.motion.xrel * w - 1.0f, e.motion.yrel * h - 1.0f);
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

        ImGui::Text("Hello World!");
        ImGui::InputFloat3("Camera position", (float *)&cam.position);
        ImGui::InputFloat3("Camera forward", (float *)&cam.forward);
        ImGui::InputFloat3("Camera right", (float *)&cam.right);
        ImGui::InputFloat3("Camera up", (float *)&cam.up);
        ImGui::InputFloat("Camera pitch", &cam.pitch);
        ImGui::InputFloat("Camera yaw", &cam.yaw);

        //RotateCamera(&cam, Input()->mouse_delta.x, Input()->mouse_delta.y);

        RendererSetViewMatrix(CameraGetViewMatrix(&cam));

        // RenderFrustum(&f);

        RenderCube(MakeV3(    0,  0,        0), MakeV3(100, 100, 100));
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

    RendererSetProjectionMatrix(PerspectiveMat4(aspect, 45.0f, 10.0f, 20000.0f));
    // RendererSetProjectionMatrix(OrthographicMat4(-1000, 1000, -1000*aspect, 1000*aspect, 100, 20000));
}
