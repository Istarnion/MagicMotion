#include <SDL.h>
#include "ui.h"
#include "renderer.cpp"
#include "files.cpp"
#include "input.cpp"
#include "camera.cpp"
#include "sensor_interface.cpp"
#include "scene_viewer.cpp"
#include "sensor_serialization.cpp"
#include "octree.cpp"

int main(int num_args, char *args[])
{
    bool running = true;

    SDL_Init(SDL_INIT_EVERYTHING);
    InitializeSensorInterface();

    RendererInit("Code", 800, 600);

    Scene viewer_scene = GetViewerScene();
    Scene *current_scene = &viewer_scene;

    current_scene->Init();

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

        current_scene->Update();

        RendererDisplay();
    }

    current_scene->End();
    RendererQuit();
    SDL_Quit();
    return 0;
}

