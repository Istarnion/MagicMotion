#include <SDL.h>
#include "ui.h"
#include "renderer.cpp"
#include "files.cpp"
#include "input.cpp"
#include "camera.cpp"
#include "sensor_interface.cpp"
#include "sensor_serialization.cpp"
#include "octree.cpp"
#include "utils.h"

#include "scene_viewer.cpp"
#include "scene_video.cpp"
#include "scene_interaction.cpp"

enum SceneType
{
    SCENE_VIDEO,
    SCENE_VIEWER,
    SCENE_INTERACTION
};

int main(int num_args, char *args[])
{
    bool running = true;

    SDL_Init(SDL_INIT_EVERYTHING);
    InitializeSensorInterface();

    RendererInit("Code", 800, 600);

    SceneType scene_type = SCENE_VIEWER;
    Scene scene = GetViewerScene();

    scene.Init();

    const size_t max_num_frametime_samples = 512;
    Uint64 frametime_samples[max_num_frametime_samples];
    memset(frametime_samples, 0, sizeof(frametime_samples));
    size_t frametime_samples_head = 0;

    while(running)
    {
        Uint64 start_time = SDL_GetPerformanceCounter();

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
                    if(!io.WantCaptureKeyboard && e.key.repeat == 0)
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

        if(ImGui::Begin("Profiling"))
        {
            float frametimes_ms[max_num_frametime_samples];
            const double frequency = 1000.0 / SDL_GetPerformanceFrequency();
            float sum = 0.0f;
            float min_frametime = 1e10f;
            float max_frametime = 0.0f;

            for(size_t i=0; i<max_num_frametime_samples; ++i)
            {
                float frametime = (float)(frametime_samples[(frametime_samples_head+i)%max_num_frametime_samples]*frequency);
                frametimes_ms[i] = frametime;
                sum += frametime;
                if(frametime > max_frametime) max_frametime = frametime;
                if(frametime < min_frametime) min_frametime = frametime;
            }

            float average_frametime = sum / max_num_frametime_samples;
            ImGui::Text("Frametime avg: %.00f ms, min: %.00f ms, max: %.00f ms",
                        average_frametime, min_frametime, max_frametime);
            ImGui::PlotLines("##", frametimes_ms, max_num_frametime_samples);
        }

        ImGui::End();

        if(ImGui::Begin("Scenes"))
        {
            SceneType old_scene = scene_type;
            ImGui::RadioButton("Video", (int *)&scene_type, SCENE_VIDEO);
            ImGui::RadioButton("Viewer", (int *)&scene_type, SCENE_VIEWER);
            ImGui::RadioButton("Interaction", (int *)&scene_type, SCENE_INTERACTION);

            if(old_scene != scene_type)
            {
                scene.End();

                switch(scene_type)
                {
                    case SCENE_VIDEO:
                        scene = GetVideoScene();
                        break;
                    case SCENE_VIEWER:
                        scene = GetViewerScene();
                        break;
                    case SCENE_INTERACTION:
                        scene = GetInteractionScene();
                        break;
                    default: break;
                }

                scene.Init();
            }
        }

        ImGui::End();

        scene.Update();

        RendererDisplay();

        Uint64 end_time = SDL_GetPerformanceCounter();
        frametime_samples[frametime_samples_head++] = end_time - start_time;
        if(frametime_samples_head >= max_num_frametime_samples) frametime_samples_head = 0;
    }

    scene.End();
    FinalizeSensorInterface();
    RendererQuit();
    SDL_Quit();
    return 0;
}

