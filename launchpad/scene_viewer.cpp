#include "scene_viewer.h"

#ifdef HAS_OPENCV
#include <opencv2/bgsegm.hpp>
#include <opencv2/video/background_segm.hpp>
#endif

#include <SDL.h>
#include "magic_motion.h"
#include "renderer.h"
#include "ui.h"
#include "utils.h"
#include "config.h"
#include "video_recorder.h"

namespace viewer
{
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

    static const V3 colors[] = {
        (V3){ 1, 0, 0 },
        (V3){ 0, 1, 0 },
        (V3){ 0, 0, 1 }
    };

    static int num_active_sensors;
    static SensorRenderData active_sensors[MAX_SENSORS];
    static int selected_sensor;
    static ImGuizmo::MODE gizmo_mode;

    static Camera cam;

    static VideoRecorder *video_recorder;

    static void *sensor_preview;
    static void *ocv_test_texture;

    enum UIMode
    {
        UI_CAMERA
    };

    static struct
    {
        UIMode mode;
        bool video_window_open;
        char recording_filename_cloud[128];
        char recording_filename_video[128];
        bool is_recording;

        bool sensor_view_open;
        int camera_index;
        int color_feed;
        bool step_depth_image;

#ifdef HAS_OPENCV
        bool opencv_view_open;
        int ocv_camera_index;
        int ocv_color_feed;
        int bgsub_algo;
        cv::Ptr<cv::BackgroundSubtractor> ocv_subtractor;
#endif

        bool render_point_cloud;
        bool render_voxels;
        bool render_voxel_bounds;
        bool visualize_bgsub;
        bool remove_bg;
    } UI;

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

            // We can add nicks to cameras here by comparing serial numbers.
            // TODO(istarnion): Add a data driven way to do this, so we
            // won't need to edit source code and recompile
            nick = MagicMotion_GetCameraName(i);

            snprintf(active_sensors[i].name, 128, "%s (%s)",
                     nick, serial);
        }

        selected_sensor = 0;
        gizmo_mode = ImGuizmo::LOCAL;

        strcpy(UI.recording_filename_cloud, "recording_cloud.vid");
        strcpy(UI.recording_filename_video, "recording_video.vid");
        UI.render_voxels = false;
        UI.render_point_cloud = true;
        UI.visualize_bgsub = true;
        UI.render_voxel_bounds = true;

        return true;
    }

    void
    SceneUpdate(float dt)
    {
        MagicMotion_CaptureFrame();

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
            ImGui::MenuItem("Visualize BG sub", NULL, &UI.visualize_bgsub);
            ImGui::MenuItem("Subtract BG", NULL, &UI.remove_bg);

            ImGui::EndMenu();
        }

        if(ImGui::BeginMenu("Tools"))
        {
            if(ImGui::MenuItem("Camera Mode", NULL, UI.mode == UI_CAMERA))
            {
                UI.mode = UI_CAMERA;
            }

            ImGui::MenuItem("Video Recording", NULL, &UI.video_window_open);
            ImGui::MenuItem("Sensor View", NULL, &UI.sensor_view_open);
#ifdef HAS_OPENCV
            ImGui::MenuItem("OpenCV comparison", NULL, &UI.opencv_view_open);
#endif

            ImGui::EndMenu();
        }

        if(MagicMotion_IsCalibrating())
        {
            ImGui::PushStyleColor(ImGuiCol_Button, 0xFF0000);
            if(ImGui::Button("Stop"))
            {
                MagicMotion_EndCalibration();
            }

            ImGui::PopStyleColor();
        }
        else
        {
            if(ImGui::Button("Calibrate"))
            {
                MagicMotion_StartCalibration();
            }
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
                ImGuizmo::OPERATION gizmo_operation = ImGuizmo::TRANSLATE;

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
                if(ImGui::Button("Go To"))
                {
                    cam.position = (V3){ pos[0], pos[1], pos[2] };
                    V3 forward = MulMat4Vec3(s->frustum.transform, (V3){ 0, 0, 1 });
                    CameraLookAt(&cam, (V3){ pos[0]+forward.x,
                                             pos[1]+forward.y,
                                             pos[2]+forward.z });
                }

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

            ImGui::InputText("Cloud File", UI.recording_filename_cloud, 128);
            ImGui::InputText("Video File", UI.recording_filename_video, 128);

            if(!UI.is_recording)
            {
                if(ImGui::Button("Start recording"))
                {
                    video_recorder = StartVideoRecording(UI.recording_filename_cloud, UI.recording_filename_video, num_active_sensors, MagicMotion_GetSensorInfo());
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

        if(UI.sensor_view_open)
        {
            ImGui::Begin("Sensor View", &UI.sensor_view_open);

            if(sensor_preview)
            {
                RendererDestroyTexture(sensor_preview);
            }

            for(int i=0; i<num_active_sensors; ++i)
            {
                ImGui::PushID(i);
                ImGui::RadioButton(active_sensors[i].name, &UI.camera_index, i);
                ImGui::PopID();
            }

            ImGui::RadioButton("Color Feed", (int *)&UI.color_feed, 1);
            ImGui::RadioButton("Depth Feed", (int *)&UI.color_feed, 0);

            int width, height;
            if(UI.color_feed)
            {
                MagicMotion_GetColorImageResolution(UI.camera_index, &width, &height);
                uint32_t pixels[width * height];
                const ColorPixel *colors = MagicMotion_GetColorImage(UI.camera_index);
                for(int i=0; i<width*height; ++i)
                {
                    ColorPixel c = *colors++;

                    uint32_t p = (c.b << 16) |
                                 (c.g << 8)  |
                                 (c.r)       |
                                 0xFF000000;
                    pixels[i] = p;
                }

                sensor_preview = RendererCreateTexture(pixels, width, height);
            }
            else
            {
                MagicMotion_GetDepthImageResolution(UI.camera_index, &width, &height);
                uint32_t pixels[width * height];
                const float *depths = MagicMotion_GetDepthImage(UI.camera_index);
                for(int i=0; i<width*height; ++i)
                {
                    float d = (*depths++) / 5000.0f;
                    if(UI.step_depth_image)
                    {
                        d = floor(d / 100.0f) * 100.0f;
                    }

                    uint8_t v = 255 * d;
                    uint32_t p = (v << 16) |
                                 (v << 8)  |
                                 (v)       |
                                 0xFF000000;
                    pixels[i] = p;
                }
                sensor_preview = RendererCreateTexture(pixels, width, height);
            }

            ImGui::Image(sensor_preview, ImVec2(width, height));
            ImGui::Text("(%d x %d)", width, height);

            if(ImGui::Button("Save component textures"))
            {
                int color_width, color_height;
                MagicMotion_GetColorImageResolution(UI.camera_index, &color_width, &color_height);
                int num_color_pixels = color_width * color_height;
                uint32_t *color_pixel_buffer = (uint32_t *)malloc(num_color_pixels * 4);

                Image color_image;
                color_image.pixels = color_pixel_buffer;
                color_image.width = color_width;
                color_image.height = color_height;

                const ColorPixel *colors = MagicMotion_GetColorImage(UI.camera_index);

                for(int i=0; i<num_color_pixels; ++i)
                {
                    color_pixel_buffer[i] = (colors[i].r) | 0xFF000000;
                }

                WriteImage(&color_image, "capture_red.png");

                for(int i=0; i<num_color_pixels; ++i)
                {
                    color_pixel_buffer[i] = (colors[i].g << 8) | 0xFF000000;
                }

                WriteImage(&color_image, "capture_green.png");

                for(int i=0; i<num_color_pixels; ++i)
                {
                    color_pixel_buffer[i] = (colors[i].b << 16) | 0xFF000000;
                }

                WriteImage(&color_image, "capture_blue.png");

                free(color_pixel_buffer);

                int depth_width, depth_height;
                MagicMotion_GetDepthImageResolution(UI.camera_index, &depth_width, &depth_height);
                int num_depth_pixels = depth_width * depth_height;
                uint32_t *depth_pixel_buffer = (uint32_t *)malloc(num_depth_pixels * 4);

                Image depth_image;
                depth_image.pixels = depth_pixel_buffer;
                depth_image.width = depth_width;
                depth_image.height = depth_height;

                const float *depths = MagicMotion_GetDepthImage(UI.camera_index);

                for(int i=0; i<num_depth_pixels; ++i)
                {
                    float normalized_depth = depths[i] / 5000.0f;
                    uint8_t d = (uint8_t)((uint32_t)(normalized_depth * 255.0f) & 0xFF);
                    depth_pixel_buffer[i] = 0xFF000000 | (d << 16) | (d << 8) | d;
                }

                WriteImage(&depth_image, "capture_depth.png");
                free(depth_pixel_buffer);
            }

            ImGui::End();
        }

#ifdef HAS_OPENCV
        if(UI.opencv_view_open)
        {
            ImGui::Begin("OpenCV Comparison", &UI.opencv_view_open);

            for(int i=0; i<num_active_sensors; ++i)
            {
                ImGui::PushID(i);
                ImGui::RadioButton(active_sensors[i].name, &UI.ocv_camera_index, i);
                ImGui::PopID();
            }

            // Using bitwise OR instead of logical because we do NOT want
            // shortcutting here. If any of these options change we want to recreate
            // the subtractor.
            if((ImGui::RadioButton("Color Feed", &UI.ocv_color_feed, 1) |
                ImGui::RadioButton("Depth Feed", &UI.ocv_color_feed, 0) |

                ImGui::RadioButton("MOG2", &UI.bgsub_algo, 0) |
                ImGui::RadioButton("KNN",  &UI.bgsub_algo, 1) |
                ImGui::RadioButton("GSOC", &UI.bgsub_algo, 2)) ||
               !UI.ocv_subtractor)
            {
                switch(UI.bgsub_algo)
                {
                    case 0: // MOG2
                        UI.ocv_subtractor = cv::createBackgroundSubtractorMOG2(500, 16, false);
                        break;
                    case 1: // KNN
                        UI.ocv_subtractor = cv::createBackgroundSubtractorKNN(500, 400, false);
                        break;
                    case 2: // GSOC
                        UI.ocv_subtractor = cv::bgsegm::createBackgroundSubtractorGSOC();
                        break;
                }
            }

            if(ocv_test_texture)
            {
                RendererDestroyTexture(ocv_test_texture);
            }

            cv::Mat input_frame, output_frame;
            int width, height;

            if(UI.ocv_color_feed)
            {
                MagicMotion_GetColorImageResolution(UI.ocv_camera_index, &width, &height);
                const ColorPixel *pixels = MagicMotion_GetColorImage(UI.ocv_camera_index);
                float values[width*height];
                for(size_t i=0; i<width*height; ++i)
                {
                    ColorPixel c = pixels[i];
                    float value = (c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f) / 255.0f;
                    values[i] = value;
                }

                input_frame = cv::Mat(height, width, CV_32FC1, values);
            }
            else
            {
                MagicMotion_GetDepthImageResolution(UI.ocv_camera_index, &width, &height);
                const float *depth_pixels = MagicMotion_GetDepthImage(UI.ocv_camera_index);
                float values[width*height];
                memcpy(values, depth_pixels, width*height*sizeof(float));
                input_frame = cv::Mat(height, width, CV_32FC1, values);
            }

            UI.ocv_subtractor->apply(input_frame, output_frame);

            // Convert the 8-bit mask to 32-bit ARGB values
            uint32_t mask[width * height];
            output_frame.forEach<uint8_t>(
                [width, &mask](uint8_t &m, const int position[]) -> void
                {
                    int x = position[1], y = position[0];
                    uint32_t value = 0xFF000000 |
                                     (m << 16)  |
                                     (m << 8)   |
                                      m;

                    mask[x + y * width] = value;
                });

            ocv_test_texture = RendererCreateTexture(mask, width, height);
            ImGui::Image(ocv_test_texture, ImVec2(width, height));

            ImGui::End();
        }
#endif

        RenderCube((V3){ 0, 0, 0 }, (V3){ 1, 1, 1 });

        V3 *positions = MagicMotion_GetPositions();
        ColorPixel *colors = MagicMotion_GetColors();
        MagicMotionTag *tags = MagicMotion_GetTags();
        unsigned int point_cloud_size = MagicMotion_GetCloudSize();

        if(UI.render_point_cloud && point_cloud_size > 0)
        {
            // Stack allocating these is risky!
            V3 *fcolors = (V3 *)calloc(point_cloud_size, sizeof(V3));
            V3 *points = (V3 *)calloc(point_cloud_size, sizeof(V3));

            size_t actual_cloud_size = 0;
            for(size_t i=0; i<point_cloud_size; ++i)
            {
                /*
                remove bg | is foreground | should render (result)
                y           y               y
                n           y               y
                y           n               n
                n           n               y

                This shows that the only case we should _not_ render a point
                is when remove bg is true and the point does not have the
                foreground tag
                */
                if(!(UI.remove_bg && (tags[i] & TAG_FOREGROUND) == 0))
                {
                    points[actual_cloud_size] = positions[i];

                    if(UI.visualize_bgsub)
                    {
                        float r = 1.0f,
                              g = 1.0f,
                              b = 1.0f;

                        if(tags[i] & TAG_FOREGROUND)
                        {
                            r = 0.5f;
                            g = 2.0f;
                            b = 0.5f;
                        }
                        else if(tags[i] & TAG_BACKGROUND)
                        {
                            r = 2.0f;
                            g = 0.5f;
                            b = 0.5f;
                        }

                        fcolors[actual_cloud_size] = (V3){
                            (colors[i].r / 255.0f) * r,
                            (colors[i].g / 255.0f) * g,
                            (colors[i].b / 255.0f) * b
                        };
                    }
                    else
                    {
                        fcolors[actual_cloud_size] = (V3){
                            colors[i].r / 255.0f,
                            colors[i].g / 255.0f,
                            colors[i].b / 255.0f
                        };
                    }

                    ++actual_cloud_size;
                }
            }

            RenderPointCloud(points, fcolors, actual_cloud_size);
            free(points);
            free(fcolors);
        }

        if(UI.is_recording)
        {

            WriteVideoFrame(video_recorder, point_cloud_size, positions, colors, tags);
            for(int i=0; i<num_active_sensors; ++i)
            {
                int cw, ch, dw, dh;
                MagicMotion_GetColorImageResolution(i, &cw, &ch);
                MagicMotion_GetDepthImageResolution(i, &dw, &dh);

                const ColorPixel *colorpixels = MagicMotion_GetColorImage(i);
                const float *depthpixels = MagicMotion_GetDepthImage(i);

                AddVideoFrame(video_recorder, cw, ch, dw, dh, colorpixels, depthpixels);
            }
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
                        const int i = VOXEL_INDEX(x, y, z);
                        if(voxels[i].point_count > 8)
                        {
                            ColorPixel c = voxels[i].color;
                            voxel_colors[voxel_index] = (V3){
                                c.r / 255.0f,
                                c.g / 255.0f,
                                c.b / 255.0f
                            };

                            voxel_centers[voxel_index++] = VOXEL_TO_WORLD(i);

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

        {

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

