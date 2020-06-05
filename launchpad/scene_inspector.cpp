#include "scene_inspector.h"

namespace inspector
{
    static FILE *recording_file;
    static size_t *frame_offsets;
    static char recording_filename[128];
    static size_t frame_index;
    static size_t frame_count;

    static size_t cloud_size;
    static V3 *spatial_cloud;
    static ColorPixel *color_cloud;
    static MagicMotionTag *tag_cloud;
    static size_t *boxed_indices;
    static size_t num_boxed_indices;

    static Camera cam;

    enum BoxEffect
    {
        BOX_INVISIBLE,
        BOX_NEUTRAL,
        BOX_BACKGROUNDINATE,
        BOX_FOREGROUNDINATE
    };

    static struct
    {
        char tooltip[128];
        bool render_point_cloud;
        bool render_voxels;
        bool render_voxel_bounds;
        bool visualize_bgsub;
        bool remove_bg;

        BoxEffect box_effect;
        V3 box_position;
        V3 box_size;
    } UI;

    static bool
    _LoadRecording(const char *file)
    {
        recording_file = NULL;
        FILE *fd = fopen(file, "rb+");
        if(!fd || ferror(fd))
        {
            return false;
        }

        fseek(fd, -sizeof(size_t), SEEK_END);
        fread(&frame_count, sizeof(size_t), 1, fd);
        rewind(fd);

        size_t *offsets = (size_t *)realloc(frame_offsets, sizeof(size_t)*frame_count);
        if(!offsets)
        {
            printf("Failed to allocate %zu bytes for frame_offsets\n",
                    sizeof(size_t)*frame_count);
            fclose(fd);
            return false;
        }

        frame_offsets = offsets;

        for(int i=0; i<frame_count; ++i)
        {
            frame_offsets[i] = ftell(fd);

            size_t index, num_points;
            size_t offset = ftell(fd);
            fscanf(fd, "frame %zu %zu\n", &index, &num_points);
            if(index != i+1)
            {
                printf("Frame %d invalid header (%zu)\n", (i+1), index);
                fclose(fd);
                return false;
            }

            fseek(fd, offset, SEEK_SET);

            // Go to right after the newline. We do this because we can't trust
            // fscanf, apparently
            for(uint8_t c=0; c != '\n'; fread(&c, 1, 1, fd)) {}

            for(int j=0; j<3; ++j)
            {
                size_t compressed_size = 0;
                fread(&compressed_size, sizeof(size_t), 1, fd);
                fseek(fd, compressed_size, SEEK_CUR);
            }

            // Skip the newline
            fseek(fd, 1, SEEK_CUR);
        }

        recording_file = fd;

        return true;
    }

    static void *
    _LoadAndDecompressBuffer(FILE *fd, void *target_buffer, size_t target_buffer_size)
    {
        size_t compressed_size = 0;
        fread(&compressed_size, sizeof(size_t), 1, fd);

        uint8_t *compressed_buffer = (uint8_t *)malloc(compressed_size);
        size_t bytes_read = fread(compressed_buffer, 1, compressed_size, fd);
        if(bytes_read != compressed_size)
        {
            printf("Failed to read compressed data\n");
            free(compressed_buffer);
            return NULL;
        }

        bytes_read = tinfl_decompress_mem_to_mem(target_buffer, target_buffer_size,
                                                 compressed_buffer, compressed_size, 0);

        free(compressed_buffer);

        if(bytes_read != target_buffer_size)
        {
            return NULL;
        }

        return target_buffer;
    }

    static void
    _LoadFrame(size_t index)
    {
        assert(recording_file);
        fseek(recording_file, frame_offsets[index], SEEK_SET);

        size_t i, num_points;
        size_t offset = ftell(recording_file);
        if(fscanf(recording_file, "frame %zu %zu\n", &i, &num_points) != 2)
        {
            printf("Frame %zu invalid header\n", (index+1));
            return;
        }

        if(i != (index+1))
        {
            printf("Frame %zu has invalid frame number (%zu)\n", index, i);
            return;
        }

        fseek(recording_file, offset, SEEK_SET);
        for(uint8_t c=0; c != '\n'; fread(&c, 1, 1, recording_file)) {}

        // NOTE(istarnion): This is unsafe. Should do reallocs safer later
        spatial_cloud = (V3 *)realloc(spatial_cloud, sizeof(V3)*num_points);
        color_cloud = (ColorPixel *)realloc(color_cloud, sizeof(ColorPixel)*num_points);
        tag_cloud = (MagicMotionTag *)realloc(tag_cloud, sizeof(MagicMotionTag)*num_points);
        boxed_indices = (size_t *)realloc(boxed_indices, sizeof(size_t)*num_points);

        _LoadAndDecompressBuffer(recording_file, spatial_cloud, sizeof(V3)*num_points);
        _LoadAndDecompressBuffer(recording_file, color_cloud, sizeof(ColorPixel)*num_points);
        _LoadAndDecompressBuffer(recording_file, tag_cloud, sizeof(MagicMotionTag)*num_points);

        cloud_size = num_points;
    }

    static void
    _UpdateFile(size_t from_frame)
    {
        // Save the file anew, starting from the tag cloud
        // of from_frame.

        fseek(recording_file, 0, SEEK_END);
        size_t end_pos = ftell(recording_file);

        // If from_frame is not the last frame, read everything that
        // follows into a temp buffer so we can write it back later
        size_t tail_size = 0;
        void *tail = NULL;
        if(from_frame < (frame_count-1))
        {
            fseek(recording_file, frame_offsets[from_frame+1], SEEK_SET);
            size_t next_frame_start = ftell(recording_file);

            tail_size = end_pos - next_frame_start;
            printf("Tail size: %zu\n", tail_size);
            tail = malloc(tail_size);
            fread(tail, 1, tail_size, recording_file);
        }

        // Find where the tag_cloud of from_frame is stored
        // First, skip the frame header
        fseek(recording_file, frame_offsets[from_frame], SEEK_SET);
        for(uint8_t c=0; c != '\n'; fread(&c, 1, 1, recording_file)) {}

        // Then, skip spatial and color
        for(int i=0; i<2; ++i)
        {
            size_t compressed_size = 0;
            fread(&compressed_size, sizeof(size_t), 1, recording_file);
            fseek(recording_file, compressed_size, SEEK_CUR);
        }

        size_t compressed_size = 0;
        void *compressed_tags = tdefl_compress_mem_to_heap(tag_cloud,
                sizeof(MagicMotionTag)*cloud_size, &compressed_size, 0);

        fwrite(&compressed_size, sizeof(size_t), 1, recording_file);
        fwrite(compressed_tags, 1, compressed_size, recording_file);
        uint8_t newline = '\n';
        fwrite(&newline, 1, 1, recording_file);

        mz_free(compressed_tags);

        if(tail)
        {
            fwrite(tail, 1, tail_size, recording_file);
            free(tail);
        }
    }

    bool
    SceneInit(void)
    {
        cam.pitch = 0;
        cam.yaw = 0;
        cam.position = MakeV3(0, 0, 0);
        CameraLookAt(&cam, MakeV3(0, 0, 0));

        spatial_cloud = NULL;
        color_cloud = NULL;
        tag_cloud = NULL;

        // Defaults
        UI.render_point_cloud = true;
        UI.render_voxels = false;
        UI.render_voxel_bounds = true;
        UI.visualize_bgsub = false;
        UI.remove_bg = false;

        UI.box_size = (V3){ 1, 1, 1 };

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

        if(frame_count > 0)
        {
            if(ImGui::Button("<"))
            {
                if(frame_index == 0)
                {
                    frame_index = frame_count;
                }

                _UpdateFile(frame_index);
                --frame_index;
                _LoadFrame(frame_index);
            }

            ImGui::Text("%04zu/%04zu", frame_index, frame_count);

            if(ImGui::Button(">"))
            {
                _UpdateFile(frame_index);

                ++frame_index;

                if(frame_index >= frame_count)
                {
                    frame_index = 0;
                }

                _LoadFrame(frame_index);
            }

            float f = (float)frame_index / (float)frame_count;
            ImGui::PushItemWidth(250);
            if(ImGui::SliderFloat("##scrub", &f, 0, 1.0f))
            {

                size_t old_frame_index = frame_index;
                frame_index = (size_t)(f * frame_count);
                if(frame_index != old_frame_index)
                {
                    _UpdateFile(old_frame_index);
                    if(frame_index >= frame_count) frame_index = frame_count-1;
                    _LoadFrame(frame_index);
                }
            }
        }
        else
        {
            ImGui::Button("<");
            ImGui::Text("0000/0000");
            ImGui::Button(">");

            float f = 0.5f;
            ImGui::PushItemWidth(250);
            ImGui::SliderFloat("##scrub", &f, 0, 1.0f);
        }

        if(ImGui::InputText("File", recording_filename, 127, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if(_LoadRecording(recording_filename))
            {
                frame_index = 0;
                _LoadFrame(frame_index);
            }
            else
            {
                strncpy(UI.tooltip, "Failed to load recording", 127);
            }
        }

        ImGui::PopItemWidth();

        if(ImGui::Button("Load"))
        {
            if(_LoadRecording(recording_filename))
            {
                frame_index = 0;
                _LoadFrame(frame_index);
            }
            else
            {
                strncpy(UI.tooltip, "Failed to load recording", 127);
            }
        }

        if(ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Point Cloud", NULL, &UI.render_point_cloud);
            ImGui::MenuItem("Voxel Bounds", NULL, &UI.render_voxel_bounds);
            ImGui::MenuItem("Visualize BG sub", NULL, &UI.visualize_bgsub);

            ImGui::EndMenu();
        }

        ImGui::Text("%s", UI.tooltip);

        ImGui::EndMainMenuBar();

        if(ImGui::Begin("Boxinator"))
        {
            if(UI.box_effect != BOX_INVISIBLE)
            {
                Mat4 *view = RendererGetViewMatrix();
                Mat4 *proj = RendererGetProjectionMatrix();
                ImGuiIO &io = ImGui::GetIO();
                ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                Mat4 box_transform = TranslationMat4(UI.box_position);
                ImGuizmo::Manipulate(view->v, proj->v,
                                     ImGuizmo::TRANSLATE, ImGuizmo::WORLD,
                                     box_transform.v);
                DecomposeMat4(box_transform, &UI.box_position, NULL, NULL);


                float size_delta = input->mouse_scroll * 1.0f;
                float box_size = UI.box_size.x + size_delta;
                if(box_size < 0.1f) box_size = 0.1f;
                else if(box_size > 100) box_size = 100;
                UI.box_size = (V3){ box_size, box_size, box_size };
            }

            V3 min = SubV3(UI.box_position, ScaleV3(UI.box_size, 0.5));
            V3 max = AddV3(UI.box_position, ScaleV3(UI.box_size, 0.5));

            num_boxed_indices = 0;
            for(size_t i=0; i<cloud_size; ++i)
            {
                V3 p = spatial_cloud[i];
                if(!(p.x < min.x || p.x > max.x ||
                     p.y < min.y || p.y > max.y ||
                     p.z < min.z || p.z > max.z))
                {
                    boxed_indices[num_boxed_indices++] = i;
                }
            }

            ImGui::Text("Contained points: %zu", num_boxed_indices);
            ImGui::RadioButton("Neutral", (int *)&UI.box_effect, (int)BOX_NEUTRAL);
            ImGui::RadioButton("Foregroundinate", (int *)&UI.box_effect, (int)BOX_FOREGROUNDINATE);
            ImGui::RadioButton("Backgroundinate", (int *)&UI.box_effect, (int)BOX_BACKGROUNDINATE);
        }
        else
        {
            UI.box_effect = BOX_INVISIBLE;
        }

        ImGui::End();

        if(UI.box_effect != BOX_INVISIBLE)
        {
            if(UI.box_effect == BOX_NEUTRAL)
            {
                RenderWireCube(UI.box_position, UI.box_size);
            }
            else
            {
                for(size_t i=0; i<num_boxed_indices; ++i)
                {
                    tag_cloud[boxed_indices[i]] =
                        (UI.box_effect == BOX_BACKGROUNDINATE) ? TAG_BACKGROUND :
                                                                 TAG_FOREGROUND;
                }

                V3 box_color = (UI.box_effect == BOX_BACKGROUNDINATE) ? (V3){ 1, 0, 0 }:
                                                                        (V3){ 0, 1, 0 };
                RenderColoredCube(UI.box_position, ScaleV3(UI.box_size, 2), box_color);
            }
        }

        if(UI.render_voxel_bounds)
        {
            RenderWireCube((V3){ 0, 0, 0 },
                           (V3){ BOUNDING_BOX_X, BOUNDING_BOX_Y, BOUNDING_BOX_Z });
        }

        if(UI.render_point_cloud)
        {
            V3 *fcolors = (V3 *)malloc(cloud_size * sizeof(V3));

            if(UI.visualize_bgsub)
            {
                for(size_t i=0; i<cloud_size; ++i)
                {
                    float r = 1.0f,
                          g = 1.0f,
                          b = 1.0f;

                    if(tag_cloud[i] & TAG_FOREGROUND)
                    {
                        r = 0.5f;
                        g = 2.0f;
                        b = 0.5f;
                    }
                    else if(tag_cloud[i] & TAG_BACKGROUND)
                    {
                        r = 2.0f;
                        g = 0.5f;
                        b = 0.5f;
                    }

                    fcolors[i] = (V3){
                        (color_cloud[i].r / 255.0f) * r,
                        (color_cloud[i].g / 255.0f) * g,
                        (color_cloud[i].b / 255.0f) * b
                    };
                }
            }
            else
            {
                for(size_t i=0; i<cloud_size; ++i)
                {
                    fcolors[i] = (V3){
                        (color_cloud[i].r / 255.0f),
                        (color_cloud[i].g / 255.0f),
                        (color_cloud[i].b / 255.0f)
                    };
                }
            }

            RenderPointCloud(spatial_cloud, fcolors, cloud_size);

            free(fcolors);
        }
    }

    void
    SceneEnd(void)
    {
        _UpdateFile(frame_index);

        free(spatial_cloud);
        free(color_cloud);
        free(tag_cloud);
        free(boxed_indices);
    }
}

Scene
GetInspectorScene(void)
{
    Scene result;
    result.Init = &inspector::SceneInit;
    result.Update = &inspector::SceneUpdate;
    result.End = &inspector::SceneEnd;
    return result;
}

