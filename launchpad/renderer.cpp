#include "renderer.h"

#include <assert.h>
#include <SDL.h>
#include "gl.h"
#include "ui.h"
#include "files.h"

typedef struct
{
    GLuint vertex_array;
    GLuint vertex_buffer;
    GLuint element_buffer;

    GLuint shader;
    GLint mvp_loc;
} RenderData;

typedef struct
{
    GLuint shader;
    GLint mvp_loc;
    GLint positions_loc;
    GLint colors_loc;
} RenderInstancedData;

static SDL_Window *window;
static SDL_GLContext gl_context;
static int width;
static int height;
static Mat4 projection_matrix;
static Mat4 view_matrix;
static Mat4 projection_view_matrix;

static RenderData frustum_data;
static RenderData cube_data;
static RenderData wire_cube_data;
static RenderData point_data;
static RenderInstancedData cube_instanced_data;
static GLuint full_quad_shader;

static GLuint _CreateShaderProgram(const char *source_file);
static void _DeleteRenderData(RenderData *data);

void
RendererInit(const char *title, int width, int height)
{
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL |
                                                     SDL_WINDOW_ALLOW_HIGHDPI |
                                                     SDL_WINDOW_RESIZABLE |
                                                     SDL_WINDOW_MAXIMIZED);
    window = SDL_CreateWindow(title,
                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               width, height,
                               window_flags);
    gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    load_gl_functions();

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);

    RendererUpdateSize();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init();

    projection_matrix = view_matrix = projection_view_matrix = IdentityMat4();

    // Initialize stuff for drawing frustums
    {
        glGenVertexArrays(1, &frustum_data.vertex_array);
        glBindVertexArray(frustum_data.vertex_array);

        glGenBuffers(1, &frustum_data.vertex_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, frustum_data.vertex_buffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &frustum_data.element_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frustum_data.element_buffer);

        static const GLushort indices[] = {
            0, 1, 2, 3, 0,
            4, 5, 6, 7, 4,
            5, 1, 2, 6, 7, 3
        };

        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        frustum_data.shader = _CreateShaderProgram("shaders/frustum.glsl");
        frustum_data.mvp_loc = glGetUniformLocation(frustum_data.shader, "MVP");
    }

    // Initialize stuff for drawing cubes
    {
        glGenVertexArrays(1, &cube_data.vertex_array);
        glBindVertexArray(cube_data.vertex_array);

        glGenBuffers(1, &cube_data.vertex_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, cube_data.vertex_buffer);

        GLfloat vertices[] = {
            // Front face
            -0.5f,  0.5f,  0.5f,   0.0f,  0.0f, -1.0f,
            -0.5f, -0.5f,  0.5f,   0.0f,  0.0f, -1.0f,
             0.5f, -0.5f,  0.5f,   0.0f,  0.0f, -1.0f,
             0.5f,  0.5f,  0.5f,   0.0f,  0.0f, -1.0f,
            // Top face
            -0.5f,  0.5f, -0.5f,   0.0f,  1.0f,  0.0f,
            -0.5f,  0.5f,  0.5f,   0.0f,  1.0f,  0.0f,
             0.5f,  0.5f,  0.5f,   0.0f,  1.0f,  0.0f,
             0.5f,  0.5f, -0.5f,   0.0f,  1.0f,  0.0f,
            // Bottom face
            -0.5f, -0.5f,  0.5f,   0.0f, -1.0f,  0.0f,
            -0.5f, -0.5f, -0.5f,   0.0f, -1.0f,  0.0f,
             0.5f, -0.5f, -0.5f,   0.0f, -1.0f,  0.0f,
             0.5f, -0.5f,  0.5f,   0.0f, -1.0f,  0.0f,
            // Left face
            -0.5f,  0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,
            -0.5f, -0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,
            -0.5f, -0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,
            -0.5f,  0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,
            // Right face
             0.5f,  0.5f,  0.5f,   1.0f,  0.0f,  0.0f,
             0.5f, -0.5f,  0.5f,   1.0f,  0.0f,  0.0f,
             0.5f, -0.5f, -0.5f,   1.0f,  0.0f,  0.0f,
             0.5f,  0.5f, -0.5f,   1.0f,  0.0f,  0.0f,
            // Back face
             0.5f,  0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
             0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
            -0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f,
            -0.5f,  0.5f, -0.5f,   0.0f,  0.0f, -1.0f
        };

        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*6, 0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)*6, (GLvoid *)(sizeof(float)*3));

        glGenBuffers(1, &cube_data.element_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_data.element_buffer);

        static const GLushort indices[] = {
             0,  1,  2,  0,  2,  3,
             4,  5,  6,  4,  6,  7,
             8,  9, 10,  8, 10, 11,
            12, 13, 14, 12, 14, 15,
            16, 17, 18, 16, 18, 19,
            20, 21, 22, 20, 22, 23
        };

        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        cube_data.shader = _CreateShaderProgram("shaders/cube.glsl");
        cube_data.mvp_loc = glGetUniformLocation(cube_data.shader, "MVP");

        cube_instanced_data.shader = _CreateShaderProgram("shaders/instanced_cubes.glsl");
        cube_instanced_data.mvp_loc = glGetUniformLocation(cube_instanced_data.shader, "MVP");
        cube_instanced_data.positions_loc = glGetUniformLocation(cube_instanced_data.shader, "Positions");
        cube_instanced_data.colors_loc = glGetUniformLocation(cube_instanced_data.shader, "Colors");

        // Then for the wire cube
        static const GLushort wire_indices[] = {
             0,  1,  1,  2,  2,  3,  3,  0,
             0,  4,  3,  7,  1, 22,  2, 21,
             7, 21, 21, 22, 22,  4,  4,  7
        };

        glGenVertexArrays(1, &wire_cube_data.vertex_array);
        glBindVertexArray(wire_cube_data.vertex_array);

        glGenBuffers(1, &wire_cube_data.vertex_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, wire_cube_data.vertex_buffer);

        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*6, 0);

        glGenBuffers(1, &wire_cube_data.element_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wire_cube_data.element_buffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(wire_indices), wire_indices, GL_STATIC_DRAW);

        wire_cube_data.shader = _CreateShaderProgram("shaders/wire_cube.glsl");
        wire_cube_data.mvp_loc = glGetUniformLocation(wire_cube_data.shader, "MVP");
    }

    {
        glGenVertexArrays(1, &point_data.vertex_array);
        glBindVertexArray(point_data.vertex_array);

        glGenBuffers(1, &point_data.vertex_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, point_data.vertex_buffer);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*3, 0);

        // We're abusing the element buffer member here to do non-interlaced position and color
        // Consider changing the RenderData struct to better suit the various needs
        glGenBuffers(1, &point_data.element_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, point_data.element_buffer);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)*3, 0);

        point_data.shader = _CreateShaderProgram("shaders/point.glsl");
        point_data.mvp_loc = glGetUniformLocation(cube_data.shader, "MVP");
    }

    {
        full_quad_shader = _CreateShaderProgram("shaders/full_quad.glsl");
    }
}

void
RendererQuit(void)
{
    _DeleteRenderData(&frustum_data);
    _DeleteRenderData(&cube_data);
    _DeleteRenderData(&point_data);

    glDeleteProgram(cube_instanced_data.shader);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyWindow(window);
}

void
RendererClear(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();
    ImGuizmo::BeginFrame();
}

void
RendererDisplay(void)
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
}

void
RendererUpdateSize(void)
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    width = viewport[2];
    height = viewport[3];
}

void
RendererGetSize(int *w, int *h)
{
    *w = width;
    *h = height;
}

void
RendererSetViewMatrix(Mat4 v)
{
    view_matrix = v;
    projection_view_matrix = MulMat4(view_matrix, projection_matrix);
}

void
RendererSetProjectionMatrix(Mat4 p)
{
    projection_matrix = p;
    projection_view_matrix = MulMat4(view_matrix, projection_matrix);
}

Mat4 *
RendererGetViewMatrix(void)
{
    return &view_matrix;
}

Mat4 *
RendererGetProjectionMatrix(void)
{
    return &projection_matrix;
}

void *
RendererCreateTexture(const void *pixels, int width, int height)
{
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D,
                 0, GL_RGBA, width, height,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    return (void *)(intptr_t)texture;
}

void
RendererDestroyTexture(void *texture)
{
    glBindTexture(GL_TEXTURE_2D, 0);
    GLuint tex = (GLuint)(intptr_t)texture;
    glDeleteTextures(1, &tex);
}

void
RenderWireCube(V3 center, V3 size)
{
    Mat4 model_matrix = TransformMat4(center, size, MakeV3(0, 0, 0));
    Mat4 mvp = MulMat4(model_matrix, projection_view_matrix);

    glBindVertexArray(wire_cube_data.vertex_array);
    glUseProgram(wire_cube_data.shader);
    glUniformMatrix4fv(wire_cube_data.mvp_loc, 1, GL_FALSE, (float *)&mvp);

    glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, NULL);
}

void
RenderCube(V3 center, V3 size)
{
    Mat4 model_matrix = TransformMat4(center, size, MakeV3(0, 0, 0));
    Mat4 mvp = MulMat4(model_matrix, projection_view_matrix);

    glBindVertexArray(cube_data.vertex_array);
    glUseProgram(cube_data.shader);
    glUniformMatrix4fv(cube_data.mvp_loc, 1, GL_FALSE, (float *)&mvp);

    glDrawElements(GL_TRIANGLES, 6*6, GL_UNSIGNED_SHORT, NULL);
}

void
RenderCubes(V3 *centers, V3 *colors, size_t num_cubes)
{
    glBindVertexArray(cube_data.vertex_array);
    glUseProgram(cube_instanced_data.shader);

    glUniformMatrix4fv(cube_instanced_data.mvp_loc, 1, GL_FALSE, (float *)&projection_view_matrix);

    // Draw in instanced batches of up to 512 cubes at a time
    for(size_t i=0; i<num_cubes; i+=256)
    {
        int num_cubes_to_draw = MIN(256, num_cubes-i);
        glUniform3fv(cube_instanced_data.positions_loc, num_cubes_to_draw, (GLfloat *)(centers+i));
        glUniform3fv(cube_instanced_data.colors_loc, num_cubes_to_draw, (GLfloat *)(colors+i));
        glDrawElementsInstanced(GL_TRIANGLES, 6*6, GL_UNSIGNED_SHORT, NULL, num_cubes_to_draw);
    }

    check_gl_errors();
}

void
RenderPointCloud(V3 *points, V3 *colors, size_t num_points)
{
    glBindVertexArray(point_data.vertex_array);
    glUseProgram(point_data.shader);

    glUniformMatrix4fv(point_data.mvp_loc, 1, GL_FALSE, (float *)&projection_view_matrix);

    glBindBuffer(GL_ARRAY_BUFFER, point_data.vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(V3)*num_points, points, GL_STREAM_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, point_data.element_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(V3)*num_points, colors, GL_STREAM_DRAW);

    glDrawArrays(GL_POINTS, 0, num_points);
}

void
RenderFrustum(const Frustum *frustum)
{
    float near = frustum->near_plane;
    float far = frustum->far_plane;
    float halfFovTanX = tanf(frustum->fov/2.0f);
    float halfFovTanY = tanf((frustum->fov / frustum->aspect)/2.0f);
    V3 vn = MakeV3(near * halfFovTanX,
                   near * halfFovTanY,
                   near);
    V3 vf = MakeV3(far * halfFovTanX,
                   far * halfFovTanY,
                   far);

    V3 vertices[] = {
        // Near plane
        { -vn.x,  vn.y, vn.z },
        { -vn.x, -vn.y, vn.z },
        {  vn.x, -vn.y, vn.z },
        {  vn.x,  vn.y, vn.z },

        // Far plane
        { -vf.x,  vf.y, vf.z },
        { -vf.x, -vf.y, vf.z },
        {  vf.x, -vf.y, vf.z },
        {  vf.x,  vf.y, vf.z }
    };

    Mat4 model_matrix = frustum->transform;
    Mat4 mvp = MulMat4(model_matrix, projection_view_matrix);

    glBindBuffer(GL_ARRAY_BUFFER, frustum_data.vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STREAM_DRAW);

    glBindVertexArray(frustum_data.vertex_array);
    glUseProgram(frustum_data.shader);
    glUniformMatrix4fv(frustum_data.mvp_loc, 1, GL_FALSE, (float *)&mvp);

    glDrawElements(GL_LINE_STRIP, 6*6, GL_UNSIGNED_SHORT, NULL);
}

void
RenderFullscreenQuad(void)
{
    glUseProgram(full_quad_shader);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

static GLint
_CreateShader(GLenum type, const char *code)
{
    const char *source[] = {
        "#version 330 core\n#define ",
        (type == GL_VERTEX_SHADER ? "VERTEX_SHADER\n" : "FRAGMENT_SHADER\n"),
        "#line 1\n",
        code
    };

    GLint shader = glCreateShader(type);
    glShaderSource(shader, 4, source, NULL);
    glCompileShader(shader);

    GLint compile_status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
    if(!compile_status)
    {
        GLchar info_log[1024];
        glGetShaderInfoLog(shader, 1024, NULL, info_log);
        fprintf(stderr, "Failed to compile %s%s\n", source[1], info_log);
        glDeleteShader(shader);
        shader = 0;
    }

    return shader;
}

static GLuint
_CreateShaderProgram(const char *source_file)
{
    GLuint shader = glCreateProgram();

    char *code = LoadTextFile(source_file);
    assert(code);
    GLuint vertex_shader = _CreateShader(GL_VERTEX_SHADER, code);
    GLuint fragment_shader = _CreateShader(GL_FRAGMENT_SHADER, code);
    FreeTextFile(code);

    assert(vertex_shader && fragment_shader);
    glAttachShader(shader, vertex_shader);
    glAttachShader(shader, fragment_shader);
    glLinkProgram(shader);

    GLint link_status;
    glGetProgramiv(shader, GL_LINK_STATUS, &link_status);
    if(!link_status)
    {
        GLchar info_log[1024];
        glGetProgramInfoLog(shader, 1024, NULL, info_log);
        fprintf(stderr, "Failed to link program%s\n", info_log);
        glDeleteShader(shader);
        shader = 0;
    }

    assert(shader);
    return shader;
}

static void
_DeleteRenderData(RenderData *data)
{
    glDeleteProgram(data->shader);
    glDeleteBuffers(1, &data->vertex_buffer);
    glDeleteBuffers(1, &data->element_buffer);
    glDeleteVertexArrays(1, &data->vertex_array);
}

