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

static SDL_Window *window;
static SDL_GLContext gl_context;
static int width;
static int height;
static Mat4 projection_matrix;
static Mat4 view_matrix;
static Mat4 projection_view_matrix;

static RenderData frustum_data;
static RenderData cube_data;

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

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
            0, 1, 2, 0, 2, 3, // Near face
            0, 3, 4, 4, 3, 7, // Top face
            3, 2, 7, 7, 2, 6, // East face
            1, 5, 2, 2, 5, 6, // Down face
            4, 5, 0, 0, 5, 6, // West face
            4, 6, 5, 4, 7, 6  // Far face
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

        glGenBuffers(1, &frustum_data.element_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frustum_data.element_buffer);

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
    }
}

void
RendererQuit(void)
{
    _DeleteRenderData(&frustum_data);
    _DeleteRenderData(&cube_data);

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

void
RenderCube(V3 center, V3 size)
{
    Mat4 model_matrix = TransformMat4(center, size, MakeV3(0, 0, 0));
    Mat4 mvp = MulMat4(model_matrix, projection_view_matrix);

    glEnable(GL_CULL_FACE);

    glBindVertexArray(cube_data.vertex_array);
    glUseProgram(cube_data.shader);
    glUniformMatrix4fv(cube_data.mvp_loc, 1, GL_FALSE, (float *)&mvp);

    glDrawElements(GL_TRIANGLES, 6*6, GL_UNSIGNED_SHORT, NULL);
}

void
RenderFrustum(const Frustum *frustum)
{
    float near = frustum->near_plane;
    float far = frustum->far_plane;
    float length_scale = far / near;
    V3 v = MakeNormalizedV3(near * tan(frustum->fov),
                            near * tan(frustum->fov * frustum->aspect),
                            near);

    V3 vertices[] = {
        // Near plane
        { -v.x,  v.y, v.z },
        { -v.x, -v.y, v.z },
        {  v.x, -v.y, v.z },
        {  v.x,  v.y, v.z },

        // Far plane
        { -v.x * length_scale,  v.y * length_scale, v.z * length_scale },
        { -v.x * length_scale, -v.y * length_scale, v.z * length_scale },
        {  v.x * length_scale, -v.y * length_scale, v.z * length_scale },
        {  v.x * length_scale,  v.y * length_scale, v.z * length_scale }
    };

    Mat4 model_matrix = TranslationMat4(frustum->position);
    Mat4 mvp = MulMat4(projection_view_matrix, model_matrix);

    glDisable(GL_CULL_FACE);

    glBindVertexArray(frustum_data.vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, frustum_data.vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STREAM_DRAW);

    glUseProgram(frustum_data.shader);
    glUniformMatrix4fv(frustum_data.mvp_loc, 1, GL_FALSE, (float *)&mvp);

    glDrawElements(GL_TRIANGLES, 6*6, GL_UNSIGNED_SHORT, NULL);
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
