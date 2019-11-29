#if defined(VERTEX_SHADER)

out vec2 uv;

void
main()
{
    float x = float(((uint(gl_VertexID) + 2u) / 3u)%2u);
    float y = float(((uint(gl_VertexID) + 1u) / 3u)%2u);

    uv = vec2(x, 1.0-y);

    gl_Position = vec4(2*(x-0.5), 2*(y-0.5), 0.0, 1.0);
}

#else

in vec2 uv;
uniform sampler2D tex;

out vec4 color;

void
main()
{
    color = texture(tex, uv);
}

#endif

