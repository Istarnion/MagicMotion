#if defined(VERTEX_SHADER)

uniform mat4 MVP;
uniform vec3 Positions[256];
uniform vec3 Color[256];

out vec3 point_color;

void
main()
{
    vec3 pos = Positions[gl_InstanceID];
    point_color = Color[gl_InstanceID];
    gl_Position = MVP * vec4(pos, 1.0);
}

#else

in vec3 point_color;

out vec4 color;

void
main()
{
    color = vec4(point_color, 1.0);
}

#endif

