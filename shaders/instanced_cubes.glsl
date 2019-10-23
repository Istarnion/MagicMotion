#if defined(VERTEX_SHADER)

layout(location=0) in vec3 v_position;
layout(location=1) in vec3 v_normal;

uniform mat4 MVP;
uniform vec3 Positions[512];

out vec3 normal;

void
main()
{
    normal = v_normal;
    vec3 pos = Positions[gl_InstanceID];
    gl_Position = MVP * vec4(v_position + pos, 1.0);
}

#else

in vec3 normal;

out vec4 color;

void
main()
{
    color = vec4(1.0, 1.0, 1.0, 1.0);
}

#endif

