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
    gl_Position = MVP * vec4(0.5*v_position + pos, 1.0);
}

#else

const vec3 light_dir = normalize(vec3(0.3, 3, -2));

uniform vec3 Color;

in vec3 normal;

out vec4 color;

void
main()
{
    float light = max(0.1, dot(light_dir, normal));
    color = vec4(Color * light, 1.0);
}

#endif

