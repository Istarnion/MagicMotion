#if defined(VERTEX_SHADER)

layout(location=0) in vec3 v_position;
layout(location=1) in vec3 v_normal;

uniform mat4 MVP;
uniform vec3 Positions[256];
uniform vec3 Colors[256];

out vec3 normal;
out vec3 color;

void
main()
{
    normal = v_normal;
    vec3 pos = Positions[gl_InstanceID];
    color = Colors[gl_InstanceID];
    gl_Position = MVP * vec4(0.5*v_position + pos, 1.0);
}

#else

const vec3 light_dir = normalize(vec3(0.3, 3, -2));
const float gain = 1.2;

in vec3 normal;
in vec3 color;

out vec4 frag_color;

void
main()
{
    float light = max(0.1, dot(light_dir, normal));
    frag_color = vec4(color * light * gain, 1.0);
}

#endif

