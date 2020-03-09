#if defined(VERTEX_SHADER)

layout(location=0) in vec3 v_position;
layout(location=1) in vec3 v_color;

uniform mat4 MVP;

out vec3 point_color;

void
main()
{
    point_color = v_color;
    gl_Position = MVP * vec4(v_position, 1.0);
    gl_PointSize = max(1.0, 10.0 - gl_Position.z * 0.1);
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

