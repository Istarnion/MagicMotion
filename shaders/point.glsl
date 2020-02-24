#if defined(VERTEX_SHADER)

layout(location=0) in vec3 v_position;
layout(location=1) in vec3 v_color;

uniform mat4 MVP;

out vec3 point_color;

float
map(float value, float min1, float max1, float min2, float max2) {
    float t = (value-min1) / (max1-min1);
    return clamp(min2*(1.0-t) + max2*t, min(min2, max2), max(min2, max2));
}

void
main()
{
    point_color = v_color;
    gl_Position = MVP * vec4(v_position, 1.0);
    gl_PointSize = 3.0; //map(gl_Position.z, 10.0, 5000.0, 50.0, 2.0);
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

