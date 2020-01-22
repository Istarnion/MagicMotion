#ifndef SENSOR_INTERFACE_H_
#define SENSOR_INTERFACE_H_

typedef struct _sensor Sensor;

typedef struct
{
    struct
    {
        int width;
        int height;
        float fov;
        float aspect_ratio;
    } color_stream_info;

    struct
    {
        int width;
        int height;
        float fov;
        float aspect_ratio;
        float min_depth;
        float max_depth;
    } depth_stream_info;

    char name[128];
    char URI[128];
    char vendor[128];
    Sensor *sensor_data;
} SensorInfo;

typedef struct
{
    unsigned char r, g, b;
} ColorPixel;

typedef float DepthPixel;

#define INTENSITY(pixel) ((pixel).r * 0.333f + (pixel).g * 0.333f + (pixel).b * 0.333f)
#define COLOR_TO_V3(c) (V3){ (c).r/255.0f, (c).g/255.0f, (c).b/255.0f }

void InitializeSensorInterface(void);
void FinalizeSensorInterface(void);
int PollSensorList(SensorInfo *sensor_list, int max_sensors);
int SensorInitialize(SensorInfo *sensor);
void SensorFinalize(SensorInfo *sensor);
ColorPixel *GetSensorColorFrame(SensorInfo *sensor);
DepthPixel *GetSensorDepthFrame(SensorInfo *sensor);

#endif /* end of include guard: SENSOR_INTERFACE_H_ */

