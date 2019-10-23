#ifndef SENSOR_INTERFACE_H_
#define SENSOR_INTERFACE_H_

typedef struct
{
    unsigned char r, g, b;
} RGBPixel;

typedef struct
{
    float r, g, b;
    float depth;
} Pixel;

#define INTENSITY(pixel) ((pixel).r * 0.333f + (pixel).g * 0.333f + (pixel).b * 0.333f)

typedef struct
{
    int width;
    int height;
    Pixel *pixels;
} Frame;

int SensorInitialize(void);
void SensorFinalize(void);
void GetSensorResolution(int *width, int *height);
float GetSensorFOV(void);
float GetSensorAspectRatio(void);
Frame GetSensorFrame(bool patch, bool record);

#endif /* end of include guard: SENSOR_INTERFACE_H_ */
