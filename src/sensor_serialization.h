#ifndef SENSOR_SERIALIZATION_H_
#define SENSOR_SERIALIZATION_H_

#include "frustum.h"

typedef struct
{
    char *URI;
    Frustum frustum;
} SerializedSensor;

void SaveSensor(const char *uri, Frustum *frustum);
int LoadSensors(SerializedSensor *sensors, int max_sensors);

#endif /* end of include guard: SENSOR_SERIALIZATION_H_ */

