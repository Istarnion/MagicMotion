#include "sensor_serialization.h"

#include <stdlib.h>
#include <stdio.h>

#define PERSIST_FILE "sensors.ser"

typedef enum
{
    PERSIST_FIELD_END,
    PERSIST_FIELD_SERIAL,
    PERSIST_FIELD_FRUSTUM_TRANS,
    PERSIST_FIELD_FRUSTUM_PLANES
} PersistFieldHeader;

void
SaveSensor(const char *serial, Frustum *frustum)
{
    FILE *f = NULL;

    // We want to overwrite the file each run, but still be able to
    // append sensors to it when running.
    // An alternative to this would be to take an array of SerializedSensor's
    // instead of the data of one.
    static bool first_sensor = true;
    if(first_sensor)
    {
        f = fopen(PERSIST_FILE, "w");
        first_sensor = false;
    }
    else
    {
        f = fopen(PERSIST_FILE, "a");
    }

    if(f)
    {
        float *t = frustum->transform.v;

        fprintf(f, "%d %s\n"
                   "%d %.3f %.3f %.3f %.3f "
                      "%.3f %.3f %.3f %.3f "
                      "%.3f %.3f %.3f %.3f "
                      "%.3f %.3f %.3f %.3f\n"
                   "%d %.3f %.3f\n"
                   "%d\n",
                PERSIST_FIELD_SERIAL, serial,
                PERSIST_FIELD_FRUSTUM_TRANS, t[0],   t[1],  t[2],  t[3],
                                             t[4],   t[5],  t[6],  t[7],
                                             t[8],   t[9], t[10], t[11],
                                             t[12], t[13], t[14], t[15],
                PERSIST_FIELD_FRUSTUM_PLANES, frustum->near_plane, frustum->far_plane,
                PERSIST_FIELD_END);

        fclose(f);
    }
    else
    {
        fprintf(stderr, "Failed to open %s for serializing sensor data\n", PERSIST_FILE);
    }
}

int
LoadSensors(SerializedSensor *sensors, int max_sensors)
{
    int sensors_read = -1;

    FILE *f = fopen(PERSIST_FILE, "r");
    if(f)
    {
        int eof = false;
        sensors_read = 0;

        fseek(f, 0, SEEK_END);
        long int file_length = ftell(f);
        rewind(f);

        if(file_length == 0)
        {
            eof = true;
        }

        for(int i=0; i<max_sensors && !eof; ++i)
        {
            SerializedSensor *sensor = &sensors[sensors_read];
            bool reading = true;

            while(reading)
            {
                char c;
                fread(&c, 1, 1, f);
                eof = feof(f);
                if(!eof)
                {
                    c -= '0';
                    switch((PersistFieldHeader)c)
                    {
                        case PERSIST_FIELD_SERIAL:
                        {
                            sensors_read++;
                            fscanf(f, " %63s\n", sensor->serial);
                            sensor->serial[63] = '\0';
                            printf("Read sensor serial: %s\n", sensor->serial);
                            break;
                        }
                        case PERSIST_FIELD_FRUSTUM_TRANS:
                        {
                            float *t = sensor->frustum.transform.v;
                            fscanf(f, " %f %f %f %f "
                                       "%f %f %f %f "
                                       "%f %f %f %f "
                                       "%f %f %f %f\n",
                                   &t[0],   &t[1],  &t[2],  &t[3],
                                   &t[4],   &t[5],  &t[6],  &t[7],
                                   &t[8],   &t[9], &t[10], &t[11],
                                   &t[12], &t[13], &t[14], &t[15]);
                            puts("Read frustum transform");
                            break;
                        }
                        case PERSIST_FIELD_FRUSTUM_PLANES:
                        {
                            fscanf(f, " %f %f\n",
                                   &sensor->frustum.near_plane,
                                   &sensor->frustum.far_plane);
                            puts("Read frustum planes");
                            break;
                        }
                        case PERSIST_FIELD_END:
                        {
                            puts("Read sensor end");
                            reading = false;
                        }
                        default: break;
                    }
                }
                else
                {
                    reading = false;
                }
            }
        }
    }

    return sensors_read;
}

