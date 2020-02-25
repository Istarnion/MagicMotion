#include <string.h>

#ifdef SENSOR_REALSENSE
#include "sensor_interface_realsense.cpp"
#else
#include "sensor_interface_openni.cpp"
#endif

#include "sensor_serialization.cpp"

#include "magic_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_HITBOXES 128
#define MAX_HITBOX_EVENTS 32

// How many points needs to be inside a hitbox to be considered a hit?
#define HITBOX_POINT_TRESHOLD 16

typedef struct
{
    V3 position;
    V3 size;
    int point_count;
    bool considered_hit;
} Hitbox;

static struct
{
    SensorInfo sensors[MAX_SENSORS];
    Frustum sensor_frustums[MAX_SENSORS];
    unsigned int num_active_sensors;

    V3 *spatial_cloud;
    Color *color_cloud;
    MagicMotionTag *tag_cloud;
    unsigned int cloud_size;
    unsigned int cloud_capacity;

    Hitbox hitboxes[MAX_HITBOXES];
    unsigned int num_hitboxes;

    // TODO(istarnion): Consider changing this for an octtree or similar
    V3 boundingbox_min;
    V3 boundingbox_max;

    MagicMotionHitboxEvent hitbox_events[MAX_HITBOX_EVENTS];
    unsigned int num_hitbox_events;
} magic_motion;

static bool
MagicMotion_BoxVsV3(V3 box_pos, V3 box_size, V3 v)
{
    float hw = box_size.x / 2.0f;
    float hh = box_size.y / 2.0f;
    float hd = box_size.z / 2.0f;

    return !(v.x < box_pos.x - hw || v.x > box_pos.x + hw ||
             v.y < box_pos.y - hh || v.y > box_pos.y + hh ||
             v.z < box_pos.z - hd || v.z > box_pos.z + hd);
}

void
MagicMotion_Initialize(void)
{
    InitializeSensorInterface();

    SerializedSensor serialized_sensors[MAX_SENSORS];
    int num_serialized_sensors = LoadSensors(serialized_sensors, MAX_SENSORS);
    printf("Loaded %d sensor configs:\n", num_serialized_sensors);

    magic_motion.cloud_size = 0;
    magic_motion.cloud_capacity = 0;
    magic_motion.num_active_sensors = PollSensorList(magic_motion.sensors, MAX_SENSORS);
    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        int rc = SensorInitialize(sensor, true, true);
        if(rc)
        {
            fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
            return;
        }

        magic_motion.sensor_frustums[i] = (Frustum){
            .transform = IdentityMat4(),
            .fov = sensor->depth_stream_info.fov,
            .aspect = sensor->depth_stream_info.aspect_ratio,
            .near_plane = MAX(0.05f, sensor->depth_stream_info.min_depth / 100.0f),
            .far_plane = sensor->depth_stream_info.max_depth / 100.0f
        };

        size_t point_cloud_size = sensor->depth_stream_info.width * sensor->depth_stream_info.height;
        magic_motion.cloud_capacity += point_cloud_size;

        for(int j=0; j<num_serialized_sensors; ++j)
        {
            if(strcmp(sensor->serial, serialized_sensors[j].serial) == 0)
            {
                printf("Loading data for sensor %s:\n", sensor->serial);
                Frustum f = serialized_sensors[j].frustum;
                magic_motion.sensor_frustums[i].transform  = f.transform;
                break;
            }
            else
            {
                printf("Failed to load sensor config: %s did not match %s\n", sensor->serial, serialized_sensors[j].serial);
            }
        }
    }

    magic_motion.spatial_cloud = (V3 *)calloc(magic_motion.cloud_capacity, sizeof(V3));
    magic_motion.tag_cloud = (MagicMotionTag *)calloc(magic_motion.cloud_capacity, sizeof(MagicMotionTag));
    magic_motion.color_cloud = (Color *)calloc(magic_motion.cloud_capacity, sizeof(Color));

    FILE *f = fopen("boxes.ser", "r");
    if(f)
    {
        int num_boxes = 0;
        fscanf(f, "%d\n", &num_boxes);
        if(num_boxes > MAX_HITBOXES) num_boxes = MAX_HITBOXES;

        for(int i=0; i<num_boxes; ++i)
        {
            V3 pos;
            V3 size;
            fscanf(f, "%f,%f,%f|%f,%f,%f\n",
                   &pos.x,  &pos.y,  &pos.z,
                   &size.x, &size.y, &size.z);

            MagicMotion_RegisterHitbox(pos, size);
        }
    }

    printf("MagicMotion initialized with %u active sensors. Point cloud size: %u\n",
           magic_motion.num_active_sensors, magic_motion.cloud_capacity);
}

void
MagicMotion_Finalize(void)
{
    free(magic_motion.color_cloud);
    free(magic_motion.tag_cloud);
    free(magic_motion.spatial_cloud);

    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SaveSensor(magic_motion.sensors[i].serial, &magic_motion.sensor_frustums[i]);
        SensorFinalize(&magic_motion.sensors[i]);
    }

    FinalizeSensorInterface();
}

unsigned int
MagicMotion_GetNumCameras(void)
{
    return magic_motion.num_active_sensors;
}

const char *
MagicMotion_GetCameraName(unsigned int camera_index)
{
    return magic_motion.sensors[camera_index].name;
}

const char *
MagicMotion_GetCameraURI(unsigned int camera_index)
{
    return magic_motion.sensors[camera_index].URI;
}

const char *
MagicMotion_GetCameraSerialNumber(unsigned int camera_index)
{
    return magic_motion.sensors[camera_index].serial;
}

const Frustum *
MagicMotion_GetCameraFrustums(void)
{
    return magic_motion.sensor_frustums;
}

void
MagicMotion_SetCameraTransform(unsigned int camera_index, Mat4 transform)
{
    magic_motion.sensor_frustums[camera_index].transform = transform;
}

void
MagicMotion_CaptureFrame(void)
{
    magic_motion.cloud_size = 0;
    magic_motion.num_hitbox_events = 0;

    for(unsigned int i=0; i<magic_motion.num_hitboxes; ++i)
    {
        magic_motion.hitboxes[i].point_count = 0;
    }

    for(unsigned int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        ColorPixel *colors = GetSensorColorFrame(sensor);
        DepthPixel *depths = GetSensorDepthFrame(sensor);

        MagicMotionTag tag = (MagicMotionTag)(TAG_CAMERA_0 + i);

        const unsigned int w = sensor->depth_stream_info.width;
        const unsigned int h = sensor->depth_stream_info.height;
        const float fov = sensor->depth_stream_info.fov;
        const float aspect = sensor->depth_stream_info.aspect_ratio;

        const Frustum f = magic_motion.sensor_frustums[i];
        const Mat4 camera_transform = f.transform;

        for(unsigned int y=0; y<h; ++y)
        {
            for(unsigned int x=0; x<w; ++x)
            {
                float depth = depths[x+y*w];
                if(depth > 0.0f)
                {
                    float pos_x = tanf((((float)x/(float)w)-0.5f)*fov) * depth;
                    float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;

                    // Convert from mm to dm as we create the point
                    V3 point = MulMat4Vec3(camera_transform,
                                           (V3){ pos_x / 100.0f, pos_y / 100.0f, depth / 100.0f });

                    if(!(point.x < magic_motion.boundingbox_min.x || point.x > magic_motion.boundingbox_max.x ||
                         point.y < magic_motion.boundingbox_min.y || point.y > magic_motion.boundingbox_max.y ||
                         point.z < magic_motion.boundingbox_min.z || point.z > magic_motion.boundingbox_max.z))
                    {
                        for(unsigned int j=0; j<magic_motion.num_hitboxes; ++j)
                        {
                            magic_motion.hitboxes[j].point_count += (int)MagicMotion_BoxVsV3(magic_motion.hitboxes[j].position,
                                                                                             magic_motion.hitboxes[j].size,
                                                                                             point);
                        }
                    }

                    ColorPixel c = colors[x+y*w];
                    Color color;
                    color.r = c.r;
                    color.g = c.g;
                    color.b = c.b;

                    // Add to point clouds
                    unsigned int index = magic_motion.cloud_size++;
                    magic_motion.spatial_cloud[index] = point;
                    magic_motion.color_cloud[index] = color;
                    magic_motion.tag_cloud[index] = tag;
                }
            }
        }
    }

    for(unsigned int i=0; i<magic_motion.num_hitboxes; ++i)
    {
        if(magic_motion.hitboxes[i].point_count >= HITBOX_POINT_TRESHOLD &&
           !magic_motion.hitboxes[i].considered_hit)
        {
            magic_motion.hitboxes[i].considered_hit = true;
            magic_motion.hitbox_events[magic_motion.num_hitbox_events++] = (MagicMotionHitboxEvent){
                .hitbox = (int)i,
                .enter = true
            };
        }
        else if(magic_motion.hitboxes[i].point_count == 0 &&
                magic_motion.hitboxes[i].considered_hit)
        {
            magic_motion.hitboxes[i].considered_hit = false;
            magic_motion.hitbox_events[magic_motion.num_hitbox_events++] = (MagicMotionHitboxEvent){
                .hitbox = (int)i,
                .enter = false
            };
        }
    }
}

void
MagicMotion_GetColorImageResolution(unsigned int camera_index, int *width, int *height)
{
    SensorInfo *sensor = &magic_motion.sensors[camera_index];
    *width = sensor->color_stream_info.width;
    *height = sensor->color_stream_info.height;
}

void
MagicMotion_GetDepthImageResolution(unsigned int camera_index, int *width, int *height)
{
    SensorInfo *sensor = &magic_motion.sensors[camera_index];
    *width = sensor->depth_stream_info.width;
    *height = sensor->depth_stream_info.height;
}

const Color *
MagicMotion_GetColorImage(unsigned int camera_index)
{
    return NULL;
}

const float *
MagicMotion_GetDepthImage(unsigned int camera_index)
{
    return NULL;
}

unsigned int
MagicMotion_GetCloudSize(void)
{
    return magic_motion.cloud_size;
}

V3 *
MagicMotion_GetPositions(void)
{
    return magic_motion.spatial_cloud;
}

Color *
MagicMotion_GetColors(void)
{
    return magic_motion.color_cloud;
}

MagicMotionTag *
MagicMotion_GetTags(void)
{
    return magic_motion.tag_cloud;
}

static void
_MagicMotion_CalcBoundingBox()
{
    magic_motion.boundingbox_min = (V3){ INFINITY, INFINITY, INFINITY };
    magic_motion.boundingbox_max = (V3){ -INFINITY, -INFINITY, -INFINITY };

    for(int i=0; i<magic_motion.num_hitboxes; ++i)
    {
        V3 min = (V3){
            magic_motion.hitboxes[i].position.x - magic_motion.hitboxes[i].size.x / 2.0f,
            magic_motion.hitboxes[i].position.y - magic_motion.hitboxes[i].size.y / 2.0f,
            magic_motion.hitboxes[i].position.z - magic_motion.hitboxes[i].size.z / 2.0f,
        };

        V3 max = (V3){
            magic_motion.hitboxes[i].position.x + magic_motion.hitboxes[i].size.x / 2.0f,
            magic_motion.hitboxes[i].position.y + magic_motion.hitboxes[i].size.y / 2.0f,
            magic_motion.hitboxes[i].position.z + magic_motion.hitboxes[i].size.z / 2.0f,
        };

        if(min.x < magic_motion.boundingbox_min.x) magic_motion.boundingbox_min.x = min.x;
        if(min.y < magic_motion.boundingbox_min.y) magic_motion.boundingbox_min.y = min.y;
        if(min.z < magic_motion.boundingbox_min.z) magic_motion.boundingbox_min.z = min.z;

        if(max.x > magic_motion.boundingbox_max.x) magic_motion.boundingbox_max.x = max.x;
        if(max.y > magic_motion.boundingbox_max.y) magic_motion.boundingbox_max.y = max.y;
        if(max.z > magic_motion.boundingbox_max.z) magic_motion.boundingbox_max.z = max.z;
    }

    printf("Min: (%f, %f, %f), Max: (%f, %f, %f)\n",
           magic_motion.boundingbox_min.x, magic_motion.boundingbox_min.y, magic_motion.boundingbox_min.z,
           magic_motion.boundingbox_max.x, magic_motion.boundingbox_max.y, magic_motion.boundingbox_max.z);
}

int
MagicMotion_RegisterHitbox(V3 pos, V3 size)
{
    int result = -1;

    if(magic_motion.num_hitboxes < MAX_HITBOXES)
    {
        magic_motion.hitboxes[magic_motion.num_hitboxes] = (Hitbox){ pos, size };
        result = magic_motion.num_hitboxes++;

        _MagicMotion_CalcBoundingBox();

        printf("Registerd hitbox %d at (%f, %f, %f), size: (%f, %f, %f)\n", result, pos.x, pos.y, pos.z, size.x, size.y, size.z);
    }

    return result;
}

void
MagicMotion_UpdateHitbox(int hitbox, V3 pos, V3 size)
{
    magic_motion.hitboxes[hitbox].position = pos;
    magic_motion.hitboxes[hitbox].size = size;
    _MagicMotion_CalcBoundingBox();
}

MagicMotionHitboxEvent *
MagicMotion_QueryHitboxes(int *num_events)
{
    *num_events = magic_motion.num_hitbox_events;
    return magic_motion.hitbox_events;
}

#ifdef __cplusplus
}
#endif

