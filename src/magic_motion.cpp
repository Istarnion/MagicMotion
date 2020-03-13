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

    Voxel *voxels;
     // A model of the background. For each voxel, this array contains the probability that voxel is part of the background
     // This will be updated from a CNN in a background thread
    float *background;
} magic_motion;

typedef struct
{
    int indices[8];
} Neighbours;

// Get the indices of the 8 nearest neighbours of point
static Neighbours
_GetNeighbours(V3 point)
{
    int x0 = (int)floor(point.x / VOXEL_SIZE - 0.5f);
    int y0 = (int)floor(point.x / VOXEL_SIZE - 0.5f);
    int z0 = (int)floor(point.x / VOXEL_SIZE - 0.5f);

    return (Neighbours){{
        VOXEL_INDEX(x0,   y0,   z0),
        VOXEL_INDEX(x0,   y0,   z0+1),
        VOXEL_INDEX(x0,   y0+1, z0),
        VOXEL_INDEX(x0,   y0+1, z0+1),
        VOXEL_INDEX(x0+1, y0,   z0),
        VOXEL_INDEX(x0+1, y0,   z0+1),
        VOXEL_INDEX(x0+1, y0+1, z0),
        VOXEL_INDEX(x0+1, y0+1, z0+1)
    }};
}

// Get the probability of point being part of the background by interpolating
// the probabilities of the 8 nearest neighbours
float
_TrilinearlyInterpolate(V3 point, float *background)
{
    float probability = 0;
    Neighbours n = _GetNeighbours(point);

    for(int i=0; i<8; ++i)
    {
        const int index = n.indices[i];
        V3 voxel = VOXEL_POSITION(index);
        weight = (1.0f - fabs(point.x - voxel.x) / VOXEL_SIZE) *
                 (1.0f - fabs(point.y - voxel.y) / VOXEL_SIZE) *
                 (1.0f - fabs(point.z - voxel.z) / VOXEL_SIZE);

        probability += weight * background[index];
    }

    return probability;
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
                magic_motion.sensor_frustums[i] = f;
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

    magic_motion.voxels = (Voxel *)calloc(NUM_VOXELS, sizeof(Voxel));
    magic_motion.background = (float *)calloc(NUM_VOXELS, sizeof(float));

    printf("MagicMotion initialized with %u active sensors. Point cloud size: %u. %u voxels.\n",
           magic_motion.num_active_sensors, magic_motion.cloud_capacity, NUM_VOXELS);
}

void
MagicMotion_Finalize(void)
{
    free(magic_motion.background);
    free(magic_motion.voxels);
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

Mat4
MagicMotion_GetCameraTransform(unsigned int camera_index)
{
    return magic_motion.sensor_frustums[camera_index].transform;
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
    memset(magic_motion.voxels, 0, NUM_VOXELS*sizeof(Voxel));

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

        for(uint32_t y=0; y<h; ++y)
        {
            for(uint32_t x=0; x<w; ++x)
            {
                float depth = depths[x+y*w];
                if(depth > 0.0f)
                {
                    float pos_x = tanf((((float)x/(float)w)-0.5f)*fov) * depth;
                    float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;

                    // Convert from mm to dm as we create the point
                    V3 point = MulMat4Vec3(camera_transform,
                                           (V3){ pos_x / 100.0f, pos_y / 100.0f, depth / 100.0f });

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

                    // Check if the point is within the voxel grid
                    if(fabs(point.x) < BOUNDING_BOX_X/2.0f &&
                       fabs(point.y) < BOUNDING_BOX_Y/2.0f &&
                       fabs(point.z) < BOUNDING_BOX_Z/2.0f)
                    {
                        uint32_t voxel_x = (uint32_t)(point.x / VOXEL_SIZE) + NUM_VOXELS_X/2;
                        uint32_t voxel_y = (uint32_t)(point.y / VOXEL_SIZE) + NUM_VOXELS_Y/2;
                        uint32_t voxel_z = (uint32_t)(point.z / VOXEL_SIZE) + NUM_VOXELS_Z/2;

                        uint32_t voxel_index = voxel_x +
                                               voxel_y*NUM_VOXELS_X +
                                               voxel_z*NUM_VOXELS_X*NUM_VOXELS_Z;

                        Voxel *v = &magic_motion.voxels[voxel_index];
                        v->color.r = (uint8_t)((color.r + v->point_count * v->color.r) / (v->point_count+1));
                        v->color.g = (uint8_t)((color.g + v->point_count * v->color.g) / (v->point_count+1));
                        v->color.b = (uint8_t)((color.b + v->point_count * v->color.b) / (v->point_count+1));
                        ++v->point_count;
                    }
                }
            }
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

Voxel *
MagicMotion_GetVoxels(void)
{
    return magic_motion.voxels;
}

#ifdef __cplusplus
}
#endif

