#include <string.h>
#include <pthread.h>

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

#define RUN_TESTS 1
#define BACKGROUND_PROBABILITY_TRESHOLD 0.01

enum Classifier
{
    CLASSIFIER_CALIBRATION_NAIVE,
    CLASSIFIER_DL
};

Classifier background_classifier = CLASSIFIER_CALIBRATION_NAIVE;

// A model of the background. For each voxel, this array contains the probability that voxel is part of the background
// This will be updated from a CNN in a background thread
// For that reason, we need a mutex as well.
// An instance of this struct is passed as
// userdata for the the background thread
struct BackgroundData
{
    float *background;
    bool running;
    bool is_calibrating; // For the calibration classifier
    pthread_t thread_handle;
    pthread_mutex_t mutex_handle;
    pthread_barrier_t barrier;
};

static struct
{
    SensorInfo sensors[MAX_SENSORS];
    Frustum sensor_frustums[MAX_SENSORS];
    unsigned int num_active_sensors;

    unsigned int frame_count;

    V3 *spatial_cloud;
    Color *color_cloud;
    MagicMotionTag *tag_cloud;
    unsigned int cloud_size;
    unsigned int cloud_capacity;

    Voxel *voxels;
    BackgroundData background_data;
} magic_motion;

typedef struct
{
    int indices[8];
} Neighbours;

// Get the indices of the 8 nearest neighbours of point
static inline Neighbours
_GetNeighbours(V3 point)
{
    // Get voxel index
    int i = WORLD_TO_VOXEL(point);
    // Get center position of that voxel
    V3 v = VOXEL_TO_WORLD(i);
    // Calc the offset from the point to the
    // center of the containing voxel
    V3 offset = SubV3(point, v);

    // Get voxel space coordinates for the voxel
    // (inverse of VOXEL_INDEX)
    int x0 = i % NUM_VOXELS_X;
    int y0 = (i / NUM_VOXELS_X) % NUM_VOXELS_Y;
    int z0 = i / (NUM_VOXELS_X * NUM_VOXELS_Y);

    if(offset.x < 0) --x0;
    if(offset.y < 0) --y0;
    if(offset.z < 0) --z0;

    x0 = MAX(x0, 0);
    y0 = MAX(y0, 0);
    z0 = MAX(z0, 0);

    int x1 = MIN(x0+1, NUM_VOXELS_X-1);
    int y1 = MIN(y0+1, NUM_VOXELS_Y-1);
    int z1 = MIN(z0+1, NUM_VOXELS_Z-1);

    return (Neighbours){{
        VOXEL_INDEX(x0, y0, z0),
        VOXEL_INDEX(x0, y0, z1),
        VOXEL_INDEX(x0, y1, z0),
        VOXEL_INDEX(x0, y1, z1),
        VOXEL_INDEX(x1, y0, z0),
        VOXEL_INDEX(x1, y0, z1),
        VOXEL_INDEX(x1, y1, z0),
        VOXEL_INDEX(x1, y1, z1)
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
        V3 voxel = VOXEL_TO_WORLD(index);
        float weight = (1.0f - (fabs(point.x - voxel.x) / VOXEL_SIZE)) *
                       (1.0f - (fabs(point.y - voxel.y) / VOXEL_SIZE)) *
                       (1.0f - (fabs(point.z - voxel.z) / VOXEL_SIZE));

        probability += weight * background[index];
    }

    return probability;
}

// Prototype of the function that will run in a background thread and
// compute the background model.
// The implementation is at the bottom of this file
static void *_ComputeBackgroundModel(void *userdata);

void
MagicMotion_Initialize(void)
{
#if RUN_TESTS
    puts("Running tests");

    {
        V3 v0 = VOXEL_TO_WORLD(0);
        printf("First voxel has position (%f, %f, %f)\n", v0.x, v0.y, v0.z);
        V3 v1 = VOXEL_TO_WORLD(VOXEL_INDEX(NUM_VOXELS_X/2, NUM_VOXELS_Y/2, NUM_VOXELS_Z/2));
        printf("Middle voxel has position (%f, %f, %f)\n", v1.x, v1.y, v1.z);
        V3 v2 = VOXEL_TO_WORLD(NUM_VOXELS-1);
        printf("Last voxel has position (%f, %f, %f)\n", v2.x, v2.y, v2.z);
    }

    {
        Neighbours n = _GetNeighbours((V3){ 0, 0, 0 });
        printf("Neighbours of point (0, 0, 0) are %d, %d, %d, %d, %d, %d, %d, %d\n",
               n.indices[0], n.indices[1], n.indices[2], n.indices[3],
               n.indices[4], n.indices[5], n.indices[6], n.indices[7]);
    }

    {
        float *bg = (float *)malloc(NUM_VOXELS * sizeof(float));
        for(size_t i=0; i<NUM_VOXELS; ++i) bg[i] = 1.0f;
        V3 p = { 0, 0, 0 };
        puts("Interpolating point");

        float probability = 0;
        Neighbours n = _GetNeighbours(p);
        printf("Neighbours are %d, %d, %d, %d, %d, %d, %d, %d\n",
               n.indices[0], n.indices[1], n.indices[2], n.indices[3],
               n.indices[4], n.indices[5], n.indices[6], n.indices[7]);

        for(int i=0; i<8; ++i)
        {
            const int index = n.indices[i];
            V3 voxel = VOXEL_TO_WORLD(index);
            float a = (1.0f - (fabs(p.x - voxel.x) / VOXEL_SIZE));
            float b = (1.0f - (fabs(p.y - voxel.y) / VOXEL_SIZE));
            float c = (1.0f - (fabs(p.z - voxel.z) / VOXEL_SIZE));
            float weight = a * b * c;

            printf("Weight for neighbour %d (%f, %f, %f) = a=%f * b=%f * c=%f = %f\n",
                   i, voxel.x, voxel.y, voxel.z, a, b, c, weight);

            probability += weight * bg[index];
        }

        printf("Calculated probability: %f\n", probability);

        free(bg);
    }

    {
        V3 p = { 0, 0, 0 };
        V3 v = VOXEL_TO_WORLD(WORLD_TO_VOXEL(p));
        float a = (1.0f - (fabs(p.x - v.x) / VOXEL_SIZE));
        float b = (1.0f - (fabs(p.y - v.y) / VOXEL_SIZE));
        float c = (1.0f - (fabs(p.z - v.z) / VOXEL_SIZE));

        printf("p: (%f, %f, %f), v: (%f, %f, %f)\na=%f * b=%f * c=%f = %f\n",
                p.x, p.y, p.z,
                v.x, v.y, v.z,
                a, b, c, a*b*c);

    }

    puts("End of testing.");
#endif

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
    magic_motion.background_data.background = (float *)calloc(NUM_VOXELS, sizeof(float));

    pthread_barrier_init(&magic_motion.background_data.barrier, NULL, 2);
    magic_motion.background_data.running = true;
    pthread_attr_t thread_attributes;
    pthread_attr_init(&thread_attributes); // Set default attributes
    pthread_attr_setdetachstate(&thread_attributes, PTHREAD_CREATE_DETACHED);
    pthread_create(&magic_motion.background_data.thread_handle, &thread_attributes,
                   &_ComputeBackgroundModel, &magic_motion.background_data);
    pthread_attr_destroy(&thread_attributes);

    printf("MagicMotion initialized with %u active sensors. Point cloud size: %u. %u voxels.\n",
           magic_motion.num_active_sensors, magic_motion.cloud_capacity, NUM_VOXELS);

}

void
MagicMotion_Finalize(void)
{
    magic_motion.background_data.running = false;
    // pthread_join(magic_motion.background_data.thread_handle, NULL);

    free(magic_motion.background_data.background);
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
    // The GetSensor*Frame functions will block for a while due to the
    // camera hardware, so we wait until those are done before we take the mutex

    SensorInfo *sensor_infos[magic_motion.num_active_sensors];
    ColorPixel *sensor_color_pixels[magic_motion.num_active_sensors];
    DepthPixel *sensor_depth_pixels[magic_motion.num_active_sensors];
    for(unsigned int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        sensor_infos[i] = sensor;
        sensor_color_pixels[i] = GetSensorColorFrame(sensor);
        sensor_depth_pixels[i] = GetSensorDepthFrame(sensor);
    }

    pthread_mutex_lock(&magic_motion.background_data.mutex_handle);

    magic_motion.cloud_size = 0;
    memset(magic_motion.voxels, 0, NUM_VOXELS*sizeof(Voxel));
    ++magic_motion.frame_count;

    for(unsigned int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = sensor_infos[i];
        ColorPixel *colors = sensor_color_pixels[i];
        DepthPixel *depths = sensor_depth_pixels[i];

        int tag = (TAG_CAMERA_0 + i);

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
                                           (V3){ pos_x / 100.0f,
                                                 pos_y / 100.0f,
                                                 depth / 100.0f });

                    ColorPixel c = colors[x+y*w];
                    Color color;
                    color.r = c.r;
                    color.g = c.g;
                    color.b = c.b;

                    // Check if the point is within the voxel grid
                    if(fabs(point.x) < BOUNDING_BOX_X/2.0f &&
                       fabs(point.y) < BOUNDING_BOX_Y/2.0f &&
                       fabs(point.z) < BOUNDING_BOX_Z/2.0f)
                    {
                        /*
                        uint32_t voxel_x = (uint32_t)(point.x / VOXEL_SIZE - 0.5f) +
                                           NUM_VOXELS_X/2;
                        uint32_t voxel_y = (uint32_t)(point.y / VOXEL_SIZE - 0.5f) +
                                           NUM_VOXELS_Y/2;
                        uint32_t voxel_z = (uint32_t)(point.z / VOXEL_SIZE - 0.5f) +
                                           NUM_VOXELS_Z/2;

                        uint32_t voxel_index = VOXEL_INDEX(voxel_x, voxel_y, voxel_z);
                        */

                        uint32_t voxel_index = WORLD_TO_VOXEL(point);

                        Voxel *v = &magic_motion.voxels[voxel_index];

                        // Add the current points color into the running average
                        v->color.r = (uint8_t)((color.r + v->point_count * v->color.r) /
                                               (v->point_count+1));
                        v->color.g = (uint8_t)((color.g + v->point_count * v->color.g) /
                                               (v->point_count+1));
                        v->color.b = (uint8_t)((color.b + v->point_count * v->color.b) /
                                               (v->point_count+1));

                        ++v->point_count;

                        // Determine if the point is background or foreground
                        float background_probability = _TrilinearlyInterpolate(point, magic_motion.background_data.background);

                        if(background_probability < BACKGROUND_PROBABILITY_TRESHOLD)
                        {
                            tag |= TAG_FOREGROUND;
                            tag &= ~TAG_BACKGROUND;
                        }
                        else
                        {
                            tag |= TAG_BACKGROUND;
                            tag &= ~TAG_FOREGROUND;
                        }
                    }

                    // Add to point clouds
                    unsigned int index = magic_motion.cloud_size++;
                    magic_motion.spatial_cloud[index] = point;
                    magic_motion.color_cloud[index] = color;
                    magic_motion.tag_cloud[index] = (MagicMotionTag)tag;
                }
            }
        }
    }

    pthread_mutex_unlock(&magic_motion.background_data.mutex_handle);
    pthread_barrier_wait(&magic_motion.background_data.barrier);
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

static void *
_ComputeBackgroundModel(void *userdata)
{
    BackgroundData *data = (BackgroundData *)userdata;

    // A place to store a copy of the latest voxel frame
    Voxel *latest_frame = (Voxel *)malloc(NUM_VOXELS * sizeof(Voxel));

    // Buffer to store average point counts per voxel during calibration
    float *avg_point_counts = (float *)malloc(NUM_VOXELS * sizeof(float));
    bool was_calibrating_last_frame = false;
    unsigned int calibration_start_frame = 0;

    while(data->running)
    {
        // This barrier is only a good idea as long as the processing in this thread
        // takes less than 30 ms
        pthread_barrier_wait(&magic_motion.background_data.barrier);

        // Get last frame voxel grid.
        // For the DL classifier we might want to feed it 4D data (+time), in
        // which case we need to get multiple frames
        pthread_mutex_lock(&data->mutex_handle);
        unsigned int frame_count = magic_motion.frame_count;
        memcpy(latest_frame, magic_motion.voxels, NUM_VOXELS * sizeof(Voxel));
        pthread_mutex_unlock(&data->mutex_handle);
        // TODO(istarnion): A barrier would be much more useful here than
        // mutexes. It would allow us to be more in sync with the camera loop
        // as well.
        // But we might not _want_ to be in sync.. We don't want the main thread
        // to wait for this thread much

        switch(background_classifier)
        {
            case CLASSIFIER_CALIBRATION_NAIVE:
            {
                if(data->is_calibrating)
                {
                    if(!was_calibrating_last_frame)
                    {
                        // This is the first frame of the calibration
                        calibration_start_frame = frame_count;
                        memset(avg_point_counts, 0, NUM_VOXELS * sizeof(float));
                        was_calibrating_last_frame = true;
                    }

                    unsigned int framenum = frame_count - calibration_start_frame;

                    for(uint32_t i=0; i<NUM_VOXELS; ++i)
                    {
                        float point_count = (float)latest_frame[i].point_count;
                        avg_point_counts[i] = (avg_point_counts[i] * framenum +
                                               point_count) / (framenum+1);
                    }
                }
                else
                {
                    if(was_calibrating_last_frame)
                    {
                        // This is the first frame after we stop calibrating
                        pthread_mutex_lock(&data->mutex_handle);

                        for(uint32_t i=0; i<NUM_VOXELS; ++i)
                        {
                            float background_prob = MIN(1.0f, avg_point_counts[i]);
                            data->background[i] = background_prob;
                        }

                        pthread_mutex_unlock(&data->mutex_handle);
                    }

                    was_calibrating_last_frame = false;
                }

                break;
            }
            case CLASSIFIER_DL:
            {
                // Process..

                pthread_mutex_lock(&data->mutex_handle);

                for(uint32_t i=0; i<NUM_VOXELS; ++i)
                {
                    // TEMP: Set probability for background to
                    // 100% for all voxels
                    data->background[i] = 1.0f;
                }

                pthread_mutex_unlock(&data->mutex_handle);
                break;
            }
        }
    }

    free(avg_point_counts);
    free(latest_frame);

    return NULL;
}

void
MagicMotion_StartCalibration(void)
{
    background_classifier = CLASSIFIER_CALIBRATION_NAIVE;
    magic_motion.background_data.is_calibrating = true;
}

void
MagicMotion_EndCalibration(void)
{
    magic_motion.background_data.is_calibrating = false;
}

bool
MagicMotion_IsCalibrating(void)
{
    return background_classifier == CLASSIFIER_CALIBRATION_NAIVE &&
           magic_motion.background_data.is_calibrating;
}

#ifdef __cplusplus
}
#endif

