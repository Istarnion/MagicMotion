#include <string.h>
#include <pthread.h>
#include <assert.h>

// #include "opencv2/video.hpp"
#include <opencv2/video/background_segm.hpp>
#include <opencv2/bgsegm.hpp>

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

#define RUN_TESTS 0
#define MAGIC_MOTION_TRACE 0
#if MAGIC_MOTION_TRACE
#define MM_TRACE(title) printf("MagicMotion Trace @ l%d: %s\n", __LINE__, title)
#else
#define MM_TRACE(title)
#endif

#define BACKGROUND_PROBABILITY_TRESHOLD 0.25

// The 3D classifiers create a voxel grid
// background model each point in the cloud
// can get it's background probability from
// using trilinear interpolation.
enum Classifier3D
{
    CLASSIFIER_3D_NONE,
    CLASSIFIER_3D_CALIBRATION_NAIVE,
    CLASSIFIER_3D_DL // Not implemented yet
};

// The 2D classifiers create a mask per sensor
// containing white pixels for foreground and
// black pixels for background
enum Classifier2D
{
    CLASSIFIER_2D_NONE,
    CLASSIFIER_2D_OPENCV
};

static const Classifier3D classifier3D = CLASSIFIER_3D_NONE;
static const Classifier2D classifier2D = CLASSIFIER_2D_NONE;

struct ClassifierData3D
{
    bool is_calibrating; // For the calibration classifier
    bool running;
    pthread_t thread_handle;
    pthread_mutex_t mutex_handle;
};

struct ClassifierData2D
{
    bool running;
    pthread_t thread_handle;
    pthread_mutex_t mutex_handle;
};

struct SensorFrame
{
    Color *color_frame;
    float *depth_frame;
};

static struct
{
    // Per sensor data:
    SensorInfo  sensors[MAX_SENSORS];
    SensorFrame sensor_frames[MAX_SENSORS];
    float      *sensor_masks[MAX_SENSORS];
    Frustum     sensor_frustums[MAX_SENSORS];
    unsigned int num_active_sensors;

    float *background_model;     // A per-voxel array of the background model

    unsigned int frame_count;    // The frame count increments at every call to CaptureFrame

    V3 *spatial_cloud;           // XYZ components of the point cloud
    Color *color_cloud;          // RGB components of the point cloud
    MagicMotionTag *tag_cloud;   // 32 bit tags for each point in the cloud
    unsigned int cloud_size;     // The number of points currently in the cloud
    unsigned int cloud_capacity; // The maximum number of points in the cloud

    Voxel voxels[NUM_VOXELS];    // The voxel grid, with the lastest information

    // Thread userdata
    ClassifierData3D classifier_thread_3D;
    ClassifierData2D classifier_thread_2D;
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

// Prototype of the functions that will run in a background thread and
// compute the background model.
// The implementation is at the bottom of this file
static void *_ComputeBackgroundModelNaiveCalibration(void *userdata);
static void *_ComputeBackgroundModelDL(void *userdata);
static void *_ComputeBackgroundModelOpenCV(void *userdata);

void
MagicMotion_Initialize(void)
{
    MM_TRACE("Initializing");

    // Simple tests to aid in debugging and development.
    // Replace this with a proper testing framework
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
    MM_TRACE("Initial tests complete");
#endif

    InitializeSensorInterface();

    SerializedSensor serialized_sensors[MAX_SENSORS];
    int num_serialized_sensors = LoadSensors(serialized_sensors, MAX_SENSORS);
    printf("Loaded %d sensor configs:\n", num_serialized_sensors);

    magic_motion.cloud_size = 0;
    magic_motion.cloud_capacity = 0;
    magic_motion.num_active_sensors = PollSensorList(magic_motion.sensors, MAX_SENSORS);
    printf("Found %d compatible sensors\n", magic_motion.num_active_sensors);
    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        int rc = SensorInitialize(sensor, true, true);
        if(rc)
        {
            fprintf(stderr, "Failed to initialize %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
            return;
        }
        else
        {
            printf("Initialized %s %s (URI: %s).\n", sensor->vendor, sensor->name, sensor->URI);
        }

        magic_motion.sensor_frustums[i] = (Frustum){
            .transform = IdentityMat4(),
            .fov = sensor->depth_stream_info.fov,
            .aspect = sensor->depth_stream_info.aspect_ratio,
            .near_plane = MAX(0.05f, sensor->depth_stream_info.min_depth / 100.0f),
            .far_plane = sensor->depth_stream_info.max_depth / 100.0f
        };

        magic_motion.sensor_frames[i].color_frame = (Color *)calloc(sensor->color_stream_info.width * sensor->color_stream_info.height, sizeof(Color));
        magic_motion.sensor_masks[i] = (float *)malloc(sensor->depth_stream_info.width *
                                                      sensor->depth_stream_info.height *
                                                      sizeof(float));
        std::fill_n(magic_motion.sensor_masks[i],
                    sensor->depth_stream_info.width*sensor->depth_stream_info.height,
                    1.0f);

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
                printf("Failed to load sensor config: %s did not match %s\n",
                       sensor->serial, serialized_sensors[j].serial);
            }
        }
    }

    MM_TRACE("Sensors initialized");

    magic_motion.spatial_cloud = (V3 *)calloc(magic_motion.cloud_capacity,
                                              sizeof(V3));
    assert(magic_motion.spatial_cloud);

    magic_motion.tag_cloud = (MagicMotionTag *)calloc(magic_motion.cloud_capacity,
                                                      sizeof(MagicMotionTag));
    assert(magic_motion.tag_cloud);

    magic_motion.color_cloud = (Color *)calloc(magic_motion.cloud_capacity,
                                               sizeof(Color));
    assert(magic_motion.color_cloud);

    magic_motion.background_model = (float *)calloc(NUM_VOXELS,
                                                    sizeof(float));
    assert(magic_motion.background_model);

    MM_TRACE("Global buffers allocated");

    pthread_attr_t thread_attributes;
    pthread_attr_init(&thread_attributes); // Set default attributes
    // pthread_attr_setdetachstate(&thread_attributes, PTHREAD_CREATE_DETACHED);

    void *(*thread_3D)(void *);
    switch(classifier3D)
    {
        case CLASSIFIER_3D_CALIBRATION_NAIVE:
            thread_3D = &_ComputeBackgroundModelNaiveCalibration;
            break;
        case CLASSIFIER_3D_DL:
            thread_3D = &_ComputeBackgroundModelDL;
            break;
        default:
            thread_3D = NULL;
            break;
    }

    void *(*thread_2D)(void *);
    switch(classifier2D)
    {
        case CLASSIFIER_2D_OPENCV:
            thread_2D = &_ComputeBackgroundModelOpenCV;
            break;
        default:
            thread_2D = NULL;
            break;
    }

    if(thread_3D)
    {
        magic_motion.classifier_thread_3D.running = true;
        pthread_create(&magic_motion.classifier_thread_3D.thread_handle, &thread_attributes,
                       thread_3D, &magic_motion.classifier_thread_3D);
    }

    if(thread_2D)
    {
        magic_motion.classifier_thread_2D.running = true;
        pthread_create(&magic_motion.classifier_thread_2D.thread_handle, &thread_attributes,
                       thread_2D, &magic_motion.classifier_thread_2D);
    }

    pthread_attr_destroy(&thread_attributes);

    MM_TRACE("Background thread(s) started");

    printf("MagicMotion initialized with %u active sensors. Point cloud size: %u. %u voxels.\n",
           magic_motion.num_active_sensors, magic_motion.cloud_capacity, NUM_VOXELS);
}

void
MagicMotion_Finalize(void)
{
    MM_TRACE("Finalizing");
    if(magic_motion.classifier_thread_3D.running)
    {
        magic_motion.classifier_thread_3D.running = false;
        pthread_join(magic_motion.classifier_thread_3D.thread_handle, NULL);
        MM_TRACE("Ended 3D classifier thread");
    }

    if(magic_motion.classifier_thread_2D.running)
    {
        magic_motion.classifier_thread_2D.running = false;
        pthread_join(magic_motion.classifier_thread_2D.thread_handle, NULL);
        MM_TRACE("Ended 2D classifier thread");
    }


    free(magic_motion.background_model);
    free(magic_motion.color_cloud);
    free(magic_motion.tag_cloud);
    free(magic_motion.spatial_cloud);
    MM_TRACE("Freed global buffers");

    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SaveSensor(magic_motion.sensors[i].serial, &magic_motion.sensor_frustums[i]);
        SensorFinalize(&magic_motion.sensors[i]);
        free(magic_motion.sensor_frames[i].color_frame);
        free(magic_motion.sensor_masks[i]);
    }
    MM_TRACE("Closed all sensors");

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
    MM_TRACE("Starting frame capture");

    // The GetSensor*Frame functions will block for a while due to the
    // camera hardware, so we wait until those are done before we take the
    // 3D mutex. The 2D classifiers uses the buffers we fill here however,
    // so we need to take the mutex up here, and do the 2D classification
    // during rendering / other work

    pthread_mutex_lock(&magic_motion.classifier_thread_2D.mutex_handle);
    MM_TRACE("Got the 2D mutex");

    for(size_t i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        ColorPixel *color_frame = GetSensorColorFrame(sensor);
        size_t num_color_pixels = sensor->color_stream_info.width *
                                  sensor->color_stream_info.height;
        MM_TRACE("Got color frame");

        for(size_t j=0; j<num_color_pixels; ++j)
        {
            Color c = (Color) { .r = color_frame[j].r,
                                .g = color_frame[j].g,
                                .b = color_frame[j].g,
                                ._padding = 0
                              };

            magic_motion.sensor_frames[i].color_frame[j] = c;
        }

        magic_motion.sensor_frames[i].depth_frame = GetSensorDepthFrame(sensor);
        MM_TRACE("Got depth frame");
    }

    pthread_mutex_lock(&magic_motion.classifier_thread_3D.mutex_handle);
    MM_TRACE("Got 3D mutex");

    magic_motion.cloud_size = 0;
    memset(magic_motion.voxels, 0, NUM_VOXELS*sizeof(Voxel));
    ++magic_motion.frame_count;

    for(unsigned int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        MM_TRACE("Starting computation on sensor");
        SensorInfo *sensor = &magic_motion.sensors[i];
        Color      *colors = magic_motion.sensor_frames[i].color_frame;
        DepthPixel *depths = magic_motion.sensor_frames[i].depth_frame;

        // NOTE(istarnion): The color and depth streams does often
        // NOT have the same resolution, especially with image
        // registration enabled, but depth resolution is always
        // smaller than color resolution.
        assert(sensor->depth_stream_info.width <= sensor->color_stream_info.width);
        assert(sensor->depth_stream_info.height <= sensor->color_stream_info.height);

        const unsigned int w = sensor->depth_stream_info.width;
        const unsigned int h = sensor->depth_stream_info.height;
        const unsigned int color_w = sensor->color_stream_info.width;
        const unsigned int color_h = sensor->color_stream_info.height;

        const float fov = sensor->depth_stream_info.fov;
        const float aspect = sensor->depth_stream_info.aspect_ratio;

        const Frustum f = magic_motion.sensor_frustums[i];
        const Mat4 camera_transform = f.transform;

        for(uint32_t y=0; y<h; ++y)
        {
            for(uint32_t x=0; x<w; ++x)
            {
                float depth = depths[x+y*w];
                float mask = magic_motion.sensor_masks[i][x+y*w];
                if(depth > 0.0f)
                {
                    float pos_x = tanf((((float)x/(float)w)-0.5f)*fov) * depth;
                    float pos_y = tanf((0.5f-((float)y / (float)h))*(fov/aspect)) * depth;

                    // Convert from mm to dm as we create the point
                    V3 point = MulMat4Vec3(camera_transform,
                                           (V3){ pos_x / 100.0f,
                                                 pos_y / 100.0f,
                                                 depth / 100.0f });

                    Color color = colors[(color_w/2-w/2+x)+(color_h/2-h/2+y)*color_w];

                    int tag = (TAG_CAMERA_0 + i);

                    // Check if the point is within the voxel grid
                    if(fabs(point.x) < BOUNDING_BOX_X/2.0f &&
                       fabs(point.y) < BOUNDING_BOX_Y/2.0f &&
                       fabs(point.z) < BOUNDING_BOX_Z/2.0f)
                    {
                        // Determine if the point is background or foreground
                        float background_probability = _TrilinearlyInterpolate(point, magic_motion.background_model);
                        // Skip trilinear interpolation, and use nearest voxel instead:
                        // float background_probability = magic_motion.background_data.background[voxel_index];

                        const float mix = 0.1f; // 0 is only background model, 1 is only sensor mask
                        background_probability = LERP(background_probability, (1.0f - mask), mix);

                        if(background_probability < BACKGROUND_PROBABILITY_TRESHOLD ||
                           magic_motion.classifier_thread_3D.is_calibrating)
                        {
                            tag |= TAG_FOREGROUND;

                            uint32_t voxel_index = WORLD_TO_VOXEL(point);

                            Voxel *v = &magic_motion.voxels[voxel_index];
                            assert(v); // The voxel should never be NULL.

                            // Add the current points color into the running average
                            v->color.r = (uint8_t)((color.r + v->point_count * v->color.r) /
                                                   (v->point_count+1));
                            v->color.g = (uint8_t)((color.g + v->point_count * v->color.g) /
                                                   (v->point_count+1));
                            v->color.b = (uint8_t)((color.b + v->point_count * v->color.b) /
                                                   (v->point_count+1));

                            ++v->point_count;
                        }
                        else
                        {
                            tag |= TAG_BACKGROUND;
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

    // The naive classifier needs some help with noise
    if(classifier3D == CLASSIFIER_3D_CALIBRATION_NAIVE)
    {
        for(int i=0; i<magic_motion.cloud_size; ++i)
        {
            uint32_t tag = magic_motion.tag_cloud[i];
            if(tag & TAG_FOREGROUND)
            {
                // If it has been tagged as foreground, it will be within the
                // voxel bounds, so the voxel_index will always be within bounds
                uint32_t voxel_index = WORLD_TO_VOXEL(magic_motion.spatial_cloud[i]);
                assert(voxel_index >= 0 && voxel_index < NUM_VOXELS);
                if(magic_motion.voxels[voxel_index].point_count < 8)
                {
                    tag |= TAG_BACKGROUND;
                    tag &= ~TAG_FOREGROUND;
                    magic_motion.tag_cloud[i] = (MagicMotionTag)tag;
                }
            }
        }
    }

    pthread_mutex_unlock(&magic_motion.classifier_thread_3D.mutex_handle);
    pthread_mutex_unlock(&magic_motion.classifier_thread_2D.mutex_handle);
    MM_TRACE("Finished frame capture");
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
    return magic_motion.sensor_frames[camera_index].color_frame;
}

const float *
MagicMotion_GetDepthImage(unsigned int camera_index)
{
    return magic_motion.sensor_frames[camera_index].depth_frame;
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
_ComputeBackgroundModelNaiveCalibration(void *userdata)
{
    ClassifierData3D *data = (ClassifierData3D *)userdata;

    // A place to store a copy of the latest voxel frame
    Voxel *latest_frame = (Voxel *)malloc(NUM_VOXELS * sizeof(Voxel));

    // Buffer to store average point counts per voxel during calibration
    float *avg_point_counts = (float *)malloc(NUM_VOXELS * sizeof(float));
    bool was_calibrating_last_frame = false;
    unsigned int calibration_start_frame = 0;

    while(data->running)
    {
        // Get last frame voxel grid.
        unsigned int frame_count = magic_motion.frame_count;
        // Consider taking the mutex for this. The main thread might be writing
        // to the voxel grid while memcpy runs.
        memcpy(latest_frame, magic_motion.voxels, NUM_VOXELS * sizeof(Voxel));

        if(data->is_calibrating)
        {
            if(!was_calibrating_last_frame)
            {
                // This is the first frame of the calibration
                calibration_start_frame = frame_count;
                memset(avg_point_counts, 0, NUM_VOXELS * sizeof(float));
                memset(magic_motion.background_model, 0, NUM_VOXELS * sizeof(float));
                was_calibrating_last_frame = true;
            }

            unsigned int framenum = frame_count - calibration_start_frame;

            for(uint32_t i=0; i<NUM_VOXELS; ++i)
            {
                float point_count = (float)latest_frame[i].point_count;
                /* Average: */
                // avg_point_counts[i] = (avg_point_counts[i] * framenum +
                //                        point_count) / (framenum+1);
                /* Max: */
                avg_point_counts[i] = MAX(avg_point_counts[i], point_count);
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
                    magic_motion.background_model[i] = background_prob;
                }

                pthread_mutex_unlock(&data->mutex_handle);
            }

            was_calibrating_last_frame = false;
        }

        sched_yield();
    }

    free(avg_point_counts);
    free(latest_frame);

    return NULL;
}

static void *
_ComputeBackgroundModelDL(void *userdata)
{
    ClassifierData3D *data = (ClassifierData3D *)userdata;

    // A place to store a copy of the latest voxel frame
    Voxel *latest_frame = (Voxel *)malloc(NUM_VOXELS * sizeof(Voxel));

    while(data->running)
    {
        // Get last frame voxel grid.
        // For the DL classifier we might want to feed it 4D data (+time), in
        // which case we need to get multiple frames
        pthread_mutex_lock(&data->mutex_handle);
        unsigned int frame_count = magic_motion.frame_count;
        memcpy(latest_frame, magic_motion.voxels, NUM_VOXELS * sizeof(Voxel));
        pthread_mutex_unlock(&data->mutex_handle);

        // Process
        sleep(1);

        pthread_mutex_lock(&data->mutex_handle);

        for(uint32_t i=0; i<NUM_VOXELS; ++i)
        {
            // TEMP: Set probability for background to
            // 100% for all voxels
            magic_motion.background_model[i] = 1.0f;
        }

        pthread_mutex_unlock(&data->mutex_handle);

        sched_yield();
    }

    free(latest_frame);

    return NULL;
}

static void *
_ComputeBackgroundModelOpenCV(void *userdata)
{
    ClassifierData2D *data = (ClassifierData2D *)userdata;

    cv::Mat frames[MAX_SENSORS];
    cv::Mat masks[MAX_SENSORS];

    cv::Ptr<cv::bgsegm::BackgroundSubtractorGSOC> subtractor =
        cv::bgsegm::createBackgroundSubtractorGSOC();

    float *input_imgs[MAX_SENSORS];
    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        SensorInfo *sensor = &magic_motion.sensors[i];
        input_imgs[i] = (float *)malloc(sensor->depth_stream_info.width*
                                        sensor->depth_stream_info.height*
                                        sizeof(float));
    }

    while(data->running)
    {
        for(int i=0; i<magic_motion.num_active_sensors; ++i)
        {
            SensorInfo *sensor = &magic_motion.sensors[i];

            // Shortcuts/cache of commonly used widths/heights
            const int cw = sensor->color_stream_info.width;
            const int ch = sensor->color_stream_info.height;
            const int dw = sensor->depth_stream_info.width;
            const int dh = sensor->depth_stream_info.height;

            float *depth_pixels = magic_motion.sensor_frames[i].depth_frame;
            Color *color_pixels = magic_motion.sensor_frames[i].color_frame;
            if(!(depth_pixels && color_pixels)) continue;

            // We feed the subtractor a mix of the depth image signal and
            // the grayscaled color image signal to try to get the
            // best of both worlds
            const float mix = 0.95f; // 0 is only depth, 1 is only color
            const size_t num_pixels = dw * dh;
            memcpy(input_imgs[i], depth_pixels, num_pixels*sizeof(float));
            for(size_t j=0; j<num_pixels; ++j)
            {
                int cx = (cw-dw)/2 + j%dw;
                int cy = (ch-dh)/2 + j/dw;
                Color c = color_pixels[cx+cy*cw];

                // Grayscale
                float value = (c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f) / 255.0f;

                // Mix the signals
                input_imgs[i][j] = LERP(input_imgs[i][j], value, mix);
            }

            frames[i] = cv::Mat(sensor->depth_stream_info.height, // Rows
                                sensor->depth_stream_info.width,  // Cols
                                CV_32FC1,                         // Type
                                input_imgs[i]);                   // Data

            subtractor->apply(frames[i], masks[i]);

            pthread_mutex_lock(&data->mutex_handle);
            float *mask = magic_motion.sensor_masks[i];
            masks[i].forEach<uint8_t>(
                [mask, cw, ch, dw, dh](uint8_t &m, const int position[]) -> void
                {
                    int x = position[1], y = position[0];
                    mask[x + y * dw] = m / 255.0f;
                });

            pthread_mutex_unlock(&data->mutex_handle);
        }

        sched_yield();
    }

    for(int i=0; i<magic_motion.num_active_sensors; ++i)
    {
        free(input_imgs[i]);
    }

    return NULL;
}

void
MagicMotion_StartCalibration(void)
{
    magic_motion.classifier_thread_3D.is_calibrating = true;
}

void
MagicMotion_EndCalibration(void)
{
    magic_motion.classifier_thread_3D.is_calibrating = false;
}

bool
MagicMotion_IsCalibrating(void)
{
    return magic_motion.classifier_thread_3D.is_calibrating;
}

#ifdef __cplusplus
}
#endif

