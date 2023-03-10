#ifndef MO_STEREO_CAMERA_DRIVER_TYPE_DEFINE_INCLUDED
#define MO_STEREO_CAMERA_DRIVER_TYPE_DEFINE_INCLUDED

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/**< enum define area */
typedef enum _mo_camera_type_e {
    MCT_UVC = 0,      // UVC  camera
    MCT_MIPI = 1,     // MIPI camera
    MCT_ETHERNET = 2  // Ethernet camera
} mo_camera_type;

typedef enum _mo_filllight_type_e {
    MFT_OFF = 0,                // Fill light off
    MFT_ON = 1,                 // Fill light on
    MFT_EXPOSURE_SYNC = 2,      // Fill light synchronizes with exposure
    MFT_ON_OFF_ALTERNATION = 3  // Fill light alternates between on and off
} mo_filllight_type;

typedef enum _mo_video_mode_e {
    MVM_RAW = 0,           // Bayer           + Bayer
    MVM_RECTIFIED = 2,     // Gray(Only Y)    + YUV_I420
    MVM_RGBD = 5,          // Disparity       + YUV_I420
    MVM_RGBD_DENSE = 10,   // Dense disparity + YUV_I420
    MVM_RGBD_DENOISE = 11  // Denosied disparity + YUV_I420
} mo_video_mode;

/**< struct define area */
typedef struct _mo_video_param_s {
    uint8_t u8VideoParamIndex;
    uint16_t u16ResolutionWidth;
    uint16_t u16ResolutionHeight;
    uint8_t u8DefaultFPS;
    uint8_t u8MaxFPS;
} mo_video_param;

typedef struct _mo_imu_data_set_s {
    uint64_t u64ImageFrameNum;  // Corresponding Image Frame number
    uint64_t u64Timestamp;      // Microsecond

    double dTemperature;  // Celsius degree

    double dAccelX;  // Meter per second squared
    double dAccelY;  // Meter per second squared
    double dAccelZ;  // Meter per second squared

    double dGyroX;  // Radian per second
    double dGyroY;  // Radian per second
    double dGyroZ;  // Radian per second
} mo_imu_data;

typedef struct _mo_rect_distance_info_s {
    struct _in_param_s {
        // Input Parameter
        // Region of interest described by rectangle with coordinate of top-left corner and width and height.
        uint16_t u16LeftTopCornerX;  // X coordinate of the top-left corner
        uint16_t u16LeftTopCornerY;  // Y coordinate of the top-left corner
        uint16_t u16RectWidth;       // Width  of the rectangle
        uint16_t u16RectHeight;      // Height of the rectangle

        // Buffer pointer to disparity data(obtained from moGetRGBDImage / moGetRGBDDisparityData function)
        uint16_t* pu16RGBDDisparityData;
    } stInParam;

    struct _out_param_s {
        // Output Parameter
        uint16_t u16MinimumDisparity;  // minimum disparity in region of interest
        uint16_t u16MaximumDisparity;  // maximum disparity in region of interest
        uint16_t u16MeanDisparity;     // mean    disparity in region of interest
        uint16_t u16MedianDisparity;   // median  disparity in region of interest

        float fDisparityDensity;  // disparity density in region of interest

        float fMinimumDepth;  // minimum depth in region of interest
        float fMaximumDepth;  // maximum depth in region of interest
        float fMeanDepth;     // mean    depth in region of interest
        float fMedianDepth;   // median  depth in region of interest

        float fMeanDistance;    // mean   distance in region of interest
        float fMedianDistance;  // median distance in region of interest
    } stOutParam;
} mo_rect_distance_info;

typedef struct _mo_camera_calibration_info_s {
    // IN - artificial setting
    float fSquareSideLen;
    uint8_t u8IterationTimes;
    float fDispOffsetAccuracyThreshold;
    float fLineOffsetAccuracyThreshold;

    // IN - from camera
    uint16_t u16FrameHeight;
    uint16_t u16FrameWidth;
    float fBxf;
    float fBase;
    float fFocus;

    // IN OUT
    float fDisparityOffset;
    float fLineOffset;
} mo_camera_calibration_info;

typedef struct _mo_camera_dev_info {
    char devName[16];
    char cameraSN[20];
    char aliasByPath[512];
    char aliasByV4lID[512];
} moCameraDevInfo;
typedef struct _mo_camera_list {
    moCameraDevInfo* pCameraInfos;
    uint32_t size;
} moCameraList;
#endif  // MO_STEREO_CAMERA_DRIVER_TYPE_DEFINE_INCLUDED
