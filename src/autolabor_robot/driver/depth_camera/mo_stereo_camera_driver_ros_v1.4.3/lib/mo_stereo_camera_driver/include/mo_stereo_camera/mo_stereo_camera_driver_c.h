#ifndef MO_SDK_C_API_H_INCLUDED
#define MO_SDK_C_API_H_INCLUDED

#include "mo_stereo_camera_driver_macro_define.h"
#include "mo_stereo_camera_driver_type_define.h"

/**< function define area */
#ifdef __cplusplus
extern "C" {
#endif

/** \brief Get current SDK version
 *
 * \param no
 * \return Version
 *
 */
DLL_API_C_DECL const char* moGetSdkVersion();

/** \brief Open UVC camera by device path
 *
 * \param pcPath - the specified path string
 *        e.g."/dev/video0"
 * \param phCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't open any camera
 */
DLL_API_C_DECL int32_t moOpenUVCCameraByPath(IN const char* const pcPath, OUT MO_CAMERA_HANDLE* const phCameraHandle);

/** \brief Open UVC camera by video device index
 *
 * \param u8Number - The index value in /dev/video*
 *                    e.g. 0 represents /dev/video0
 * \param phCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't open any camera
 */
DLL_API_C_DECL int32_t moOpenUVCCameraByNumber(IN const uint8_t u8Number, OUT MO_CAMERA_HANDLE* const phCameraHandle);

/** \brief Close specified camera with camera handle
 *
 * \param phCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 */
DLL_API_C_DECL int32_t moCloseCamera(IN MO_CAMERA_HANDLE* const phCameraHandle);

/** \brief Suspend video streaming of specified camera
 *
 * \param hCameraHandle - The handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSuspendCamera(IN const MO_CAMERA_HANDLE hCameraHandle);

/** \brief Resume video streaming of specified camera
 *
 * \param hCameraHandle - The handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moResumeCamera(IN const MO_CAMERA_HANDLE hCameraHandle);

/** \brief Get the BxF(baseline * focus) and baseline of camera
 *
 * \param hCameraHandle - the handle of camera
 * \param pfBxf - Output value of camera baseline * focus
 *        Focus = pfBxf / pfBase
 * \param pfBase - Output value of camera baseline
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetBxfAndBase(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* const pfBxf,
                                       OUT float* const pfBase);

/** \brief Get IMU data
 *
 * \param hCameraHandle - the handle of camera
 * \param ppstIMUData - Output IMU data
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetIMUData(IN const MO_CAMERA_HANDLE hCameraHandle, OUT mo_imu_data** const ppstIMUData);

/** \brief Get current frame data
 *        Note:
 *        Data buffer is circled and handled by SDK
 *        And data in the buffer may be refreshed after certain period
 *
 * \param hCameraHandle - the handle of camera
 * \param pu64ImageFrameNum - output frame ID
 * \param ppu8FrameBuffer - output pointer of the data buffer
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't get current frame
 */
DLL_API_C_DECL int32_t moGetCurrentFrame(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint64_t* const pu64ImageFrameNum,
                                         OUT uint8_t** const ppu8FrameBuffer);

/** \brief Get a data pointer to a safe buffer of current frame
 *        Note:
 *        Frame data will be copied from circled data buffer.
 *        Data will not be refreshed until another call of moGetSafeCurrentFrame
 *
 * \param hCameraHandle - the handle of camera
 * \param pu64ImageFrameNum - output frame ID
 * \param ppu8FrameBuffer - output pointer of the data buffer
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't get current frame
 */
DLL_API_C_DECL int32_t moGetSafeCurrentFrame(IN const MO_CAMERA_HANDLE hCameraHandle,
                                             OUT uint64_t* const pu64ImageFrameNum,
                                             OUT uint8_t** const ppu8FrameBuffer);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *         to get disparity and YUV_I420 data(valid in RGBD video mode)
 *
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu16DisparityData - Output disparity data(size: Width * Height * 2 bytes)
 *                             corresponding to right camera image
 * \param ppu8YUVI420Img -Output image(YUV_I420) data(size: Width * Height * 1.5 bytes)
 *                        in the type of YUV_I420 of right camera image
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRGBDImage(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t* const pu8FrameBuffer,
                                      OUT uint16_t** const ppu16DisparityData, OUT uint8_t** const ppu8YUVI420Img);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get disparity data(valid in RGBD video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer -  Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu16DisparityData - Output disparity data(: Width * Height * 2 bytes)
 *                             corresponding to right camera image
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRGBDDisparityData(IN const MO_CAMERA_HANDLE hCameraHandle,
                                              IN const uint8_t* const pu8FrameBuffer,
                                              OUT uint16_t** const ppu16DisparityData);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get YUV_I420 data(valid in RGBD video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8YUVI420Img - Output image(YUV_I420) data(size: Width * Height * 1.5 bytes)
 *                         in the type of YUV_I420 of right camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRGBDYUVI420Image(IN const MO_CAMERA_HANDLE hCameraHandle,
                                             IN const uint8_t* const pu8FrameBuffer,
                                             OUT uint8_t** const ppu8YUVI420Img);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get left and right raw image data (valid in RAW video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8LeftBayerImg - output original left bayer image data (size: Width * Height * 2 bytes)
 * \param ppu8RightBayerImg - output original right bayer image data (size: Width * Height * 2 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRawImage(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t* const pu8FrameBuffer,
                                     OUT uint8_t** const ppu8LeftBayerImg, OUT uint8_t** const ppu8RightBayerImg);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get left raw image data (valid in RAW video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8LeftBayerImg - output original left bayer image data (size: Width * Height * 2 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRawLeftBayerImage(IN const MO_CAMERA_HANDLE hCameraHandle,
                                              IN const uint8_t* const pu8FrameBuffer,
                                              OUT uint8_t** const ppu8LeftBayerImg);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *         to get right raw image data (valid in RAW video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8RightBayerImg - output original right bayer image data (size: Width * Height * 2 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRawRightBayerImage(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               IN const uint8_t* const pu8FrameBuffer,
                                               OUT uint8_t** const ppu8RightBayerImg);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get left and right rectified image data (valid in RECTIFIED video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8LeftGrayImg - output left gray rectified image data(Size: Width * Height * 1 bytes)
 * \param ppu8RightYUVI420Img - output right YUV_I420 rectified image data(Size: Width * Height * 1.5 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRectifiedImage(IN const MO_CAMERA_HANDLE hCameraHandle,
                                           IN const uint8_t* const pu8FrameBuffer, OUT uint8_t** const ppu8LeftGrayImg,
                                           OUT uint8_t** const ppu8RightYUVI420Img);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get left rectified image data (valid in RECTIFIED video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8LeftGrayImg - output left gray rectified image data(Size: Width * Height * 1 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRectifiedLeftGrayImage(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                   IN const uint8_t* const pu8FrameBuffer,
                                                   OUT uint8_t** const ppu8LeftGrayImg);

/** \brief Parse frame data(obtained by moGetCurrentFrame or moGetSafeCurrentFrame)
 *        to get right rectified image data (valid in RECTIFIED video mode)
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * \param ppu8RightYUVI420Img - output right YUV_I420 rectified image data(Size: Width * Height * 1.5 bytes)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetRectifiedRightYUVI420Image(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                       IN const uint8_t* const pu8FrameBuffer,
                                                       OUT uint8_t** const ppu8RightYUVI420Img);

/** \brief Set video mode
 *
 * \param hCameraHandle - the handle of camera
 * \param eVideoMode - video mode to be set
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetVideoMode(IN const MO_CAMERA_HANDLE hCameraHandle, IN const mo_video_mode eVideoMode);

/** \brief Get video mode
 *
 * \param hCameraHandle - the handle of camera
 * \param peVideoMode - output video mode
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 */
DLL_API_C_DECL int32_t moGetVideoMode(IN const MO_CAMERA_HANDLE hCameraHandle, OUT mo_video_mode* const peVideoMode);

/** \brief Query supported video parameter of specified camera
 *
 * \param hCameraHandle - the handle of camera
 * \param ppstVideoParamArray - output video parameter array
 *        Note:
 *        video parameter array is handled by SDK
 *        video parameter array will not be refreshed until another call of moQuerySupportedVideoParam
 * \param pArraySize - output a size of array
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moQuerySupportedVideoParam(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                  OUT mo_video_param** const ppstVideoParamArray,
                                                  OUT uint8_t* const pArraySize);

/** \brief Choise a index of video param
 *        Note: please take a little time to wait for completion.
 *
 * \param hCameraHandle - the handle of camera
 * \param u8VideoParamIndex - the index of video parameter obtained by moQuerySupportedVideoParam
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetVideoParam(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t u8VideoParamIndex);

/** \brief Get video frame param
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8VideoParamIndex - the index of video parameter obtained by moQuerySupportedVideoParam
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetVideoParam(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8VideoParamIndex);

/** \brief Get video resolution
 *
 * \param hCameraHandle - the handle of camera
 * \param pu16ResolutionWidth - the width of video resolution
 * \param pu16ResolutionHeight - the height of video resolution
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetVideoResolution(IN const MO_CAMERA_HANDLE hCameraHandle,
                                            OUT uint16_t* const pu16ResolutionWidth,
                                            OUT uint16_t* const pu16ResolutionHeight);

/** \brief Get real-time video frames per second
 *
 * \param hCameraHandle - the handle of camera
 * \param pd8FPS - the real-time video frames per second
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - Failed to get FPS(usually caused by insufficient frame counts during initial period)
 *
 */
DLL_API_C_DECL int32_t moGetRealTimeFPS(IN const MO_CAMERA_HANDLE hCameraHandle, OUT double* const pd8FPS);

/** \brief Set fill light type
 *
 * \param hCameraHandle - the handle of camera
 * \param eFilllightType - desired fill light type
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetFilllightType(IN const MO_CAMERA_HANDLE hCameraHandle,
                                          IN const mo_filllight_type eFilllightType);

/** \brief Get fill light type
 *
 * \param hCameraHandle - the handle of camera
 * \param peFilllightType - output current fill light type
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetFilllightType(IN const MO_CAMERA_HANDLE hCameraHandle,
                                          OUT mo_filllight_type* const peFilllightType);

/** \brief Get the Camera SN
 *
 * \param hCameraHandle - the handle of camera
 * \param pcSN - camera SN
 *        A pre-allocated char array with size greater than 16 is required. For instance: char cam_sn[20]
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetCameraSN(IN const MO_CAMERA_HANDLE hCameraHandle, OUT char* const pcSN);

/**
 * @brief Query current connected stereo camera info list
 *        Space of camera info list is handled by SDK, please free the list by moReleaseCameraList(...) to avoid
 * memory leak Function depends on camera attribute
 *
 * @param ppstCameraInfoList
 * @return 0 : Query success
 *        -1 : No camera device found
 */
DLL_API_C_DECL int32_t moQueryCameraList(OUT moCameraList** ppstCameraInfoList);

/**
 * @brief  Release moCameraInfoList
 *
 * @param ppstCameraInfoList
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int32_t moReleaseCameraList(OUT moCameraList** ppstCameraInfoList);

/**
 * @brief Retrive the Camera Device Name (i.e /dev/video0) By CameraSn
 *
 * @param pstCameraInfoList  Achieved by moQueryCameraList(...)
 * @param pcSN Camera SN
 * @return  Device name.
 *          NULL: if device does not exist
 */
DLL_API_C_DECL char* moGetCameraDeviceNameBySn(IN moCameraList* pstCameraInfoList, IN const char* pcSN);

/**
 * @brief Retrive the Camera usb path name (i.e /dev/v4l/by-path/...) By CameraSn
 *        The path is bind with USB port on your PC
 *
 * @param pstCameraInfoList  Achieved by moQueryCameraList(...)
 * @param pcSN Camera SN
 * @return  Device name.
 *          NULL: if device does not exist
 */
DLL_API_C_DECL char* moGetCameraUsbPathNameBySn(IN moCameraList* pstCameraInfoList, IN const char* pcSN);

DLL_API_C_DECL void moPrintCameraList(IN moCameraList* pstCameraInfoList);

/**
 * @brief Open camera by SN
 *
 * @param pstCameraInfoList Achieved by moQueryCameraList(...)
 * @param pcSN Camera Sn
 * @param phCameraHandle Out CameraHandle
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int32_t moOpenUVCCameraBySN(IN moCameraList* pstCameraInfoList, IN const char* const pcSN,
                                           OUT MO_CAMERA_HANDLE* const phCameraHandle);

/**
 * @brief Set camera fps
 *
 * @param hCameraHandle
 * @param _fps
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moSetCameraFps(IN const MO_CAMERA_HANDLE hCameraHandle, float _fps);

/**
 * @brief Restore camera fps setting to default value.
 *
 * @note  Reset on simor will happen and video stream will pause for a few seconds.
 *        Highly recommended that to apply this funcion without video stream on
 *
 * @param hCameraHandle
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moRestoreCameraFpsToDefault(IN const MO_CAMERA_HANDLE hCameraHandle);

/**
 * @brief Same as moGetRGBDImage with additional timestamp value
 *
 * @param hCameraHandle - the handle of camera
 * @param pu8FrameBuffer - Pointer of video frame data buffer(obtained from moGetCurrentFrame/moGetSafeCurrentFrame)
 * @param ppu16DisparityData - Output disparity data(size: Width * Height * 2 bytes)
 * @param ppu8YUVI420Img -Output image(YUV_I420) data(size: Width * Height * 1.5 bytes)
 *                        in the type of YUV_I420 of reference camera image
 * @param pu64Timestamp Frame timestamp based on simor clock. Unit: 16.67 ns (50/3 ns)
 * @return 0 - success, other - failure
 *
 */
DLL_API_C_DECL int32_t moGetRGBDImageWithTimestamp(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                   IN const uint8_t* const pu8FrameBuffer,
                                                   OUT uint16_t** const ppu16DisparityData,
                                                   OUT uint8_t** const ppu8YUVI420Img, OUT uint64_t* pu64Timestamp);
/**
 * @brief Same as moGetRectifiedImage with additional timestamp value
 *
 * @param hCameraHandle
 * @param pu8FrameBuffer
 * @param ppu8LeftGrayImg
 * @param ppu8RightYUVI420Img
 * @param pu64Timestamp
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int32_t moGetRectifiedImageWithTimestamp(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                        IN const uint8_t* const pu8FrameBuffer,
                                                        OUT uint8_t** const ppu8LeftGrayImg,
                                                        OUT uint8_t** const ppu8RightYUVI420Img,
                                                        OUT uint64_t* pu64Timestamp);
/**
 * @brief  Same as moGetRawImage with additional timestamp value
 *
 * @param hCameraHandle
 * @param pu8FrameBuffer
 * @param ppu8LeftBayerImg
 * @param ppu8RightBayerImg
 * @param pu64Timestamp
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int32_t moGetRawImageWithTimestamp(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                  IN const uint8_t* const pu8FrameBuffer,
                                                  OUT uint8_t** const ppu8LeftBayerImg,
                                                  OUT uint8_t** const ppu8RightBayerImg, OUT uint64_t* pu64Timestamp);

/**
 * @brief Set distance max and min threshold for dense disparity or denoise dispairty data.
 *        Valid only in mode MVM_RGBD_DENSE OR MVM_RGBD_DENOISE
 *
 * @param hCameraHandle
 * @param fMaxDistanceThres Max distance threshold in mm
 * @param fMinDistanceThres Min distance threshold in mm
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int32_t moSetDisparityFilterDistanceThreshold(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                             IN const float fMaxDistanceThres,
                                                             IN const float fMinDistanceThres);

#ifdef __cplusplus
}
#endif

#endif  // MO_SDK_C_API_H_INCLUDED
