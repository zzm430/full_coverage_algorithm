#ifndef MO_STEREO_CAMERA_DRIVER_C_UTILITIES_H_INCLUDED
#define MO_STEREO_CAMERA_DRIVER_C_UTILITIES_H_INCLUDED

#include "mo_stereo_camera_driver_macro_define.h"
#include "mo_stereo_camera_driver_type_define.h"

/**< function define area */
#ifdef __cplusplus
extern "C" {
#endif

/** \brief Calculate distance info of specified rectangle area
 *         RGBD video mode(default mode) is required to get disparity data.
 *
 *
 * \param hCameraHandle - The handle of camera
 * \param pstRectDistInfo - Distance info of specified rectangle area.
 *                          Input Parameter: stInParam
 *                              uint16_t u16LeftTopCornerX;
 *                              uint16_t u16LeftTopCornerY;
 *                              uint16_t u16RectWidth;
 *                              uint16_t u16RectHeight;
 *                              uint16_t* pu16RGBDDisparityData; // Data pointer
 * to disparity data
 *
 *
 * \return 0  - Success
 *         -1 - Camera handle is invalid
 *         -2 - Calculation operation fails
 */
DLL_API_C_DECL int32_t moCalculateRectDistance(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               IN OUT mo_rect_distance_info* const pstRectDistInfo);

/** \brief Convert disparity data to point cloud data
 *
 * \param hCameraHandle - The handle of camera
 * \param pu16RGBDDisparityData - Buffer pointer to disparity data(obtained from
 * moGetRGBDImage / moGetRGBDDisparityData function) \param pu32AxisArraySize -
 * the valid size of axis data buffer \param ppfXAxisArray - A one-dimensional
 * array for X axis data, data buffer is handled by SDK, don't free up the
 * memory! \param ppfYAxisArray - A one-dimensional array for Y axis data, data
 * buffer is handled by SDK, don't free up the memory! \param ppfZAxisArray
 * - A one-dimensional array for Z axis data, data buffer is handled by SDK,
 * don't free up the memory! \return 0 - success, else - failure -1 - parameter
 * is invalid -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moConvertDisparity2PointCloud(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                     IN const uint16_t* const pu16RGBDDisparityData,
                                                     OUT uint32_t* const pu32AxisArraySize,
                                                     OUT float** const ppfXAxisArray, OUT float** const ppfYAxisArray,
                                                     OUT float** const ppfZAxisArray);

#ifdef __cplusplus
}
#endif

#endif  // MO_STEREO_CAMERA_DRIVER_C_UTILITIES_H_INCLUDED
