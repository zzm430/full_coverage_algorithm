#ifndef MO_STEREO_CAMERA_DRIVER_C_CALIBRATION_H_INCLUDED
#define MO_STEREO_CAMERA_DRIVER_C_CALIBRATION_H_INCLUDED

#include <inttypes.h>

#include "opencv2/opencv.hpp"
using namespace cv;

#include "mo_stereo_camera_driver_macro_define.h"
#include "mo_stereo_camera_driver_type_define.h"

/**< function define area */
#ifdef __cplusplus
extern "C" {
#endif

/** \brief Detect chessboard corners
 *
 * \param matImage - the input image to detect chessboard
 * \param bIsPlotChessBoard - whether plot the chessboard
 * \return 0 - success, else - failure
 *        -1 - Didn't detect any chessboard
 */
DLL_API_C_DECL int32_t moDetectChessboardCorners(IN OUT Mat& matImage, IN bool bIsPlotChessBoard);

/** \brief Get line offset and disparity offset by calibration
 *
 * \param matLeftBGR  - the image of left  camera to calibrate camera
 * \param matRightBGR - the image of right camera to calibrate camera
 * \param stCameraCalibrationInfo - camera info to be used in calibration
 * \return 0 - success, else - failure
 *        -1 - Failed to get line offset and disparity offset
 */
DLL_API_C_DECL int32_t moCalculateLineOffsetAndDispOffset(IN Mat& matLeftBGR, IN Mat& matRightBGR,
                                                          IN OUT mo_camera_calibration_info& stCameraCalibrationInfo);

#ifdef __cplusplus
}
#endif

#endif  // MO_STEREO_CAMERA_DRIVER_C_CALIBRATION_H_INCLUDED
