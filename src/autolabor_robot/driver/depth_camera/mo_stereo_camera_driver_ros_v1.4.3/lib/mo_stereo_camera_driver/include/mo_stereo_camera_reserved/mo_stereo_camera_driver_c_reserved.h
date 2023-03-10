#ifndef MO_STEREO_CAMERA_DRIVER_C_RESERVED_H_INCLUDED
#define MO_STEREO_CAMERA_DRIVER_C_RESERVED_H_INCLUDED

#include <inttypes.h>

#include "mo_stereo_camera_driver_macro_define.h"
#include "mo_stereo_camera_driver_type_define.h"

/**< function define area */
#ifdef __cplusplus
extern "C" {
#endif

/** \brief Open UVC camera by device path, and call initCamCtrl
 *
 * \param pcPath - the specified path string
 *        e.g."/dev/video0"
 * \param phCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't open any camera
 */
DLL_API_C_DECL int32_t moOpenUVCCameraByPathCallInitCamCtrl(IN const char* const pcPath,
                                                            OUT MO_CAMERA_HANDLE* const phCameraHandle);

/** \brief Open UVC camera by device path, and call initMoakCameraSimple
 *
 * \param pcPath - the specified path string
 *        e.g."/dev/video0"
 * \param phCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - didn't open any camera
 */
DLL_API_C_DECL int32_t moOpenUVCCameraByPathCallInitMoakCameraSimple(IN const char* const pcPath,
                                                                     OUT MO_CAMERA_HANDLE* const phCameraHandle);

/** \brief Set search range
 *
 * \param hCameraHandle - the handle of camera
 * \param u8Value - a value of horizontal pixel offset of search range
 *        Possible values are 0 to 255
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetSearchRange(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t u8Value);

/** \brief Get search range
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8Value - points value of horizontal pixel offset of search range
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetSearchRange(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8Value);

/** \brief Set line offset
 *
 * \param hCameraHandle - the handle of camera
 * \param fLineOffset - a float value of line offset
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetLineOffset(IN const MO_CAMERA_HANDLE hCameraHandle, IN const float fLineOffset);

/** \brief Get line offset
 *
 * \param hCameraHandle - the handle of camera
 * \param pfLineOffset - points a float value of line offset
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetLineOffset(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* const pfLineOffset);

/** \brief Set disparity offset
 *
 * \param hCameraHandle - the handle of camera
 * \param fDispOffset - a float value of disparity offset
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetDispOffset(IN const MO_CAMERA_HANDLE hCameraHandle, IN const float fDispOffset);

/** \brief Get disparity offset
 *
 * \param hCameraHandle - the handle of camera
 * \param pfDispOffset - points a float value of disparity offset
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetDispOffset(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* const pfDispOffset);

/** \brief Set the line trim of the image
 *
 * \param hCameraHandle - the handle of camera
 * \param u16LeftLineTrim  - the value
 * \param u16RightLineTrim - the value
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetLineTrim(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint16_t u16LeftLineTrim,
                                     IN const uint16_t u16RightLineTrim);

/**
 * @brief Get the line trim of the image
 *
 * \param hCameraHandle - the handle of camera
 * \param pu16LeftLineTrim  - points a value
 * \param pu16RightLineTrim - points a value
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetLineTrim(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint16_t* const pu16LeftLineTrim,
                                     OUT uint16_t* const pu16RightLineTrim);

/** \brief Set exposure
 *
 * \param hCameraHandle - the handle of camera
 * \param u16Level - exposure level
 *        0 - automatic exposure
 *      >=1 - exposure level
 *            General range is 1 ~ 800.(1 leads to lowest image brightness)
 *            In door circumstance typical value: 300
 * \return 0 - success, else - failure
 *        -1 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetExposure(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint16_t u16Level);

/** \brief Get exposure
 *
 * \param hCameraHandle - the handle of camera
 * \param pu16Level - exposure level
 *        0 - automatic exposure
 *      >=1 - exposure level
 *            General range is 1 ~ 800.(1 leads to lowest image brightness)
 *            In door circumstance typical value: 300
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetExposure(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint16_t* const pu16Level);

/** \brief Set the Camera SN
 *
 * \param hCameraHandle - the handle of camera
 * \param pcSN - camera SN
 *        Size of camera SN should be no larger than 16
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetCameraSN(IN const MO_CAMERA_HANDLE hCameraHandle, IN const char* const pcSN);

/** \brief Erase Camera SN
 *
 * \param hCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moEraseCameraSN(IN const MO_CAMERA_HANDLE hCameraHandle);

/** \brief Get USB version
 *
 * \param hCameraHandle - the handle of camera
 * \param pcVersion - the version of USB
 *        A char array with size greater than 32 is required
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetUsbVersion(IN const MO_CAMERA_HANDLE hCameraHandle, OUT char* const pcVersion);

/** \brief Get Fpga Release Date
 *        Please note that YY/MM/DD is saved in hex
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8Year - Year
 * \param pu8Month - Month
 * \param pu8Day - Day
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetFpgaReleaseDate(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8Year,
                                            OUT uint8_t* const pu8Month, OUT uint8_t* const pu8Day);

/** \brief Get FPGA Version
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8LogicVersion - the version of logic
 * \param pu8PatchVersion - the version of patch
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetFpgaVersion(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8LogicVersion,
                                        OUT uint8_t* const pu8PatchVersion);

/** \brief Upload Fpga bit file
 *        Hard restart is required for bit taking effect
 *
 * \param hCameraHandle - the handle of camera
 * \param pcFpgaBitFile - the path of FPGA bit file(with file name)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moUploadFpgaBitFile(IN const MO_CAMERA_HANDLE hCameraHandle, IN const char* const pcFpgaBitFile);

/** \brief Upload epr file to storage slot and make it effect in camera
 *
 * \param hCameraHandle - the handle of camera
 * \param pcEprFile - the path of epr file(with file name)
 * \param pstVideoParam - Video frame parameter obtained by EPR file publisher
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moUploadEprFile(IN const MO_CAMERA_HANDLE hCameraHandle, IN const char* const pcEprFile,
                                       IN const mo_video_param* const pstVideoParam);

/** \brief Amend epr info(about video frame parameter)
 *        Designed for amend epr info in old camera(if needed)
 *
 * \param hCameraHandle - the handle of camera
 * \param pstVideoParam - Video frame parameter obtained by EPR file publisher
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moAmendVideoFrameParam(IN const MO_CAMERA_HANDLE hCameraHandle,
                                              IN const mo_video_param* const pstVideoParam);

/** \brief Upload epr file to storage slot (file will not be effective)
 *
 * \param hCameraHandle - the handle of camera
 * \param pcEprFile - the path of epr file(with file name)
 * \param u8Slot - the index of storage slot
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moStoreEprFile(IN const MO_CAMERA_HANDLE hCameraHandle, IN const char* const pcEprFile,
                                      IN const uint8_t u8Slot);

/** \brief Upload epr file to storage slot (file will not be effective)
 *
 * \param hCameraHandle - the handle of camera
 * \param pcEprFile - the path of epr file(with file name)
 * \param pstVideoParam - Video frame parameter obtained by EPR file publisher
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moStoreEprFileWithVideoFrameParam(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                         IN const char* const pcEprFile,
                                                         IN const mo_video_param* const pstVideoParam);

/** \brief Select epr file in specified slot and make it effect
 *
 * \param hCameraHandle - the handle of camera
 * \param u8Slot - the index of storage slot
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSelectEprFile(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t u8Slot);

/** \brief Print all custom eeprom info (only for developing purpose)
 *
 * \param hCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moShowAllCustomEEPROM(IN const MO_CAMERA_HANDLE hCameraHandle);

/** \brief Set the Camera Gain
 *        Implement to fix OV9281 origin image issue
 *        Trying to limit exposure no larger than 150 to prevent corrupted original image
 * \param hCameraHandle - the handle of camera
 * \param u16GainValue - Default val: 0x10 (represents 1.0).
 *                       Typical val: 0x20 (represents 2.0)
 *                                    0x18 (represents 1.5)
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSetCameraGain(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint16_t u16GainValue);

/** \brief Get the Camera Hardware info from FPGA
 *
 * \param hCameraHandle - the handle of camera
 * \param pu8PCBType  - 0xA USB board
 *                      0xB FPD board
 * \param pu8CMOSType - 0x1 IMX224
 *                      0x2 OV9281
 *                      0x3 AR0144
 *                      0x4 AR0132
 *                      0x5 SC132GS
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetCameraHardwareInfo(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8PCBType,
                                               OUT uint8_t* const pu8CMOSType);

/** \brief Get the Camera CMOS Swap Mode
 *        Note: (2021/09/02)
 *        As to OV9281 setting, swap_mode would be 0.
 *        This function has not been fully verified for other conditions
 * \param hCameraHandle - the handle of camera
 * \param pu8SwapMode - CMOS Swap Mode
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moGetCameraCMOSSwapMode(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* const pu8SwapMode);

/** \brief Switch to origin disparity
 *
 * \param hCameraHandle - the handle of camera
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moSwitchToOriginDisparity(IN const MO_CAMERA_HANDLE hCameraHandle);

/** \brief Write the Simor Register with value
 *        Warning:
 *        Only used for specific developing purpose.
 *        Instructions from hardware department is required.
 *
 * \param hCameraHandle - the handle of camera
 * \param u16RegisterAddress - the address of register
 * \param u16RegisterValue - the value of register
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moWriteSimorRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                 IN const uint16_t u16RegisterAddress,
                                                 IN const uint16_t u16RegisterValue);

/** \brief Read the Simor Register with value
 *        Warning:
 *        Only used for specific developing purpose.
 *        Instructions from hardware department is required.
 *
 * \param hCameraHandle - the handle of camera
 * \param u16RegisterAddress - the address of register
 * \param pu16RegisterValue - the value of register
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 *        -2 - command fails unexpectedly
 */
DLL_API_C_DECL int32_t moReadSimorRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                IN const uint16_t u16RegisterAddress,
                                                OUT uint16_t* const pu16RegisterValue);
/**
 * @brief Open SDK log with log level
 *
 * @param loglevel
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL void moOpenSDKLog(int32_t const loglevel);
/**
 * @brief Shutdown SDK log
 *
 * @param loglevel
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL void moCloseSDKLog();

/**
 * @brief Upload EPR file(only to storage) and cam info based on auto calibration output files(erp file and log file)
 *
 * @param hCameraHandle
 * @param uSlot
 * @param pcEprDirectory
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moUploadCameraEprForAutoCalibration(IN const MO_CAMERA_HANDLE hCameraHandle, IN const uint8_t uSlot,
                                                       IN const char* pcEprDirectory);

/**
 * @brief BackUp Current Effective Epr File of simor camera
 *
 * @param hCameraHandle
 * @param pcDestEprFullFileName destination epr full file name
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moBackUpEffectiveEprFile(IN const MO_CAMERA_HANDLE hCameraHandle,
                                            IN const char* pcDestEprFullFileName);
/**
 * @brief Reset simor to initial state with cleared eeprom data
 *
 * @param hCameraHandle
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moResetCameraToFactoryState(IN const MO_CAMERA_HANDLE hCameraHandle);

/**
 * @brief Get Optical Center of Rectified Cam
 *
 * @param hCameraHandle
 * @param _pfCx
 * @param _pfCy
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moGetRectifyOpticalCenter(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* _pfCx,
                                             OUT float* _pfCy);
/**
 * @brief Get Rectified Cam Parameter
 *
 * @param hCameraHandle
 * @param _pfFocus
 * @param _pfCx
 * @param _pfCy
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moGetRectifyCamParam(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* _pfFocus, OUT float* _pfCx,
                                        OUT float* _pfCy);
/**
 * @brief Check simor chip working stauts
 *
 * @param hCameraHandle
 * @return -1 FPGA status err
 *         -2 Simor Default Value err
 *         -3 EPR init err
 */
DLL_API_C_DECL int moCheckSimorChipStatus(IN const MO_CAMERA_HANDLE hCameraHandle);

/**
 * @brief Check CMOS Workikng status
 *
 * @param hCameraHandle
 * @return  0 ok
 *         -1 cmosA err
 *         -2 cmosB err
 *         -3 both cmos err
 *         -99 Read CMOS idx err
 */
DLL_API_C_DECL int moCheckCMOSStatus(IN const MO_CAMERA_HANDLE hCameraHandle);

DLL_API_C_DECL int moInitMetaDataSHM();
DLL_API_C_DECL int moReleaseMetaDataSHM();
DLL_API_C_DECL int moGetNextMetaData(uint16_t** pMetaData, uint32_t* pFrameID);

DLL_API_C_DECL const char* moGetImplLevelVersion(IN const MO_CAMERA_HANDLE hCameraHandle);
/**
 * @brief Get lastest meta data buffer pointer.
 *        moGetCurrentFrame and any frame parser (i.e moGetRGBDImage) are needed to get the latest data pointer
 *
 * @param MO_CAMERA_HANDLE
 * @param ppMetaData
 * @return 0
 */
DLL_API_C_DECL int moGetLastestMetaDataPointer(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint16_t** ppMetaData);

DLL_API_C_DECL int moGetCmosType(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint8_t* _pu8CmosCode,
                                 OUT const char** _ppc_cmos_name);

/**
 * @brief Set Fill Light Intensity
 *
 * @param hCameraHandle
 * @param _light_intensity_val Range is sfrom 0 to 128.
 * @return
 */
DLL_API_C_DECL int32_t moSetFillLightIntensity(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               IN uint8_t _u8_light_intensity_val);
/**
 * @brief Get Fill Light Intensity
 *
 * @param hCameraHandle
 * @param _pu8_light_intensity_val
 * @return
 */
DLL_API_C_DECL int32_t moGetFillLightIntensity(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               OUT uint8_t* _pu8_light_intensity_val);
/**
 * @brief Get Image channel info from source image data.
 *
 * @param hCameraHandle
 * @param _pu8_src_data Data pointer obtained by moGetCurrentFrame or moGetSafeCurrentFrame
 * @param _pu8_channel_val Internal channel setting value defined by hardware
 * @return Channel mode defined by enum mo_video_mode
 */
DLL_API_C_DECL int32_t moGetImageChannelFromSrcData(IN const MO_CAMERA_HANDLE hCameraHandle, IN uint8_t* _pu8_src_data,
                                                    OUT uint8_t* _pu8_channel_val);

DLL_API_C_DECL int32_t moWriteFpgaRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                IN const uint16_t u16RegisterAddress,
                                                IN const uint16_t u16RegisterValue);

DLL_API_C_DECL int32_t moReadFpgaRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               IN const uint16_t u16RegisterAddress,
                                               OUT uint16_t* const pu16RegisterValue);

DLL_API_C_DECL int32_t moWriteCmosRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                IN const uint16_t u16RegisterAddress,
                                                IN const uint16_t u16RegisterValue);

DLL_API_C_DECL int32_t moReadCmosRegisterValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                               IN const uint16_t u16RegisterAddress,
                                               OUT uint16_t* const pu16RegisterValue);

DLL_API_C_DECL int32_t moWriteEEpromAddressValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                 IN const uint16_t u16RegisterAddress,
                                                 IN const uint16_t u16RegisterValue);

DLL_API_C_DECL int32_t moReadEEpromAddressValue(IN const MO_CAMERA_HANDLE hCameraHandle,
                                                IN const uint16_t u16RegisterAddress,
                                                OUT uint16_t* const pu16RegisterValue);
/**
 * @brief Get the lastest stat value of 5 zones on Image for ae ctrl.
 *         moGetCurrentFrame() and moGetRGBDImage is required for data updating
 *        |            Zone 0              |
 *        | Zone 1 |   Zone 2   |  Zone 3  |
 *        |            Zone 4              |
 *        Data space is handled by SDK.
 *        Example: Value of Zone 0: (*_pp_ae_stats)[0]
 *
 *
 * @param MO_CAMERA_HANDLE
 * @param _pp_ae_stats
 * @return DLL_API_C_DECL
 */
DLL_API_C_DECL int moGetLastestAeStatValues(IN const MO_CAMERA_HANDLE hCameraHandle, OUT uint32_t** _pp_ae_stats);
#ifdef __cplusplus
}
#endif

#endif  // MO_STEREO_CAMERA_DRIVER_C_RESERVED_H_INCLUDED
