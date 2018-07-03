/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 *
 * 1. Definitions
 *
 * "Authorized Accessory" means a STEREOLABS branded ZED or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera and the ZED Mini Camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, pre-installed in the ZED USB flash drive
 * included in the ZED packaging, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED or ZED Mini Camera.
 *
 * 2. License
 *
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as pre-installed in Your ZED USB flash drive, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * b. As conditions to this Software license, You agree that:
 *	i. You will use Your Software with ZED or ZED Mini Camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *	ii. You will not use or install any Unauthorized Software. If You do, Your ZED
 *       or ZED Mini Camera may stop working permanently at that time or after a later
 *       Software update.
 *	iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED or ZED Mini Camera may stop
 *        working permanently at that time or after a later Software update.
 *	iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED or ZED Mini Camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED or ZED Mini Camera.
 *	v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 *
 * 3. Warranty
 * The Software is covered by the Limited Warranty for Your ZED or ZED Mini Camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 *
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 *
 * 5. Choice of Law
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */
#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__

#include <sl_core/utils/types.hpp>

#if defined WIN32
#if defined(SL_SDK_COMPIL)
#define SL_SDK_EXPORT __declspec(dllexport)
#else
#define SL_SDK_EXPORT
#endif
#elif __GNUC__
#define SL_SDK_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

 // SDK VERSION NUMBER
#define ZED_SDK_MAJOR_VERSION 2
#define ZED_SDK_MINOR_VERSION 4
#define ZED_SDK_PATCH_VERSION 1


#define ZED_SDK_VERSION_ATTRIBUTE private: uint32_t _zed_sdk_major_version = ZED_SDK_MAJOR_VERSION, _zed_sdk_minor_version = ZED_SDK_MINOR_VERSION, _zed_sdk_patch_version = ZED_SDK_PATCH_VERSION;

// Dynamic version verification

/**
 * \ingroup Core_group
 * \brief [Windows only] Returns the ZED SDK version currently installed on the computer.
 * The major, minor, patch parameters will be filled by reference.
 * \return -1 if the SDK wasn't found, -2 if the version wasn't found, 0 if success. Will return -3 on Linux.
 * \note Should be used on Windows with a /delayload on the ZED SDK DLLs.
 */

inline int getZEDSDKRuntimeVersion(int &major, int& minor, int& patch) {
#if defined WIN32
#ifdef UNICODE
    HINSTANCE hGetProcIDDLL = LoadLibrary(L"sl_zed64.dll");
#else
    HINSTANCE hGetProcIDDLL = LoadLibrary("sl_zed64.dll");
#endif

    if(!hGetProcIDDLL)
        return -1; //could not load the dynamic library

    typedef void(__stdcall *f_func)(int&a, int&b, int&c);
    f_func func = reinterpret_cast<f_func> (reinterpret_cast<void*> (GetProcAddress(hGetProcIDDLL, "getZEDSDKBuildVersion")));
    if(!func)
        return -2; //could not locate the function

    func(major, minor, patch);
    return 0;
#else
    return -3; //on Linux
#endif
}

extern "C" {

    /**
     * \ingroup Core_group
     * \brief Returns the ZED SDK version which the current program has been compiled with.
     * The major, minor, patch parameters will be filled by reference.
     * \code{.cpp}
    // [Windows only]
    //using /delayload on the ZED SDK DLLs, this snippet gives you boolean to decide if the SDK should be loaded.
    int major_dll, minor_dll, patch_dll;
    int major_prg, minor_prg, patch_prg;
    bool older_ZED_SDK_available = false;
    bool same_ZED_SDK_available = false;
    bool newer_ZED_SDK_available = false;
    bool ZED_SDK_available = false;
    if (getZEDSDKRuntimeVersion(major_dll, minor_dll, patch_dll) == 0) {
            getZEDSDKBuildVersion(major_prg, minor_prg, patch_prg);
            if (major_dll == major_prg && minor_dll == minor_prg && patch_dll == patch_prg)
                    same_ZED_SDK_available = true;
            if (major_dll > major_prg || (major_dll == major_prg && minor_dll > minor_prg) || (major_dll == major_prg && minor_dll == minor_prg && patch_dll > patch_prg))
                    newer_ZED_SDK_available = true;
            else
                    older_ZED_SDK_available = true;
            if (older_ZED_SDK_available || same_ZED_SDK_available || newer_ZED_SDK_available)
                    ZED_SDK_available = true;
    }
    //use ZED_SDK_available or older_ZED_SDK_available or same_ZED_SDK_available or newer_ZED_SDK_available
     * \endcode
     */
    inline const void /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ getZEDSDKBuildVersion(int &major, int& minor, int& patch) {
        major = ZED_SDK_MAJOR_VERSION;
        minor = ZED_SDK_MINOR_VERSION;
        patch = ZED_SDK_PATCH_VERSION;
    }
}

namespace sl {

    ///@{
    ///  @name Unavailable Values
    /**
    Defines an unavailable depth value that is above the depth Max value.
     */
    static const float TOO_FAR = INFINITY;
    /**
    Defines an unavailable depth value that is below the depth Min value.
     */
    static const float TOO_CLOSE = -INFINITY;
    /**
    Defines an unavailable depth value that is on an occluded image area.
     */
    static const float OCCLUSION_VALUE = NAN;
    ///@}

    static const float INVALID_VALUE = NAN;

    //macro to detect wrong data measure
#define isValidMeasure(v) (std::isfinite(v))

    /// \defgroup Video_group Video classes
    /// \defgroup Depth_group Depth Sensing classes												   
    /// \defgroup Core_group Core classes
    /// \defgroup SpatialMapping_group Spatial Mapping classes
    /// \defgroup PositionalTracking_group Positional Tracking classes

    /**
    \enum RESOLUTION
    \ingroup Video_group
    \ingroup Enumerations
    \brief Represents the available resolution defined in sl::cameraResolution.
    \note Since v1.0, RESOLUTION_VGA mode has been updated to WVGA (from 640*480 to 672*376) and requires a firmware update to function (>= 1142). Firmware can be updated in the ZED Explorer.
    \warning NVIDIA Jetson X1 only supports RESOLUTION_HD1080@15, RESOLUTION_HD720@30/15, and RESOLUTION_VGA@60/30/15.
     */
    enum RESOLUTION {
        RESOLUTION_HD2K, /**< 2208*1242, available framerates: 15 fps.*/
        RESOLUTION_HD1080, /**< 1920*1080, available framerates: 15, 30 fps.*/
        RESOLUTION_HD720, /**< 1280*720, available framerates: 15, 30, 60 fps.*/
        RESOLUTION_VGA, /**< 672*376, available framerates: 15, 30, 60, 100 fps.*/
        RESOLUTION_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const RESOLUTION &resolution);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const RESOLUTION &resolution) {
        return os << toString(resolution);
    }
    ///@endcond

    /**
    \enum CAMERA_SETTINGS
    \ingroup Video_group
    \brief Lists available camera settings for the ZED camera (contrast, hue, saturation, gain...).
    \warning CAMERA_SETTINGS_GAIN and CAMERA_SETTINGS_EXPOSURE are linked in auto/default mode (see \ref sl::Camera::setCameraSettings).
    \brief Each enum defines one of those settings.
     */
    enum CAMERA_SETTINGS {
        CAMERA_SETTINGS_BRIGHTNESS, /**< Defines the brightness control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_CONTRAST, /**< Defines the contrast control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_HUE, /**< Defines the hue control. Affected value should be between 0 and 11.*/
        CAMERA_SETTINGS_SATURATION, /**< Defines the saturation control. Affected value should be between 0 and 8.*/
        CAMERA_SETTINGS_GAIN, /**< Defines the gain control. Affected value should be between 0 and 100 for manual control.*/
        CAMERA_SETTINGS_EXPOSURE, /**< Defines the exposure control. Affected value should be between 0 and 100 for manual control.*/
        CAMERA_SETTINGS_WHITEBALANCE, /**< Defines the color temperature control. Affected value should be between 2800 and 6500 with a step of 100.*/
        CAMERA_SETTINGS_AUTO_WHITEBALANCE, /**< Defines the status of white balance (automatic or manual). A value of 0 disable the AWB, while 1 activate it.*/
        CAMERA_SETTINGS_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const CAMERA_SETTINGS &camSettings);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const CAMERA_SETTINGS &camSettings) {
        return os << toString(camSettings);
    }
    ///@endcond

    /**
    \enum SELF_CALIBRATION_STATE
    \ingroup Video_group
    \brief Status for self calibration. Since v0.9.3, self-calibration is done in background and start in the sl::Camera::open or Reset function.
    \brief You can follow the current status for the self-calibration any time once ZED object has been constructed.
     */
    enum SELF_CALIBRATION_STATE {
        SELF_CALIBRATION_STATE_NOT_STARTED, /**< Self calibration has not run yet (no sl::Camera::open or sl::Camera::resetSelfCalibration called).*/
        SELF_CALIBRATION_STATE_RUNNING, /**< Self calibration is currently running.*/
        SELF_CALIBRATION_STATE_FAILED, /**< Self calibration has finished running but did not manage to get accurate values. Old parameters are taken instead.*/
        SELF_CALIBRATION_STATE_SUCCESS, /**< Self calibration has finished running and did manage to get accurate values. New parameters are set.*/
        SELF_CALIBRATION_STATE_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SELF_CALIBRATION_STATE &selfCalibState);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const SELF_CALIBRATION_STATE &selfCalibState) {
        return os << toString(selfCalibState);
    }
    ///@endcond

    /**
    \enum DEPTH_MODE
    \ingroup Depth_group
    \brief Lists available depth computation modes.
     */
    enum DEPTH_MODE {
        DEPTH_MODE_NONE, /**< This mode does not compute any depth map. Only rectified stereo images will be available.*/
        DEPTH_MODE_PERFORMANCE, /**< Computation mode optimized for speed.*/
        DEPTH_MODE_MEDIUM, /**< Balanced quality mode. Depth map is robust in any environment and requires medium resources for computation.*/
        DEPTH_MODE_QUALITY, /**< Computation mode designed for high quality results.*/
        DEPTH_MODE_ULTRA, /**< Computation mode favorising edges and sharpness. Requires lot of GPU memory and high computation power in /ref SENSING_MODE_FILL.*/
        DEPTH_MODE_LAST
    };

    /**
    \ingroup Depth_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const DEPTH_MODE &depthMode);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const DEPTH_MODE &depthMode) {
        return os << toString(depthMode);
    }
    ///@endcond

    /**
    \enum SENSING_MODE
    \ingroup Depth_group
    \brief Lists available depth sensing modes.
     */
    enum SENSING_MODE {
        SENSING_MODE_STANDARD, /**< This mode outputs ZED standard depth map that preserves edges and depth accuracy.
                               * Applications example: Obstacle detection, Automated navigation, People detection, 3D reconstruction.*/
        SENSING_MODE_FILL, /**< This mode outputs a smooth and fully dense depth map.
                           * Applications example: AR/VR, Mixed-reality capture, Image post-processing.*/
        SENSING_MODE_LAST
    };

    /**
    \ingroup Depth_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SENSING_MODE &sensingMode);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const SENSING_MODE &sensingMode) {
        return os << toString(sensingMode);
    }
    ///@endcond

    /**
    \enum MEASURE
    \ingroup Depth_group
    \brief Lists retrievable measures.
     */
    enum MEASURE {
        MEASURE_DISPARITY, /**< Disparity map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_DEPTH, /**< Depth map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_CONFIDENCE, /**< Certainty/confidence of the depth map, sl::MAT_TYPE_32F_C1.*/
        MEASURE_XYZ, /**< Point cloud, sl::MAT_TYPE_32F_C4, channel 4 is empty.*/
        MEASURE_XYZRGBA, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in R-G-B-A order.*/
        MEASURE_XYZBGRA, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in B-G-R-A order.*/
        MEASURE_XYZARGB, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in A-R-G-B order.*/
        MEASURE_XYZABGR, /**< Colored point cloud,  sl::MAT_TYPE_32F_C4, channel 4 contains color in A-B-G-R order.*/
        MEASURE_NORMALS, /**< Normals vector,  sl::MAT_TYPE_32F_C4, channel 4 is empty (set to 0)*/
        MEASURE_DISPARITY_RIGHT, /**< Disparity map for right sensor, sl::MAT_TYPE_32F_C1.*/
        MEASURE_DEPTH_RIGHT, /**< Depth map for right sensor, sl::MAT_TYPE_32F_C1.*/
        MEASURE_XYZ_RIGHT, /**< Point cloud for right sensor, sl::MAT_TYPE_32F_C1, channel 4 is empty.*/
        MEASURE_XYZRGBA_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in R-G-B-A order.*/
        MEASURE_XYZBGRA_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in B-G-R-A order.*/
        MEASURE_XYZARGB_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in A-R-G-B order.*/
        MEASURE_XYZABGR_RIGHT, /**< Colored point cloud for right sensor, sl::MAT_TYPE_32F_C4, channel 4 contains color in A-B-G-R order.*/
        MEASURE_NORMALS_RIGHT, /**< Normals vector for right view, sl::MAT_TYPE_32F_C4, channel 4 is empty (set to 0)*/
        MEASURE_LAST
    };

    /**
    \ingroup Depth_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const MEASURE &measure);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const MEASURE &measure) {
        return os << toString(measure);
    }
    ///@endcond

    /**
    \enum VIEW
    \ingroup Video_group
    \brief Lists available views.
     */
    enum VIEW {
        VIEW_LEFT, /**< Left RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_RIGHT, /**< Right RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_LEFT_GRAY, /**< Left GRAY image, sl::MAT_TYPE_8U_C1. */
        VIEW_RIGHT_GRAY, /**< Right GRAY image, sl::MAT_TYPE_8U_C1. */
        VIEW_LEFT_UNRECTIFIED, /**< Left RGBA unrectified image, sl::MAT_TYPE_8U_C4. */
        VIEW_RIGHT_UNRECTIFIED, /**< Right RGBA unrectified image, sl::MAT_TYPE_8U_C4. */
        VIEW_LEFT_UNRECTIFIED_GRAY, /**< Left GRAY unrectified image, sl::MAT_TYPE_8U_C1. */
        VIEW_RIGHT_UNRECTIFIED_GRAY, /**< Right GRAY unrectified image, sl::MAT_TYPE_8U_C1. */
        VIEW_SIDE_BY_SIDE, /**< Left and right image (the image width is therefore doubled). RGBA image, sl::MAT_TYPE_8U_C4. */
        VIEW_DEPTH, /**< Color rendering of the depth, sl::MAT_TYPE_8U_C4. */
        VIEW_CONFIDENCE, /**< Color rendering of the depth confidence, sl::MAT_TYPE_8U_C4. */
        VIEW_NORMALS, /**< Color rendering of the normals, sl::MAT_TYPE_8U_C4. */
        VIEW_DEPTH_RIGHT, /**< Color rendering of the right depth mapped on right sensor, sl::MAT_TYPE_8U_C4. */
        VIEW_NORMALS_RIGHT, /**< Color rendering of the normals mapped on right sensor, sl::MAT_TYPE_8U_C4. */
        VIEW_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const VIEW &view);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const VIEW &view) {
        return os << toString(view);
    }
    ///@endcond

    /**
    \enum TIME_REFERENCE
    \ingroup Video_group
    \brief Lists specific and particular timestamps
     */
    enum TIME_REFERENCE {
        TIME_REFERENCE_IMAGE, /**< Defines the timestamp at the time the frame has been extracted from USB stream. */
        TIME_REFERENCE_CURRENT, /**<  Defines the timestamp at the time of the function call. */
        TIME_REFERENCE_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const TIME_REFERENCE &time_reference);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const TIME_REFERENCE &time_reference) {
        return os << toString(time_reference);
    }
    ///@endcond

    /**
    \enum DEPTH_FORMAT
    \ingroup Depth_group
    \brief Lists available file formats for saving depth maps.
     */
    enum DEPTH_FORMAT {
        DEPTH_FORMAT_PNG, /**< PNG image format in 16bits. 32bits depth is mapped to 16bits color image to preserve the consistency of the data range.*/
        DEPTH_FORMAT_PFM, /**< stream of bytes, graphic image file format.*/
        DEPTH_FORMAT_PGM, /**< gray-scale image format.*/
        DEPTH_FORMAT_LAST
    };

    /**
    \ingroup Depth_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const DEPTH_FORMAT &depth_frmt);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const DEPTH_FORMAT &depth_frmt) {
        return os << toString(depth_frmt);
    }
    ///@endcond

    /**
    \enum POINT_CLOUD_FORMAT
    \ingroup Depth_group
    \brief Lists available file formats for saving point clouds. Stores the spatial coordinates (x,y,z) of each pixel and optionally its RGB color.
     */
    enum POINT_CLOUD_FORMAT {
        POINT_CLOUD_FORMAT_XYZ_ASCII, /**< Generic point cloud file format, without color information.*/
        POINT_CLOUD_FORMAT_PCD_ASCII, /**< Point Cloud Data file, with color information.*/
        POINT_CLOUD_FORMAT_PLY_ASCII, /**< PoLYgon file format, with color information.*/
        POINT_CLOUD_FORMAT_VTK_ASCII, /**< Visualization ToolKit file, without color information.*/
        POINT_CLOUD_FORMAT_LAST
    };

    /**
    \ingroup Depth_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const POINT_CLOUD_FORMAT &pc_frmt);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const POINT_CLOUD_FORMAT &pc_frmt) {
        return os << toString(pc_frmt);
    }
    ///@endcond

    /**
    \enum TRACKING_STATE
    \ingroup PositionalTracking_group
    \brief Lists the different states of positional tracking.
     */
    enum TRACKING_STATE {
        TRACKING_STATE_SEARCHING, /**< The camera is searching for a previously known position to locate itself.*/
        TRACKING_STATE_OK, /**< Positional tracking is working normally.*/
        TRACKING_STATE_OFF, /**< Positional tracking is not enabled.*/
        TRACKING_STATE_FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720))*/
        TRACKING_STATE_LAST
    };

    /**
    \ingroup PositionalTracking_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const TRACKING_STATE &track_state);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const TRACKING_STATE &track_state) {
        return os << toString(track_state);
    }
    ///@endcond

    /**
    \enum AREA_EXPORT_STATE
    \ingroup SpatialMapping_group
    \brief Lists the different states of spatial memory area export.
     */
    enum AREA_EXPORT_STATE {
        AREA_EXPORT_STATE_SUCCESS, /**< The spatial memory file has been successfully created.*/
        AREA_EXPORT_STATE_RUNNING, /**< The spatial memory is currently written.*/
        AREA_EXPORT_STATE_NOT_STARTED, /**< The spatial memory file exportation has not been called.*/
        AREA_EXPORT_STATE_FILE_EMPTY, /**< The spatial memory contains no data, the file is empty.*/
        AREA_EXPORT_STATE_FILE_ERROR, /**< The spatial memory file has not been written because of a wrong file name.*/
        AREA_EXPORT_STATE_SPATIAL_MEMORY_DISABLED, /**< The spatial memory learning is disable, no file can be created.*/
        AREA_EXPORT_STATE_LAST
    };

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const AREA_EXPORT_STATE &area_export);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const AREA_EXPORT_STATE &area_export) {
        return os << toString(area_export);
    }
    ///@endcond

    /**
    \enum REFERENCE_FRAME
    \ingroup PositionalTracking_group
    \brief Defines which type of position matrix is used to store camera path and pose.
     */
    enum REFERENCE_FRAME {
        REFERENCE_FRAME_WORLD, /**< The transform of sl::Pose will contains the motion with reference to the world frame (previously called PATH).*/
        REFERENCE_FRAME_CAMERA, /**< The transform of sl::Pose will contains the motion with reference to the previous camera frame (previously called POSE).*/
        REFERENCE_FRAME_LAST
    };

    /**
    \ingroup PositionalTracking_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const REFERENCE_FRAME &ref_frame);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const REFERENCE_FRAME &ref_frame) {
        return os << toString(ref_frame);
    }
    ///@endcond

    /**
    \enum SPATIAL_MAPPING_STATE
    \ingroup SpatialMapping_group
    \brief Gives the spatial mapping state.
     */
    enum SPATIAL_MAPPING_STATE {
        SPATIAL_MAPPING_STATE_INITIALIZING, /**< The spatial mapping is initializing.*/
        SPATIAL_MAPPING_STATE_OK, /**< The depth and tracking data were correctly integrated in the fusion algorithm.*/
        SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY, /**< The maximum memory dedicated to the scanning has been reach, the mesh will no longer be updated.*/
        SPATIAL_MAPPING_STATE_NOT_ENABLED, /**< Camera::enableSpatialMapping() wasn't called (or the scanning was stopped and not relaunched).*/
        SPATIAL_MAPPING_STATE_FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for spatial mapping. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720), spatial mapping low resolution)*/
        SPATIAL_MAPPING_STATE_LAST
    };

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SPATIAL_MAPPING_STATE &mapping_state);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const SPATIAL_MAPPING_STATE &mapping_state) {
        return os << toString(mapping_state);
    }
    ///@endcond

    /**
    \enum SVO_COMPRESSION_MODE
    \ingroup Video_group
    \brief Lists available compression modes for SVO recording.
    \brief sl::SVO_COMPRESSION_MODE_LOSSLESS is an improvement of previous lossless compression (used in ZED Explorer), even if size may be bigger, compression time is much faster.
     */
    enum SVO_COMPRESSION_MODE {
        SVO_COMPRESSION_MODE_RAW, /**< \deprecated {This compresion is deprecated this it doesn't support timestamp and IMU data} RAW images, no compression.*/
        SVO_COMPRESSION_MODE_LOSSLESS, /**< PNG/ZSTD (lossless) based compression : avg size = 42% (of RAW).*/
        SVO_COMPRESSION_MODE_LOSSY, /**< JPEG (lossy) based compression : avg size = 22% (of RAW).*/
        SVO_COMPRESSION_MODE_LAST
    };

    /**
    \ingroup Video_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SVO_COMPRESSION_MODE &svo_compression);
    ///@cond

    inline ::std::ostream &operator<<(::std::ostream &os, const SVO_COMPRESSION_MODE &svo_compression) {
        return os << toString(svo_compression);
    }
    ///@endcond

    /**
    \struct RecordingState
    \ingroup Video_group
    \brief Recording structure that contains information about SVO.
     */
    struct RecordingState {
        RecordingState() {
            status = false;
            current_compression_time = 0;
            current_compression_ratio = 0;
            average_compression_time = 0;
            average_compression_ratio = 0;
        }
        bool status; /**< status of current frame. May be true for success or false if frame could not be written in the SVO file.*/
        double current_compression_time; /**< compression time for the current frame in ms.*/
        double current_compression_ratio; /**< compression ratio (% of raw size) for the current frame.*/
        double average_compression_time; /**< average compression time in ms since beginning of recording.*/
        double average_compression_ratio; /**< compression ratio (% of raw size) since beginning of recording.*/
    };


    ///@{
    ///  @name ZED Camera Resolution
    /**
    \ingroup Video_group
    \brief Lists available video modes for the ZED camera. std::vector<std::pair(width, height)>
     */
    static const std::vector<std::pair<int, int>> cameraResolution = {
        std::make_pair(2208, 1242), /**< sl::RESOLUTION_HD2K */
        std::make_pair(1920, 1080), /**< sl::RESOLUTION_HD1080 */
        std::make_pair(1280, 720), /**< sl::RESOLUTION_HD720 */
        std::make_pair(672, 376) /**< sl::RESOLUTION_VGA */
    };
    ///@}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    /*!
    \ingroup Video_group
    \brief Converts the given RESOLUTION into a string
    \param res : a specific RESOLUTION
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ resolution2str(RESOLUTION res)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Video_group
    \brief Converts the given SELF_CALIBRATION_STATE into a string
    \param state : a specific SELF_CALIBRATION_STATE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ statusCode2str(SELF_CALIBRATION_STATE state)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Depth_group
    \brief Converts the given string into a DEPTH_MODE
    \param mode : a specific depth
    \return The corresponding DEPTH_MODE
     */
    DEPTH_MODE /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ str2mode(String mode);

    /*!
    \ingroup Depth_group
    \brief Converts the given DEPTH_MODE into a string
    \param mode : a specific DEPTH_MODE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ depthMode2str(DEPTH_MODE mode)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Depth_group
    \brief Converts the given SENSING_MODE into a string
    \param mode : a specific SENSING_MODE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ sensingMode2str(SENSING_MODE mode)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup PositionalTracking_group
    \brief Converts the given TRACKING_STATE into a string
    \param state : a specific TRACKING_STATE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ trackingState2str(TRACKING_STATE state)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup SpatialMapping_group
    \brief Converts the given SPATIAL_MAPPING_STATE into a string
    \param state : a specific SPATIAL_MAPPING_STATE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ spatialMappingState2str(SPATIAL_MAPPING_STATE state)/*@cond SHOWHIDDEN*/)/*@endcond*/;
};

#endif /*__DEFINES_HPP__*/
