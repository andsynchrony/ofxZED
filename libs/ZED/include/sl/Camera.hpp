/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 * 
 * 1. Definitions
 * 
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2 or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera, ZED 2 camera and the ZED Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2 or ZED Mini camera.
 * 
 * 2. License
 * 
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * 
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2 or ZED Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2
 *       or ZED Mini camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2 or ZED Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2 or ZED Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2 or ZED Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 * 
 * 3. Warranty
 * 
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2 or ZED Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 * 
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * 
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 * 
 * 5. Choice of Law
 * 
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <ctype.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory.h>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <map>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>


#if !defined(NDEBUG) && !defined(ALLOW_BUILD_DEBUG)
#ifdef _MSC_VER
#pragma message("WARNING : 'Debug' builds are not supported since this library was built in 'Release', 'RelWithDebInfo' should be preferred to avoid crashes and memory issues")
#else
#warning("'Debug' builds are not supported since this library was built in 'Release', 'RelWithDebInfo' should be preferred to avoid crashes and memory issues")
#endif
#endif

#define SL_DEPRECATED(str) [[deprecated(str)]]

#if defined _WIN32
#if defined CORE_COMPILATION
#define SL_CORE_EXPORT __declspec(dllexport)
#else
#define SL_CORE_EXPORT
#endif
#elif __GNUC__
#define SL_CORE_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 
#endif
//#include <Windows.h>
#define __CUSTOM__PRETTY__FUNC__ __FUNCSIG__
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#elif __GNUC__
#include <unistd.h>
#define __CUSTOM__PRETTY__FUNC__ __PRETTY_FUNCTION__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _FCT_CPU_GPU_ __host__ __device__ // for CUDA device code
#define IS_FINITE(x) isfinite(x)
#else
#define _FCT_CPU_GPU_
#define IS_FINITE(x) std::isfinite(x)
#endif

namespace sl {

    /**
    \class String
    \ingroup Core_group
    \brief Defines a string.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ String {
    public:
        String();
        ~String();

        String(const String &str);
        String(const char *data);

        void set(const char *data);
        const char *get() const;
        bool empty() const;

        String &operator=(const String &str1);
        String &operator=(const char *data);

        operator const char *() {
            return get();
        }

        const char* c_str() const;

        size_t size();
        void clear();


    private:
        char *p_data = 0;
        size_t m_size = 0;

        void clean() {
            if (m_size && p_data) free(p_data);
            m_size = 0;
            p_data = 0;
        }
    };

    /**
       \struct Resolution.
       \ingroup Core_group
       \brief Width and height of an array.
     */
    struct Resolution {
        size_t width; /**< array width in pixels  */
        size_t height; /**< array height in pixels*/

        Resolution(size_t w_ = 0, size_t h_ = 0) {
            width = w_;
            height = h_;
        }

        /**
        \brief Returns the area of the image.
        \return The number of pixels of the array.
         */
        size_t area() const {
            return width * height;
        }

        /**
        \brief Tests if the given \ref Resolution has the same properties.
        \return True if the sizes matches.
         */
        bool operator==(const Resolution &that)const {
            return ((width == that.width) && (height == that.height));
        }

        /**
        \brief Tests if the given \ref Resolution has different properties.
        \return True if the sizes are not equal.
         */
        bool operator!=(const Resolution &that)const {
            return ((width != that.width) || (height != that.height));
        }
    };

    /**
       \class Rect.
       \ingroup Core_group
       \brief Defines a 2D rectangle with top-left corner coordinates and width/height in pixels
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Rect {
    public:
        size_t x; /**< x coordinates of top-left corner.*/
        size_t y; /**< y coordinates of top-left corner.*/
        size_t width; /**< width of the rectangle in pixels.*/
        size_t height; /**< height of the rectangle in pixels.*/

        Rect(size_t x_ = 0, size_t y_ = 0, size_t w_ = 0, size_t h_ = 0) {
            x = x_;
            y = y_;
            width = w_;
            height = h_;
        }

        /**
        \brief Returns the area of the rectangle.
        \return .
         */
        size_t area() {
            return width * height;
        }

        /**
        \brief Tests if the given \ref Rect has the same properties.
        \return True if all the components matches.
         */
        bool operator==(const Rect &that)const {
            return ((x == that.x) && (y == that.y) && (width == that.width) && (height == that.height));
        }

        /**
        \brief Tests if the given \ref Rect has different properties.
        \return True if one of the components does not match.
         */
        bool operator!=(const Rect &that)const {
            return ((x != that.x) || (y != that.y) || (width != that.width) || (height != that.height));
        }

        /**
        \brief Tests if the given \ref Rect is empty (width or/and height is null)
        \return True if rectangle is empty
         */
        bool isEmpty() const {
            return (width * height == 0);
        }

        /**
        \brief Tests if this \ref Rect contains the <target> \ref Rect.
        \return Returns true if this rectangle contains the <target> rectangle. otherwise returns false.
        If proper is true, this function only returns true if the target rectangle is entirely inside this rectangle (not on the edge).
         */
        inline bool contains(const Rect &target, bool proper = false) const {
            bool ret = false;
            if (!proper)
                ret = (x <= target.x) && (y <= target.y) && (x + width >= target.x + target.width) && (y + height >= target.y + target.height);
            else
                ret = (x < target.x) && (y < target.y) && (x + width > target.x + target.width) && (y + height > target.y + target.height);
            return ret;
        }

        /**
        \brief Tests if this \ref Rect is contained inside the given <target> \ref Rect.
        \return Returns true if this rectangle is inside the current target \ref Rect. otherwise returns false.
        If proper is true, this function only returns true if this rectangle is entirely inside the <target> rectangle (not on the edge).
         */
        inline bool isContained(const Rect &target, bool proper = false) const {
            bool ret = false;
            if (!proper)
                ret = (x >= target.x) && (y >= target.y) && (x + width <= target.x + target.width) && (y + height <= target.y + target.height);
            else
                ret = (x > target.x) && (y > target.y) && (x + width < target.x + target.width) && (y + height < target.y + target.height);
            return ret;
        }

        /**
        \brief Overloaded function. Tests if this \ref Rect is contained inside a given \ref Resolution.
        It tests if the current rect is contained inside a Rect defined by Rect(O,0,resolution.width,resolution.height).
        \return Returns true if this rectangle is inside the rectangle defined by Rect(O,0,resolution.width,resolution.height). otherwise returns false.
        If proper is true, this function only returns true if this rectangle is entirely inside the rectangle defined by the resolution (not on the edge).
         */
        inline bool isContained(const Resolution &resolution, bool proper = false) const {
            bool ret = false;
            if (!proper)
                ret = (x + width <= resolution.width) && (y + height <= resolution.height);
            else
                ret = x > 0 && y > 0 && (x + width < resolution.width) && (y + height < resolution.height);
            return ret;
        }

    };

    /**
    \enum UNIT
    \ingroup Core_group
    \brief Lists available unit for measures.
     */
    enum class UNIT {
        MILLIMETER, /**< International System, 1/1000 METER. */
        CENTIMETER, /**< International System, 1/100 METER. */
        METER, /**< International System, 1 METER */
        INCH, /**< Imperial Unit, 1/12 FOOT */
        FOOT, /**< Imperial Unit, 1 FOOT */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const UNIT &unit);

    inline std::ostream &operator<<(std::ostream &os, const UNIT &unit) {
        return os << toString(unit);
    }
    ///@endcond

    /**
    \enum COORDINATE_SYSTEM
    \ingroup Core_group
    \brief Lists available coordinates systems for positional tracking and 3D measures.

    \image html CoordinateSystem.png
     */
    enum class COORDINATE_SYSTEM {
        IMAGE, /**< Standard coordinates system in computer vision. Used in OpenCV : see here : http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html */
        LEFT_HANDED_Y_UP, /**< Left-Handed with Y up and Z forward. Used in Unity with DirectX. */
        RIGHT_HANDED_Y_UP, /**< Right-Handed with Y pointing up and Z backward. Used in OpenGL. */
        RIGHT_HANDED_Z_UP, /**< Right-Handed with Z pointing up and Y forward. Used in 3DSMax. */
        LEFT_HANDED_Z_UP, /**< Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine. */
        RIGHT_HANDED_Z_UP_X_FWD, /**< Right-Handed with Z pointing up and X forward. Used in ROS (REP 103). */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const COORDINATE_SYSTEM &coord_system);

    inline std::ostream &operator<<(std::ostream &os, const COORDINATE_SYSTEM &coord_system) {
        return os << toString(coord_system);
    }
    ///@endcond

    /**
    \enum ERROR_CODE
    \ingroup Core_group
    \brief Lists error codes in the ZED SDK.
     */
    enum class ERROR_CODE {
        SUCCESS, /**< Standard code for successful behavior.*/
        FAILURE, /**< Standard code for unsuccessful behavior.*/
        NO_GPU_COMPATIBLE, /**< No GPU found or CUDA capability of the device is not supported.*/
        NOT_ENOUGH_GPU_MEMORY, /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).*/
        CAMERA_NOT_DETECTED, /**< The ZED camera is not plugged or detected.*/
        SENSORS_NOT_AVAILABLE, /**< a ZED-M or ZED2 camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2 devices*/
        INVALID_RESOLUTION, /**< In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage */
        LOW_USB_BANDWIDTH, /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
        CALIBRATION_FILE_NOT_AVAILABLE, /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
        INVALID_CALIBRATION_FILE, /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
        INVALID_SVO_FILE, /**< The provided SVO file is not valid.*/
        SVO_RECORDING_ERROR, /**< An recorder related error occurred (not enough free storage, invalid file).*/
        SVO_UNSUPPORTED_COMPRESSION, /**< An SVO related error when NVIDIA based compression cannot be loaded.*/
        END_OF_SVOFILE_REACHED, /**<SVO end of file has been reached, and no frame will be available until the SVO position is reset.*/
        INVALID_COORDINATE_SYSTEM, /**< The requested coordinate system is not available.*/
        INVALID_FIRMWARE, /**< The firmware of the ZED is out of date. Update to the latest version.*/
        INVALID_FUNCTION_PARAMETERS, /**< An invalid parameter has been set for the function. */
        CUDA_ERROR, /**< In grab() or retrieveXXX() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.*/
        CAMERA_NOT_INITIALIZED, /**< In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.*/
        NVIDIA_DRIVER_OUT_OF_DATE, /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
        INVALID_FUNCTION_CALL, /**< The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open. */
        CORRUPTED_SDK_INSTALLATION, /**< The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched. */
        INCOMPATIBLE_SDK_VERSION, /**< The installed SDK is incompatible SDK used to compile the program. */
        INVALID_AREA_FILE, /**< The given area file does not exist, check the path. */
        INCOMPATIBLE_AREA_FILE, /**< The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set. */
        CAMERA_FAILED_TO_SETUP, /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
        CAMERA_DETECTION_ISSUE, /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
        CANNOT_START_CAMERA_STREAM, /**< Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.*/
        NO_GPU_DETECTED, /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
        PLANE_NOT_FOUND, /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
        MODULE_NOT_COMPATIBLE_WITH_CAMERA, /**< The Object detection module is only compatible with the ZED 2*/
        MOTION_SENSORS_REQUIRED, /**< The module needs the sensors to be enabled (see InitParameters::disable_sensors) */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const ERROR_CODE &errorCode);
    ///@endcond

    /**
    \ingroup Video_group
    \brief Return a text explaining how to fix the given ERROR_CODE.
    \return A string of advice for the user.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toVerbose(const ERROR_CODE &errorCode);


    ///@cond SHOWHIDDEN

    inline std::ostream &operator<<(std::ostream &os, const ERROR_CODE &errorCode) {
        return os << toString(errorCode);
    }
    ///@endcond

    /**
    \enum MODEL
    \ingroup Video_group
    \brief Lists compatible ZED Camera model
     */
    enum class MODEL {
        ZED, /**< Defines ZED Camera model */
        ZED_M, /**<  Defines ZED Mini (ZED-M) Camera model */
        ZED2, /**< Defines ZED 2 Camera model */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const MODEL &model);

    inline std::ostream &operator<<(std::ostream &os, const MODEL &model) {
        return os << toString(model);
    }
    ///@endcond

    /**
    \enum INPUT_TYPE
    \ingroup Video_group
    \brief Lists available input type in SDK
     */
    enum class INPUT_TYPE {
        USB, /**< USB input mode  */
        SVO, /**<  SVO file input mode */
        STREAM, /**< STREAM input mode (requires to use enableStreaming()/disableStreaming() on the "sender" side) */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const INPUT_TYPE &input_type);

    inline std::ostream &operator<<(std::ostream &os, const INPUT_TYPE &input_type) {
        return os << toString(input_type);
    }
    ///@endcond

    /**
    \enum CAMERA_STATE
    \ingroup Video_group
    \brief List of possible camera state
     */
    enum class CAMERA_STATE {
        AVAILABLE, /**< Defines if the camera can be opened by the SDK */
        NOT_AVAILABLE, /**<  Defines if the camera is already opened and unavailable*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const CAMERA_STATE &camera_state);

    inline std::ostream &operator<<(std::ostream &os, const CAMERA_STATE &camera_state) {
        return os << toString(camera_state);
    }
    ///@endcond

    /**
   \struct DeviceProperties
   \ingroup Video_group
   \brief Properties of a camera

   \note A camera_model MODEL::ZED_M with an id '-1' can be due to an inverted USB-C cable.

   \warning Experimental on Windows.
     */
    struct DeviceProperties {
        /**
        the camera state
         */
        sl::CAMERA_STATE camera_state = sl::CAMERA_STATE::NOT_AVAILABLE;

        /**
        the camera id (Notice that only the camera with id '0' can be used on Windows)
         */
        int id = -1;

        /**
        the camera system path
         */
        sl::String path;

        /**
        the camera model
         */
        sl::MODEL camera_model = sl::MODEL::LAST;

        /**
        the camera serial number
        \n Not provided for Windows
         */
        unsigned int serial_number = 0;
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const DeviceProperties &properties);

    inline std::ostream &operator<<(std::ostream &os, const DeviceProperties &properties) {
        return os << toString(properties);
    }
    ///@endcond

    /**
    \enum STREAMING_CODEC
    \ingroup Video_group
    \brief List of possible camera state
     */
    enum class STREAMING_CODEC {
        H264, /**< AVCHD/H264 encoding used in image streaming.*/
        H265, /**<  HEVC/H265 encoding used in image streaming.*/
        LAST
    };

    /**
   \struct StreamingProperties
   \ingroup Video_group
   \brief Properties of a streaming device
     */
    struct StreamingProperties {
        /**
        the streaming IP of the device
         */
        sl::String ip = "";

        /**
        the streaming port
         */
        unsigned short port = 0;

        /**
        the serial number of the streaming device
         */
        unsigned int serial_number = 0;

        /**
        the current bitrate of encoding of the streaming device
         */
        int current_bitrate = 0;

        /**
        the current codec used for compression in streaming device
         */
        STREAMING_CODEC codec = STREAMING_CODEC::H265;
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const StreamingProperties &properties);

    inline std::ostream &operator<<(std::ostream &os, const StreamingProperties &properties) {
        return os << toString(properties);
    }
    ///@endcond

    /**
    \enum DEVICE_SENSORS
    \ingroup Sensors_group
    \brief List of the available onboard sensors
     */
    enum class SENSOR_TYPE {
        ACCELEROMETER, /**< Three axis Accelerometer sensor to measure the inertial accelerations. */
        GYROSCOPE, /**< Three axis Gyroscope sensor to measure the angular velocitiers. */
        MAGNETOMETER, /**< Three axis Magnetometer sensor to measure the orientation of the device respect to the earth magnetic field. */
        BAROMETER, /**< Barometer sensor to measure the atmospheric pressure. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const SENSOR_TYPE &sens);

    inline std::ostream &operator<<(std::ostream &os, const SENSOR_TYPE &sens) {
        return os << toString(sens);
    }
    ///@endcond

    /**
    \enum SENSORS_UNIT
    \ingroup Sensors_group
    \brief List of the available onboard sensors measurement units
     */
    enum class SENSORS_UNIT {
        M_SEC_2, /**< Acceleration [m/s²]. */
        DEG_SEC, /**< Angular velocity [deg/s]. */
        U_T, /**< MAgnetic Fiels [uT]. */
        HPA, /**< Atmospheric pressure [hPa]. */
        CELSIUS, /**< Temperature [°C]. */
        HERTZ, /**< Frequency [Hz]. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const SENSORS_UNIT &unit);

    inline std::ostream &operator<<(std::ostream &os, const SENSORS_UNIT &unit) {
        return os << toString(unit);
    }
    ///@endcond

    /**
    \class InputType
    \ingroup Video_group
    \brief Defines the input type used in the ZED SDK. Can be used to select a specific camera with ID or serial number, or a svo file
     */
    class SL_CORE_EXPORT InputType {
        friend class Camera;
        friend struct InitParameters;
    public:

        /**
        Default constructor
         */
        InputType();

        /**
        Copy constructor
         */
        InputType(const InputType &type);

        /**
        Set the input as the camera with specified id
         */
        void setFromCameraID(unsigned int id);

        /**
        Set the input as the camera with specified serial number
         */
        void setFromSerialNumber(unsigned int serial_number);

        /**
        Set the input as the svo specified with the filename
         */
        void setFromSVOFile(sl::String svo_input_filename);

        /**
        Set the input as the stream defined by the IP and the port of the sending device
        \note the following notes are only available for v2.8.5 and up. Previously, the internal port used to receive stream was randomly choosen between a list of available ports.
        \note Since 2.8.5: Internally, the receiving part will try to use
        the same port number. If the port number is already taken by a process, the session will retry on port + 2, +4, +6, .....
        Therefore the internal port used to receive the stream will be the closest upper port available.
        \note the protocol used for the streaming module is based on RTP/RTCP. Port must be even number, since the port+1 is used for control data.
         */
        void setFromStream(sl::String senderIP, unsigned short port = 30000);

    private:

        enum class INPUT_TYPE {
            ID,
            SERIAL,
            SVO_FILE,
            STREAM,
            LAST
        };

        INPUT_TYPE input_type = INPUT_TYPE::ID;
        unsigned int serial_number = 0;
        unsigned int id = 0;
        sl::String svo_input_filename = "";
        sl::String stream_input_ip = "";
        unsigned short stream_input_port = 0;
    };

    /**
    \enum DETECTION_MODEL
    \ingroup Object_group
    \brief List available models for detection
     */
    enum class DETECTION_MODEL {
        MULTI_CLASS_BOX, /**< Any objects, bounding box based */
        HUMAN_BODY_FAST, /**<  Keypoints based, specific to human skeleton, real time performance even on Jetson or low end GPU cards */
        HUMAN_BODY_ACCURATE, /**<  Keypoints based, specific to human skeleton, state of the art accuracy, requires powerful GPU */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const DETECTION_MODEL& input_type);

    inline std::ostream& operator<<(std::ostream& os, const DETECTION_MODEL& input_type) {
        return os << toString(input_type);
    }
    ///@endcond

    ///@cond SHOWHIDDEN
    /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use toString")/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ model2str(MODEL model)/*@cond SHOWHIDDEN*/ /*@endcond*/;
    /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use toString")/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ errorCode2str(ERROR_CODE errorCode)/*@cond SHOWHIDDEN*/ /*@endcond*/;
    /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use toString")/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ unit2str(UNIT unit)/*@cond SHOWHIDDEN*/ /*@endcond*/;
    UNIT /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ str2unit(String unit);
    ///@endcond

    /*!
    \ingroup Core_group
    \brief Tells the program to wait for x ms.
    \param time : the number of ms to wait.
     */
    inline void sleep_ms(int time) {
        std::this_thread::sleep_for(std::chrono::milliseconds(time));
    }

    /*!
    \ingroup Core_group
    \brief Tells the program to wait for x us.
    \param time : the number of us to wait.
     */
    inline void sleep_us(int time) {
        std::this_thread::sleep_for(std::chrono::microseconds(time));
    }


    /// @cond
    template <typename T>
    class Vector3;
    template <typename T>
    class Vector4;
    ///@endcond

    /**
    \class Matrix3f
    \ingroup Core_group
    \brief Represents a generic 3*3 matrix

    It is defined in a row-major order, it means that, in the value buffer, the entire first row is stored first, followed by the entire second row, and so on.
    You can access the data with the 'r' ptr or by element attribute.
     * | | | |
     * |-|-|-|
     * | r00 | r01 | r02 |
     * | r10 | r11 | r12 |
     * | r20 | r21 | r22 |
     * 
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Matrix3f {
    public:
        /// @cond
        static const int nbElem = 9;

        union {
            // access inner data by dedicated ref.

            struct {
                float r00, r01, r02, r10, r11, r12, r20, r21, r22;
            };
            // ptr to inner data
            float r[nbElem];
        };
        /// @endcond

        /**
        \brief Matrix3f default constructor.
         */
        Matrix3f();

        /**
        \brief Matrix3f copy constructor (deep copy).
         */
        Matrix3f(float data[]);

        /**
        \brief Matrix3f copy constructor (deep copy).
        \param mat : the Matrix3f to copy.
         */
        Matrix3f(const Matrix3f &mat);

        /**
        \brief Gives the result of the multiplication between two Matrix3f
         */
        Matrix3f operator*(const Matrix3f &mat) const;

        /**
        \brief Gives the result of the multiplication between a Matrix3f and a scalar.
         */
        Matrix3f operator*(const float &scalar) const;

        /**
        \brief Gives the result of the addition between two Matrix3f
         */
        Matrix3f operator+(const Matrix3f &mat) const;

        /**
        \brief Gives the result of the addition between a Matrix3f and a scalar.
         */
        Matrix3f operator+(const float &scalar) const;

        /**
        \brief Gives the result of the subtraction between two Matrix3f
         */
        Matrix3f operator-(const Matrix3f &mat) const;

        /**
        \brief Gives the result of the subtraction between a Matrix3f and a scalar.
         */
        Matrix3f operator-(const float &scalar) const;

        /**
        \brief Test two Matrix3f equality.
         */
        bool operator==(const Matrix3f &mat) const;

        /**
        \brief Test two Matrix3f inequality.
         */
        bool operator!=(const Matrix3f &mat) const;

        /**
        \brief Gets access to a specific point in the Matrix3f (read / write).
        \param u : specify the row
        \param v : specify the column
        \return The value at the u, v coordinates.
         */
        float &operator()(int u, int v);

        /**
        \brief Sets the Matrix3f to its inverse.
         */
        void inverse();

        /**
        \brief Returns the inverse of a Matrix3f.
        \param rotation : the Matrix3f to compute the inverse from.
        \return The inverse of the given Matrix3f
         */
        static Matrix3f inverse(const Matrix3f &rotation);

        /**
        \brief Sets the RotationArray to its transpose.
         */
        void transpose();

        /**
        \brief Returns the transpose of a Matrix3f.
        \param rotation : the Matrix3f to compute the transpose from.
        \return The transpose of the given Matrix3f
         */
        static Matrix3f transpose(const Matrix3f &rotation);

        /**
        \brief Sets the Matrix3f to identity.
         */
        void setIdentity();

        /**
        \brief Creates an identity Matrix3f.
        \return A Matrix3f set to identity.
         */
        static Matrix3f identity();

        /**
        \brief Sets the Matrix3f to zero.
         */
        void setZeros();

        /**
        \brief Creates a Matrix3f filled with zeros.
        \return A Matrix3f set to zero.
         */
        static Matrix3f zeros();

        /**
        \brief Returns the components of the Matrix3f in a \ref String.
        \return A \ref String containing the components of the current Matrix3f.
         */
        String getInfos();

        /**
        \brief Name of the matrix (optional).
         */
        String matrix_name;
    };

    /**
    \class Matrix4f
    \ingroup Core_group
    \brief Represents a generic 4*4 matrix.

    It is defined in a row-major order, it means that, in the value buffer, the entire first row is stored first, followed by the entire second row, and so on.
    You can access the data by the 'm' ptr or by the element attribute.
     * 
     * | | | | |
     * |-|-|-|-|
     * | r00 | r01 | r02 | tx |
     * | r10 | r11 | r12 | ty |
     * | r20 | r21 | r22 | tz |
     * | m30 | m31 | m32 | m33 |
     * 
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Matrix4f {
    public:
        /// @cond
        static const int nbElem = 16;

        union {

            struct {
                float r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, m30, m31, m32, m33;
            };
            // ptr to inner data.
            float m[nbElem];
        };
        /// @endcond

        /**
        \brief Matrix4f default constructor.
         */
        Matrix4f();

        /**
        \brief Matrix4f copy constructor (deep copy).
         */
        Matrix4f(float data[]);

        /**
        \brief Matrix4f copy constructor (deep copy).
        \param mat : the Matrix4f to copy.
         */
        Matrix4f(const Matrix4f &mat);

        /**
        brief Gives the result of the multiplication between two Matrix4f.
         */
        Matrix4f operator*(const Matrix4f &mat) const;

        /**
        brief Gives the result of the multiplication between a Matrix4f and a float4.
         */
        Matrix4f operator*(const Vector4<float> &vect) const;

        /**
        brief Gives the result of the multiplication between a Matrix4f and a scalar.
         */
        Matrix4f operator*(const float &scalar) const;

        /**
        brief Gives the result of the addition between two Matrix4f.
         */
        Matrix4f operator+(const Matrix4f &mat) const;

        /**
        brief Gives the result of the addition between a Matrix4f and a scalar.
         */
        Matrix4f operator+(const float &scalar) const;

        /**
        brief Gives the result of the subtraction between two Matrix4f.
         */
        Matrix4f operator-(const Matrix4f &mat) const;

        /**
        brief Gives the result of the subtraction between a Matrix4f and a scalar.
         */
        Matrix4f operator-(const float &scalar) const;

        /**
        brief Tests two Matrix4f equality.
         */
        bool operator==(const Matrix4f &mat) const;

        /**
        brief Tests two Matrix4f inequality.
         */
        bool operator!=(const Matrix4f &mat) const;

        /**
        \brief Gets access to a specific point in the Matrix4f (read / write).
        \param u : specify the row.
        \param v : specify the column.
        \return The value at  the u, v coordinates.
         */
        float &operator()(int u, int v);

        /**
        \brief Sets the Matrix4f to its inverse.
        \return SUCCESS if the inverse has been computed, ERROR_CODE.FAILURE is not (det = 0).
         */
        ERROR_CODE inverse();

        /**
        \brief Creates the inverse of a Matrix4f.
        \param mat : the Matrix4f to compute the inverse from.
        \return The inverse of the given Matrix4f.
         */
        static Matrix4f inverse(const Matrix4f &mat);

        /**
        \brief Sets the Matrix4f to its transpose.
         */
        void transpose();

        /**
        \brief Creates the transpose of a Matrix4f.
        \param mat : the Matrix4f to compute the transpose from.
        \return The transpose of the given Matrix4f.
         */
        static Matrix4f transpose(const Matrix4f &mat);

        /**
        \brief Sets the Matrix4f to identity.
         */
        void setIdentity();

        /**
        \brief Creates an identity Matrix4f.
        \return A Matrix4f set to identity.
         */
        static Matrix4f identity();

        /**
        \brief Sets the Matrix4f to zero.
         */
        void setZeros();

        /**
        \brief Creates a Matrix4f filled with zeros.
        \return A Matrix4f set to zero.
         */
        static Matrix4f zeros();

        /**
        \brief Sets a 3x3 Matrix inside the Matrix4f.
        \note Can be used to set the rotation matrix when the matrix4f is a pose or an isometric matrix.
        \param input  : sub matrix to put inside the Matrix4f.
        \param row : index of the row to start the 3x3 block. Must be 0 or 1.
        \param column : index of the column to start the 3x3 block. Must be 0 or 1.
        \return SUCCESS if everything went well, ERROR_CODE_FAILURE otherwise.
         */
        ERROR_CODE setSubMatrix3f(Matrix3f input, int row = 0, int column = 0);

        /**
        \brief Sets a 3x1 Vector inside the Matrix4f at the specified column index.
        \note Can be used to set the Translation/Position matrix when the matrix4f is a pose or an isometry.
        \param input  : sub vector to put inside the Matrix4f.
        \param column : index of the column to start the 3x3 block. By default, it is the last column (translation for a Pose).
        \return SUCCESS if everything went well, ERROR_CODE_FAILURE otherwise.
         */
        ERROR_CODE setSubVector3f(Vector3<float> input, int column = 3);

        /**
        \brief Sets a 4x1 Vector inside the Matrix4f at the specified column index.
        \note Can be used to set the Translation/Position matrix when the matrix4f is a pose or an isometry.
        \param input  : sub vector to put inside the Matrix4f.
        \param column : index of the column to start the 3x3 block. By default, it is the last column (translation for a Pose).
        \return SUCCESS if everything went well, ERROR_CODE_FAILURE otherwise.
         */
        ERROR_CODE setSubVector4f(Vector4<float> input, int column = 3);

        /**
        \brief Returns the components of the Matrix4f in a \ref String.
        \return A \ref String containing the components of the current Matrix4f.
         */
        String getInfos();

        /**
        \brief Name of the matrix (optional).
         */
        String matrix_name;
    };

    /**
    \class Vector2
    \ingroup Core_group
    \brief Represents a two dimensions vector for both CPU and GPU.
     */
    template <typename T>
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Vector2 {
        static const int nbElem = 2;
    public:
        /// @cond

        union {

            struct {
                T x, y;
            };
            T v[nbElem];
        };
        /// @endcond

        inline _FCT_CPU_GPU_ int size() const {
            return nbElem;
        }

        _FCT_CPU_GPU_ Vector2() {
        }

        _FCT_CPU_GPU_ Vector2(const T &t) {
            this->x = t;
            this->y = t;
        }

        _FCT_CPU_GPU_ Vector2(const T *tp) {
            this->x = tp[0];
            this->y = tp[1];
        }

        _FCT_CPU_GPU_ Vector2(const T v0, const T v1) {
            this->x = v0;
            this->y = v1;
        }

        _FCT_CPU_GPU_ Vector2<T>(const Vector2<T> &v) {
            this->x = v.x;
            this->y = v.y;
        }

        _FCT_CPU_GPU_ const T *ptr() const {
            return &this->v[0];
        }

        inline _FCT_CPU_GPU_ Vector2 &setValues(const T *b) {
            this->x = b[0];
            this->y = b[1];
            return *this;
        }

        _FCT_CPU_GPU_ T &operator[](int i) {
            return this->v[i];
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator*=(Vector2<T>&itself, T d) {
            itself.x *= d;
            itself.y *= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator*=(Vector2<T>&itself, const Vector2<T> &b) {
            itself.x *= b.x;
            itself.y *= b.y;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator/=(Vector2<T>&itself, T d) {
            if (d == 0) return itself;
            itself.x /= d;
            itself.y /= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator/=(Vector2<T>&itself, const Vector2<T> &b) {
            itself.x /= b.x;
            itself.y /= b.y;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator+=(Vector2<T>&itself, const Vector2<T> &b) {
            itself.x += b.x;
            itself.y += b.y;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> &operator-=(Vector2<T>&itself, const Vector2<T> &b) {
            itself.x -= b.x;
            itself.y -= b.y;
            return itself;
        }

        /**
        \brief returns the norm of the vector
         */
        inline _FCT_CPU_GPU_ float norm() {
            return sqrt(this->x * this->x + this->y * this->y);
        }

        /**
        \brief returns the squared norm of the vector
         */
        inline _FCT_CPU_GPU_ float square() {
            return (this->x * this->x + this->y * this->y);
        }

        /**
        \brief returns the sum of the vector
         */
        inline _FCT_CPU_GPU_ float sum() {
            return (this->x + this->y);
        }

        /**
        \brief returns the dot product of two vector
         */
        inline _FCT_CPU_GPU_ float dot(const Vector2<T> &a, const Vector2<T> &b) {
            return (a.x * b.x + a.y * b.y);
        }

        /**
        \brief returns the distance between two vector
         */
        inline _FCT_CPU_GPU_ float distance(const Vector2<T> &a, const Vector2<T> &b) {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator+(const Vector2<T> &a, const Vector2<T> &b) {
            Vector2<T> tmp(a);
            return tmp += b;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator-(const Vector2<T> &a, const Vector2<T> &b) {
            Vector2<T> tmp(a);
            return tmp -= b;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator*(const Vector2<T> &a, T b) {
            Vector2<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator*(const Vector2<T> &a, const Vector2<T> &b) {
            Vector2<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator/(const Vector2<T> &a, T b) {
            Vector2<T> tmp(a);
            return tmp /= b;
        }

        inline _FCT_CPU_GPU_ friend Vector2<T> operator/(const Vector2<T> &a, const Vector2<T> &b) {
            Vector2<T> tmp(a);
            return tmp /= b;
        }
    };

    /**
    \class Vector3
    \ingroup Core_group
    \brief Represents a three dimensions vector for both CPU and GPU.
     */
    template <typename T>
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Vector3 {
        static const int nbElem = 3;
    public:
        /// @cond

        union {

            struct {
                T x, y, z;
            };

            struct {
                T r, g, b;
            };

            struct {
                T tx, ty, tz;
            };
            T v[nbElem];
        };
        /// @endcond

        inline _FCT_CPU_GPU_ int size() const {
            return nbElem;
        }

        _FCT_CPU_GPU_ Vector3() {
        }

        _FCT_CPU_GPU_ Vector3(const T &t) {
            this->x = t;
            this->y = t;
            this->z = t;
        }

        _FCT_CPU_GPU_ Vector3(const T *tp) {
            this->x = tp[0];
            this->y = tp[1];
            this->z = tp[2];
        }

        _FCT_CPU_GPU_ Vector3(const T v0, const T v1, const T v2) {
            this->x = v0;
            this->y = v1;
            this->z = v2;
        }

        _FCT_CPU_GPU_ Vector3<T>(const Vector3<T> &v) {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
        }

        _FCT_CPU_GPU_ Vector3<T>(const Vector2<T> &v, const T d = 0) {
            this->x = v.x;
            this->y = v.y;
            this->z = d;
        }

        _FCT_CPU_GPU_ Vector3<T>(const Vector4<T>& v) {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
        }

        _FCT_CPU_GPU_ const T *ptr() const {
            return &this->v[0];
        }

        inline _FCT_CPU_GPU_ Vector3<T> &setValues(const T *b) {
            this->x = b[0];
            this->y = b[1];
            this->z = b[2];
            return *this;
        }

        inline _FCT_CPU_GPU_ Vector3<T>& operator=(const Vector4<T>& other) {
            this->x = other.x;
            this->y = other.y;
            this->z = other.z;
            return *this;
        }

        _FCT_CPU_GPU_ T &operator[](int i) {
            return this->v[i];
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator+=(Vector3<T>&itself, T d) {
            itself.x += d;
            itself.y += d;
            itself.z += d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator+=(Vector3<T>&itself, const Vector3<T> &b) {
            itself.x += b.x;
            itself.y += b.y;
            itself.z += b.z;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator-=(Vector3<T>&itself, T d) {
            itself.x -= d;
            itself.y -= d;
            itself.z -= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator-=(Vector3<T>&itself, const Vector3<T> &b) {
            itself.x -= b.x;
            itself.y -= b.y;
            itself.z -= b.z;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator*=(Vector3<T>&itself, T d) {
            itself.x *= d;
            itself.y *= d;
            itself.z *= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator*=(Vector3<T>&itself, const Vector3<T> &b) {
            itself.x *= b.x;
            itself.y *= b.y;
            itself.z *= b.z;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator*=(Vector3<T> &itself, const Matrix3f &b) {
            Vector3<T> tmp(itself);
            itself.v[0] = tmp.v[0] * b.r[0] + tmp.v[1] * b.r[1] + tmp.v[2] * b.r[2];
            itself.v[1] = tmp.v[0] * b.r[3] + tmp.v[1] * b.r[4] + tmp.v[2] * b.r[5];
            itself.v[2] = tmp.v[0] * b.r[6] + tmp.v[1] * b.r[7] + tmp.v[2] * b.r[8];
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator/=(Vector3<T>&itself, T d) {
            if (d == 0) return itself;
            itself.x /= d;
            itself.y /= d;
            itself.z /= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> &operator/=(Vector3<T>&itself, const Vector3<T> &b) {
            if (b.x != 0) itself.x /= b.x;
            if (b.y != 0) itself.y /= b.y;
            if (b.z != 0) itself.z /= b.z;
            return itself;
        }

        /**
        \brief returns the norm of the vector
         */
        inline _FCT_CPU_GPU_ float norm() {
            return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
        }

        /**
        \brief returns the squared norm of the vector
         */
        inline _FCT_CPU_GPU_ float square() {
            return (this->x * this->x + this->y * this->y + this->z * this->z);
        }

        /**
        \brief returns the sum of the vector
         */
        inline _FCT_CPU_GPU_ float sum() {
            return (this->x + this->y + this->z);
        }

        /**
        \brief returns the dot product of two vector
         */
        static inline _FCT_CPU_GPU_ float dot(const Vector3<T> &a, const Vector3<T> &b) {
            return (a.x * b.x + a.y * b.y + a.z * b.z);
        }

        /**
        \brief returns the distance between two vector
         */
        static inline _FCT_CPU_GPU_ float distance(const Vector3<T> &a, const Vector3<T> &b) {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
        }

        /**
        \brief returns the cross product between two vector
         */
        static inline _FCT_CPU_GPU_ Vector3<T> cross(const Vector3<T> &a, const Vector3<T> &b) {
            Vector3<T> r;
            r.x = a.y * b.z - a.z * b.y;
            r.y = a.z * b.x - a.x * b.z;
            r.z = a.x * b.y - a.y * b.x;
            return r;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator+(const Vector3<T> &a, const Vector3<T> &b) {
            Vector3<T> tmp(a);
            return tmp += b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator-(const Vector3<T> &a, const Vector3<T> &b) {
            Vector3<T> tmp(a);
            return tmp -= b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator*(const Vector3<T> &a, T b) {
            Vector3<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator*(T a, const Vector3<T> &b) {
            Vector3<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator*(const Vector3<T> &a, const Vector3<T> &b) {
            Vector3<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator*(const Vector3<T> &a, const Matrix3f &b) {
            Vector3<T> tmp;
            tmp.v[0] = a.v[0] * b.r[0] + a.v[1] * b.r[1] + a.v[2] * b.r[2];
            tmp.v[1] = a.v[0] * b.r[3] + a.v[1] * b.r[4] + a.v[2] * b.r[5];
            tmp.v[2] = a.v[0] * b.r[6] + a.v[1] * b.r[7] + a.v[2] * b.r[8];
            return tmp;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator/(const Vector3<T> &a, T b) {
            Vector3<T> tmp(a);
            return tmp /= b;
        }

        inline _FCT_CPU_GPU_ friend Vector3<T> operator/(const Vector3<T> &a, const Vector3<T> &b) {
            Vector3<T> tmp(a);
            return tmp /= b;
        }
    };

    /**
    \class Vector4
    \ingroup Core_group
    \brief Represents a four dimensions vector for both CPU and GPU.
     */
    template <typename T>
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Vector4 {
        static const int nbElem = 4;
    public:
        /// @cond

        union {

            struct {
                T x, y, z, w;
            };

            struct {
                T r, g, b, a;
            };

            struct {
                T ox, oy, oz, ow;
            };
            T v[nbElem];
        };
        /// @endcond

        inline _FCT_CPU_GPU_ int size() const {
            return nbElem;
        }

        _FCT_CPU_GPU_ Vector4() {
        }

        _FCT_CPU_GPU_ Vector4(const T &t) {
            this->x = t;
            this->y = t;
            this->z = t;
            this->w = t;
        }

        _FCT_CPU_GPU_ Vector4(const T *tp) {
            this->x = tp[0];
            this->y = tp[1];
            this->z = tp[2];
            this->w = tp[3];
        }

        _FCT_CPU_GPU_ Vector4(const T v0, const T v1, const T v2, const T v3) {
            this->x = v0;
            this->y = v1;
            this->z = v2;
            this->w = v3;
        }

        _FCT_CPU_GPU_ Vector4<T>(const Vector4<T> &v) {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
            this->w = v.w;
        }

        _FCT_CPU_GPU_ Vector4<T>(const Vector4<T> &v, const T d) {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
            this->w = d;
        }

        _FCT_CPU_GPU_ Vector4<T>(const Vector3<T>& v, const T d = 0) {
            this->x = v.x;
            this->y = v.y;
            this->z = v.z;
            this->w = d;
        }

        _FCT_CPU_GPU_ const T *ptr() const {
            return &this->v[0];
        }

        inline _FCT_CPU_GPU_ Vector4<T> &setValues(const T *b) {
            this->x = b[0];
            this->y = b[1];
            this->z = b[2];
            this->w = b[3];
            return *this;
        }

        inline _FCT_CPU_GPU_ Vector4<T>& operator=(const Vector3<T>& other) {
            this->x = other.x;
            this->y = other.y;
            this->z = other.z;
            return *this;
        }

        _FCT_CPU_GPU_ T &operator[](int i) {
            return this->v[i];
        }

        _FCT_CPU_GPU_ const T &operator[](int i) const {
            return this->v[i];
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator+=(Vector4<T>&itself, T d) {
            itself.x += d;
            itself.y += d;
            itself.z += d;
            itself.w += d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator+=(Vector4<T>&itself, const Vector4<T> &b) {
            itself.x += b.x;
            itself.y += b.y;
            itself.z += b.z;
            itself.w += b.w;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator-=(Vector4<T>&itself, T d) {
            itself.x -= d;
            itself.y -= d;
            itself.z -= d;
            itself.w -= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator-=(Vector4<T>&itself, const Vector4<T> &b) {
            itself.x -= b.x;
            itself.y -= b.y;
            itself.z -= b.z;
            itself.w -= b.w;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator*=(Vector4<T>&itself, T d) {
            itself.x *= d;
            itself.y *= d;
            itself.z *= d;
            itself.w *= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator*=(Vector4<T>&itself, const Vector4<T> &b) {
            itself.x *= b.x;
            itself.y *= b.y;
            itself.z *= b.z;
            itself.w *= b.w;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator*=(Vector4<T> &itself, const Matrix4f &b) {
            Vector4<T> tmp(itself);
            itself.v[0] = tmp.v[0] * b.m[0] + tmp.v[1] * b.m[1] + tmp.v[2] * b.m[2] + tmp.v[3] * b.m[3];
            itself.v[1] = tmp.v[0] * b.m[4] + tmp.v[1] * b.m[5] + tmp.v[2] * b.m[6] + tmp.v[3] * b.m[7];
            itself.v[2] = tmp.v[0] * b.m[8] + tmp.v[1] * b.m[9] + tmp.v[2] * b.m[10] + tmp.v[3] * b.m[11];
            itself.v[3] = tmp.v[0] * b.m[12] + tmp.v[1] * b.m[13] + tmp.v[2] * b.m[14] + tmp.v[3] * b.m[15];
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator/=(Vector4<T>&itself, T d) {
            if (d == 0) return itself;
            itself.x /= d;
            itself.y /= d;
            itself.z /= d;
            itself.w /= d;
            return itself;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> &operator/=(Vector4<T>&itself, const Vector4<T> &b) {
            if (b.x != 0) itself.x /= b.x;
            if (b.y != 0) itself.y /= b.y;
            if (b.z != 0) itself.z /= b.z;
            if (b.w != 0) itself.w /= b.w;
            return itself;
        }

        /**
        \brief returns the norm of the vector
         */
        inline _FCT_CPU_GPU_ float norm() {
            return sqrt(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
        }

        /**
        \brief returns the squared norm of the vector
         */
        inline _FCT_CPU_GPU_ float square() {
            return (this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
        }

        /**
        \brief returns the sum of the vector
         */
        inline _FCT_CPU_GPU_ float sum() {
            return (this->x + this->y + this->z + this->w);
        }

        /**
        \brief returns the dot product of two vector
         */
        static inline _FCT_CPU_GPU_ float dot(const Vector4<T> &a, const Vector4<T> &b) {
            return (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        }

        /**
        \brief returns the distance between two vector
         */
        static inline _FCT_CPU_GPU_ float distance(const Vector4<T> &a, const Vector4<T> &b) {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2) + pow(a.w - b.w, 2));
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator-(const Vector4<T> &b) {
            Vector4<T> tmp;
            tmp.x = -b.x;
            tmp.y = -b.y;
            tmp.z = -b.z;
            tmp.w = -b.w;
            return tmp;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator+(const Vector4<T> &a, const Vector4<T> &b) {
            Vector4<T> tmp(a);
            return tmp += b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator-(const Vector4<T> &a, const Vector4<T> &b) {
            Vector4<T> tmp(a);
            return tmp -= b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator*(const Vector4<T> &a, T b) {
            Vector4<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator*(T a, const Vector4<T> &b) {
            Vector4<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator*(const Vector4<T> &a, const Vector4<T> &b) {
            Vector4<T> tmp(a);
            return tmp *= b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator*(const Vector4<T> &a, const Matrix4f &b) {
            Vector4<T> tmp;
            tmp.v[0] = a.v[0] * b.m[0] + a.v[1] * b.m[1] + a.v[2] * b.m[2] + a.v[3] * b.m[3];
            tmp.v[1] = a.v[0] * b.m[4] + a.v[1] * b.m[5] + a.v[2] * b.m[6] + a.v[3] * b.m[7];
            tmp.v[2] = a.v[0] * b.m[8] + a.v[1] * b.m[9] + a.v[2] * b.m[10] + a.v[3] * b.m[11];
            tmp.v[3] = a.v[0] * b.m[12] + a.v[1] * b.m[13] + a.v[2] * b.m[14] + a.v[3] * b.m[15];
            return tmp;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator/(const Vector4<T> &a, T b) {
            Vector4<T> tmp(a);
            return tmp /= b;
        }

        inline _FCT_CPU_GPU_ friend Vector4<T> operator/(const Vector4<T> &a, const Vector4<T> &b) {
            Vector4<T> tmp(a);
            return tmp /= b;
        }
    };

    ///@{
    ///  @name Types definition

#ifndef float1
    typedef float float1;
#endif

#ifndef float2
    typedef Vector2<float> float2;
#endif

#ifndef float3
    typedef Vector3<float> float3;
#endif

#ifndef float4
    typedef Vector4<float> float4;
#endif

#ifndef uchar1
    typedef unsigned char uchar1;
#endif

#ifndef uchar2
    typedef Vector2<unsigned char> uchar2;
#endif

#ifndef uchar3
    typedef Vector3<unsigned char> uchar3;
#endif

#ifndef uchar4
    typedef Vector4<unsigned char> uchar4;
#endif

#ifndef double1
    typedef double double1;
#endif

#ifndef double2
    typedef Vector2<double> double2;
#endif

#ifndef double3
    typedef Vector3<double> double3;
#endif

#ifndef double4
    typedef Vector4<double> double4;
#endif

#ifndef uint1
    typedef unsigned int uint1;
#endif

#ifndef uint2
    typedef Vector2<unsigned int> uint2;
#endif

#ifndef uint3
    typedef Vector3<unsigned int> uint3;
#endif

#ifndef uint4
    typedef Vector4<unsigned int> uint4;
#endif

    /**
    \ingroup Core_group
    \brief \ref Timesamp representation and utilities.
     */
    struct Timestamp {
        /**
        \brief Timestamp in nanoseconds.
         */
        uint64_t data_ns = 0;

        /**
        \brief Default constructor, inits the Timestamp to 0.
         */
        Timestamp() {
        }

        /**
        \brief Inits the Timestamp to a number in nanoseconds.
         */
        Timestamp(uint64_t _data_ns) : data_ns(_data_ns) {
        }

        template<typename T>
        Timestamp(T _data_ns) : data_ns(static_cast<uint64_t> (_data_ns)) {
        }

        /**
        \brief Gets the timestamp in nanoseconds.
         */
        inline uint64_t getNanoseconds() {
            return data_ns;
        }

        /**
        \brief Gets the timestamp in microseconds.
         */
        inline uint64_t getMicroseconds() {
            return data_ns / 1000ULL;
        }

        /**
        \brief Gets the timestamp in milliseconds.
         */
        inline uint64_t getMilliseconds() {
            return data_ns / 1000000ULL;
        }

        /**
        \brief Gets the timestamp in seconds.
         */
        inline uint64_t getSeconds() {
            return data_ns / 1000000000ULL;
        }

        /**
        \brief Sets the timestamp to a value in nanoseconds.
         */
        inline void setNanoseconds(uint64_t t_ns) {
            data_ns = t_ns;
        }

        /**
        \brief Sets the timestamp to a value in microoseconds.
         */
        inline void setMicroseconds(uint64_t t_us) {
            data_ns = t_us * 1000ULL;
        }

        /**
        \brief Sets the timestamp to a value in milliseconds.
         */
        inline void setMilliseconds(uint64_t t_ms) {
            data_ns = t_ms * 1000000ULL;
        }

        /**
        \brief Sets the timestamp to a value in seconds.
         */
        inline void setSeconds(uint64_t t_s) {
            data_ns = t_s * 1000000000ULL;
        }

        Timestamp& operator=(Timestamp other) {
            std::swap(data_ns, other.data_ns);
            return *this;
        }

        Timestamp& operator=(uint64_t other_data_ns) {
            std::swap(data_ns, other_data_ns);
            return *this;
        }

        Timestamp& operator+=(const Timestamp& rhs) {
            this->data_ns = this->data_ns + rhs.data_ns;
            return *this;
        }

        Timestamp& operator-=(const Timestamp& rhs) {
            this->data_ns = this->data_ns - rhs.data_ns;
            return *this;
        }

        Timestamp& operator*=(const Timestamp& rhs) {
            this->data_ns = this->data_ns * rhs.data_ns;
            return *this;
        }

        Timestamp& operator/=(const Timestamp& rhs) {
            this->data_ns = this->data_ns / rhs.data_ns;
            return *this;
        }

        operator unsigned long long int() const {
            return static_cast<unsigned long long int> (data_ns);
        }
    };

    inline bool operator<(const Timestamp& lhs, const Timestamp& rhs) {
        return lhs.data_ns < rhs.data_ns; // keep the same order
    }

    inline bool operator>(const Timestamp& lhs, const Timestamp& rhs) {
        return rhs < lhs;
    }

    inline bool operator<=(const Timestamp& lhs, const Timestamp& rhs) {
        return !(lhs > rhs);
    }

    inline bool operator>=(const Timestamp& lhs, const Timestamp& rhs) {
        return !(lhs < rhs);
    }

    inline bool operator==(const Timestamp& lhs, const Timestamp& rhs) {
        return lhs.data_ns == rhs.data_ns;
    }

    inline bool operator!=(const Timestamp& lhs, const Timestamp& rhs) {
        return !(lhs == rhs);
    }

    inline Timestamp operator+(Timestamp lhs, const Timestamp& rhs) {
        lhs += rhs;
        return lhs;
    }

    inline Timestamp operator-(Timestamp lhs, const Timestamp& rhs) {
        lhs -= rhs;
        return lhs;
    }

    inline Timestamp operator/(Timestamp lhs, const Timestamp& rhs) {
        lhs /= rhs;
        return lhs;
    }

    inline Timestamp operator*(Timestamp lhs, const Timestamp& rhs) {
        lhs *= rhs;
        return lhs;
    }
    ///@}

    /// @cond

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector2<T> &v2) {
        os << v2.x << " " << v2.y;
        return os;
    }

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector3<T> &v3) {
        os << v3.x << " " << v3.y << " " << v3.z;
        return os;
    }

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector4<T> &v4) {
        os << v4.x << " " << v4.y << " " << v4.z << " " << v4.w;
        return os;
    }

    /**
    \ingroup Core_group
    \brief Returns the current timestamp at the time the function is called. Can be compared to  sl::Camera::getCameraTimestamp for synchronization.

    Use this function to compare the current timestamp and the camera timestamp, since they have the same reference (Computer start time).
    \return The current timestamp in ns.
     */
    static inline Timestamp getCurrentTimeStamp() {
        Timestamp current_ts = 0ULL;
        current_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        return current_ts;
    }


#ifndef ZEDcudaSafeCall
#define ZEDcudaSafeCall(err) __cudaSafeCall(err, /*__FILENAME__*/ __CUSTOM__PRETTY__FUNC__, __FILENAME__, __LINE__)

    static inline cudaError __cudaSafeCall(cudaError err, const char *func, const char *file, const int line) {
        if (err != cudaSuccess)
            printf("in %s : cuda error [%d]: %s.\n", func, err, cudaGetErrorString(err));
        return err;
    }
#endif

#define TIMING
#ifdef TIMING
#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER start = std::chrono::high_resolution_clock::now();
#define DEF_START_TIMER auto start = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name) std::cout << name << " " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms " << std::endl;
#else
#define INIT_TIMER
#define START_TIMER
#define DEF_START_TIMER
#define STOP_TIMER(name)
#endif
    /// @endcond
}
#endif /* __TYPES_HPP__ */
/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 * 
 * 1. Definitions
 * 
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2 or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera, ZED 2 camera and the ZED Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2 or ZED Mini camera.
 * 
 * 2. License
 * 
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * 
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2 or ZED Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2
 *       or ZED Mini camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2 or ZED Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2 or ZED Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2 or ZED Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 * 
 * 3. Warranty
 * 
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2 or ZED Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 * 
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * 
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 * 
 * 5. Choice of Law
 * 
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __CORE_HPP__
#define __CORE_HPP__


namespace sl {

    // Avoid conflict with Darknet GPU macro and MEM::GPU enum
#ifdef GPU
#undef GPU
#endif

    /**
    \enum MEM
    \ingroup Core_group
    \brief List available memory type
     */
    enum class MEM {
        CPU = 1, /**< CPU Memory (Processor side).*/
        GPU = 2 /**< GPU Memory (Graphic card side).*/
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const MEM &mem);

    inline std::ostream &operator<<(std::ostream &os, const MEM &mem) {
        return os << toString(mem);
    }

    inline MEM operator|(MEM a, MEM b) {
        return static_cast<MEM> (static_cast<int> (a) | static_cast<int> (b));
    }
    ///@endcond

    /**
    \enum COPY_TYPE
    \ingroup Core_group
    \brief List available copy operation on Mat
     */
    enum class COPY_TYPE {
        CPU_CPU, /**< copy data from CPU to CPU.*/
        CPU_GPU, /**< copy data from CPU to GPU.*/
        GPU_GPU, /**< copy data from GPU to GPU.*/
        GPU_CPU /**< copy data from GPU to CPU.*/
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const COPY_TYPE &cpy);

    inline std::ostream &operator<<(std::ostream &os, const COPY_TYPE &cpy) {
        return os << toString(cpy);
    }
    ///@endcond

    /**
    \enum MAT_TYPE
    \ingroup Core_group
    \brief List available Mat formats.
     */
    enum class MAT_TYPE {
        F32_C1, /**< float 1 channel.*/
        F32_C2, /**< float 2 channels.*/
        F32_C3, /**< float 3 channels.*/
        F32_C4, /**< float 4 channels.*/
        U8_C1, /**< unsigned char 1 channel.*/
        U8_C2, /**< unsigned char 2 channels.*/
        U8_C3, /**< unsigned char 3 channels.*/
        U8_C4 /**< unsigned char 4 channels.*/
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const MAT_TYPE &type);

    inline std::ostream &operator<<(std::ostream &os, const MAT_TYPE &type) {
        return os << toString(type);
    }
    ///@endcond

    /**
    \class Mat
    \ingroup Core_group
    \brief The Mat class can handle multiple matrix format from 1 to 4 channels, with different value types (float or uchar), and can be stored CPU and/or GPU side.

    \ref Mat is defined in a row-major order, it means that, for an image buffer, the entire first row is stored first, followed by the entire second row, and so on.

    The CPU and GPU buffer aren't automatically synchronized for performance reasons, you can use \ref updateCPUfromGPU / \ref updateGPUfromCPU to do it.
    If you are using the GPU side of the Mat object, you need to make sure to call \ref free before destroying the sl::Camera object.
    The destruction of the sl::Camera object delete the CUDA context needed to free the GPU Mat memory.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Mat {
    private:
        //  Array size.
        Resolution size;

        // Number of values by pixels.
        size_t channels = 0;

        // GPU Step of the Mat in Bytes.
        size_t step_gpu = 0;

        // CPU Step of the Mat in Bytes.
        size_t step_cpu = 0;

        // size in bytes of one pixel
        size_t pixel_bytes = 0;

        // Data format.
        MAT_TYPE data_type;

        // Type of allocated memory.
        MEM mem_type = MEM::CPU;

        // Pointer to memory on HOST/CPU, if available.
        uchar1 *ptr_cpu = NULL;

        // Pointer to memory on DEVICE/GPU, if available.
        uchar1 *ptr_gpu = NULL;

        // Defines if the Mat is initialized.
        bool init = false;

        // Defines if the memory is owned (and thus freed) or shared.
        bool memory_owner = false;

        //private
        int castSLMat();

        //private
        void ref(const Mat &mat);
    public:
        // Variable used in verbose mode to indicate which Mat is printing informations.
        // Default set to n/a to avoid empty string if not filled.
        String name = "n/a";

        // Whether the MAT can display informations or not.
        bool verbose = false;

        // Data timestamp
        Timestamp timestamp = 0;

        /**
         * \brief empty Mat default constructor.
         */
        Mat();

        /**
        \brief Mat constructor.

        This function directly allocates the requested memory. It calls \ref alloc.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param memory_type : defines where the buffer will be stored. (\ref MEM::CPU and/or \ref MEM::GPU).
         */
        Mat(size_t width, size_t height, MAT_TYPE mat_type, MEM memory_type = MEM::CPU);

        /**
        \brief Mat constructor from an existing data pointer.

        This function doesn't allocate the memory.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param ptr : pointer to the data array. (CPU or GPU).
        \param step : step of the data array. (the Bytes size of one pixel row).
        \param memory_type : defines where the buffer will be stored. (\ref MEM::CPU and/or \ref MEM::GPU).
         */
        Mat(size_t width, size_t height, MAT_TYPE mat_type, sl::uchar1 *ptr, size_t step, MEM memory_type = MEM::CPU);

        /**
        \brief Mat constructor from two existing data pointers, CPU and GPU.

        This function doesn't allocate the memory.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param ptr_cpu : CPU pointer to the data array.
        \param step_cpu : step of the CPU data array (the Bytes size of one pixel row).
        \param ptr_gpu : GPU pointer to the data array.
        \param step_gpu : step of the GPU data array (the Bytes size of one pixel row).
         */
        Mat(size_t width, size_t height, MAT_TYPE mat_type, sl::uchar1 *ptr_cpu, size_t step_cpu, sl::uchar1 *ptr_gpu, size_t step_gpu);

        /**
        \brief Mat constructor.

        This function directly allocates the requested memory. It calls \ref alloc.

        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param memory_type : defines where the buffer will be stored (\ref MEM::CPU and/or \ref MEM::GPU).
         */
        Mat(Resolution resolution, MAT_TYPE mat_type, MEM memory_type = MEM::CPU);

        /**
        \brief Mat constructor from an existing data pointer.

        This function doesn't allocate the memory.

        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param ptr : pointer to the data array. (CPU or GPU).
        \param step : step of the data array (the Bytes size of one pixel row).
        \param memory_type : defines where the buffer will be stored. (\ref MEM::CPU and/or \ref MEM::GPU).
         */
        Mat(Resolution resolution, MAT_TYPE mat_type, sl::uchar1 *ptr, size_t step, MEM memory_type = MEM::CPU);

        /**
        \brief Mat constructor from two existing data pointers, CPU and GPU.

        This function doesn't allocate the memory.

        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param ptr_cpu : CPU pointer to the data array.
        \param step_cpu : step of the CPU data array (the Bytes size of one pixel row).
        \param ptr_gpu : GPU pointer to the data array.
        \param step_gpu : step of the GPU data array (the Bytes size of one pixel row).
         */
        Mat(Resolution resolution, MAT_TYPE mat_type, sl::uchar1 *ptr_cpu, size_t step_cpu, sl::uchar1 *ptr_gpu, size_t step_gpu);

        /**
        \brief Mat constructor by copy (shallow copy).

        This function doesn't allocate the memory.

        \param mat : the reference to the \ref Mat to copy.
         */
        Mat(const Mat &mat);

        /**
        \brief Allocates the Mat memory.
        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE::F32_C1, \ref MAT_TYPE::U8_C4...).
        \param memory_type : defines where the buffer will be stored. (\ref MEM::CPU and/or \ref MEM::GPU).

        \warning It erases previously allocated memory.
         */
        void alloc(size_t width, size_t height, MAT_TYPE mat_type, MEM memory_type = MEM::CPU);

        /**
        \brief Allocates the Mat memory.
        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (sl::MAT_TYPE::F32_C1,sl::MAT_TYPE::U8_C4...).
        \param memory_type : defines where the buffer will be stored. (sl::MEM::CPU and/or sl::MEM::GPU).

        \warning It erases previously allocated memory.
         */
        void alloc(Resolution resolution, MAT_TYPE mat_type, MEM memory_type = MEM::CPU);

        /**
        \brief Mat destructor. This function calls \ref free to release owned memory.
         */
        ~Mat();

        /**
        \brief Free the owned memory.
        \param memory_type : specify whether you want to free the \ref MEM::CPU and/or \ref MEM::GPU memory.
         */
        void free(MEM memory_type = MEM::CPU | MEM::GPU);

        /**
        \brief Performs a shallow copy.

        This function doesn't copy the data array, it only copies the pointer.

        \param that : the \ref Mat to be copied.
        \return The new \ref Mat object which point to the same data as that.
         */
        Mat &operator=(const Mat &that);

        /**
        \brief Downloads data from DEVICE (GPU) to HOST (CPU), if possible.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note If no CPU or GPU memory are available for this Mat, some are directly allocated.
        \note If verbose sets, you have informations in case of failure.
         */
        ERROR_CODE updateCPUfromGPU();

        /**
        \brief Uploads data from HOST (CPU) to DEVICE (GPU), if possible.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note If no CPU or GPU memory are available for this Mat, some are directly allocated.
        \note If verbose sets, you have informations in case of failure.
         */
        ERROR_CODE updateGPUfromCPU();

        /**
        \brief Copies data an other Mat (deep copy).
        \param dst : the Mat where the data will be copied.
        \param cpyType : specify the memories that will be used for the copy.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note If the destination is not allocated or has a not a compatible \ref MAT_TYPE or \ref Resolution,
        current memory is freed and new memory is directly allocated.
         */
        ERROR_CODE copyTo(Mat &dst, COPY_TYPE cpyType = COPY_TYPE::CPU_CPU) const;

        /**
        \brief Copies data from an other Mat (deep copy).
        \param src : the Mat where the data will be copied from.
        \param cpyType : specify the memories that will be used for the update.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note If the current Mat is not allocated or has a not a compatible \ref MAT_TYPE or \ref Resolution with the source,
        current memory is freed and new memory is directly allocated.
         */
        ERROR_CODE setFrom(const Mat &src, COPY_TYPE cpyType = COPY_TYPE::CPU_CPU, cudaStream_t stream = 0);

        /**
        \brief Reads an image from a file (only if \ref MEM::CPU is available on the current \ref Mat).

        \param filePath : file path including the name and extension.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

                \note Supported \ref MAT_TYPE are :
                \ref MAT_TYPE::F32_C1 for PNG/PFM/PGM,
                \ref MAT_TYPE::F32_C3 for PCD/PLY/VTK/XYZ,
                \ref MAT_TYPE::F32_C4 for PCD/PLY/VTK/WYZ,
                \ref MAT_TYPE::U8_C1 for PNG/JPG,
                \ref MAT_TYPE::U8_C3 for PNG/JPG,
                \ref MAT_TYPE::U8_C4 for PNG/JPG,
         */
        ERROR_CODE read(const String &filePath);

        /**
        \brief Writes the \ref Mat (only if \ref MEM::CPU is available) into a file defined by its extension.
		       
        \param filePath : file path including the name and extension.
        \param memory_type : memory type (CPU or GPU) of the Mat.
        \param compression_level : level of compression between 0 (lowest compression == highest size == highest quality(jpg)) and 100 (highest compression == lowest size == lowest quality(jpg)).
        \note Specific/default value for compression_level = -1 : This will set the default quality for PNG(30) or JPEG(5).
        \note compression_level is only supported for U8_Cx \ref MAT_TYPE.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note Supported \ref MAT_TYPE are : 
                \ref MAT_TYPE::F32_C1 for PNG/PFM/PGM,
                \ref MAT_TYPE::F32_C3 for PCD/PLY/VTK/XYZ,
                \ref MAT_TYPE::F32_C4 for PCD/PLY/VTK/WYZ,
                \ref MAT_TYPE::U8_C1 for PNG/JPG,
                \ref MAT_TYPE::U8_C3 for PNG/JPG,
                \ref MAT_TYPE::U8_C4 for PNG/JPG,
         */
        ERROR_CODE write(const String &filePath, sl::MEM memory_type = sl::MEM::CPU, int compression_level = -1);

        /**
        \brief Fills the Mat with the given value.

        This function overwrite all the matrix.

        \param value : the value to be copied all over the matrix.
        \param memory_type : defines which buffer to fill, CPU and/or GPU.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.
         */
        template <typename T>
        ERROR_CODE setTo(T value, sl::MEM memory_type = MEM::CPU);

        /**
        \brief Sets a value to a specific point in the matrix.
        \param x : specify the column.
        \param y : specify the row.
        \param value : the value to be set.
        \param memory_type : defines which memory will be updated.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.

        \warning Not efficient for \ref MEM::GPU, use it on sparse data.
         */
        template <typename N>
        ERROR_CODE setValue(size_t x, size_t y, N value, sl::MEM memory_type = MEM::CPU);

        /**
        \brief Returns the value of a specific point in the matrix.
        \param x : specify the column
        \param y : specify the row
        \param memory_type : defines which memory should be read.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.

        \warning Not efficient for \ref MEM::GPU, use it on sparse data.
         */
        template <typename N>
        ERROR_CODE getValue(size_t x, size_t y, N *value, MEM memory_type = MEM::CPU) const;

        /**
        \brief Returns the width of the matrix.
        \return The width of the matrix in pixels.
         */
        inline size_t getWidth() const {
            return size.width;
        }

        /**
        \brief Returns the height of the matrix.
        \return The height of the matrix in pixels.
         */
        inline size_t getHeight() const {
            return size.height;
        }

        /**
        \brief Returns the resolution (width and height) of the matrix.
        \return The resolution of the matrix in pixels.
         */
        inline Resolution getResolution() const {
            return size;
        }

        /**
        \brief Returns the number of values stored in one pixel.
        \return The number of values in a pixel.
         */
        inline size_t getChannels() const {
            return channels;
        }

        /**
        \brief Returns the format of the matrix.
        \return The format of the current Mat.
         */
        inline MAT_TYPE getDataType() const {
            return data_type;
        }

        /**
        \brief Returns the type of memory (CPU and/or GPU).
        \return The type of allocated memory.
         */
        inline MEM getMemoryType() const {
            return mem_type;
        }

        /**
        \brief Returns the CPU or GPU data pointer.
        \param memory_type : specify whether you want \ref MEM::CPU or \ref MEM::GPU step.
        \return The pointer of the Mat data.
         */
        template <typename N>
        N *getPtr(MEM memory_type = MEM::CPU) const;

        /**
        \brief Returns the memory step in Bytes (the Bytes size of one pixel row).
        \param memory_type : specify whether you want \ref MEM::CPU or \ref MEM::GPU step.
        \return The step in bytes of the specified memory.
         */
        size_t getStepBytes(MEM memory_type = MEM::CPU) const;

        /**
        \brief Returns the memory step in number of elements (the number of values in one pixel row).
        \param memory_type : specify whether you want \ref MEM::CPU or \ref MEM::GPU step.
        \return The step in number of elements.
         */
        template <typename N>
        inline size_t getStep(MEM memory_type = MEM::CPU) const {
            return getStepBytes(memory_type) / sizeof (N);
        }

        /**
        \brief Returns the memory step in number of elements (the number of values in one pixel row).
        \param memory_type : specify whether you want \ref MEM::CPU or \ref MEM::GPU step.
        \return The step in number of elements.
         */
        inline size_t getStep(MEM memory_type = MEM::CPU)const {
            switch (data_type) {
                case MAT_TYPE::F32_C1:
                    return getStep<sl::float1>(memory_type);
                case MAT_TYPE::F32_C2:
                    return getStep<sl::float2>(memory_type);
                case MAT_TYPE::F32_C3:
                    return getStep<sl::float3>(memory_type);
                case MAT_TYPE::F32_C4:
                    return getStep<sl::float4>(memory_type);
                case MAT_TYPE::U8_C1:
                    return getStep<sl::uchar1>(memory_type);
                case MAT_TYPE::U8_C2:
                    return getStep<sl::uchar2>(memory_type);
                case MAT_TYPE::U8_C3:
                    return getStep<sl::uchar3>(memory_type);
                case MAT_TYPE::U8_C4:
                    return getStep<sl::uchar4>(memory_type);
            }
            return 0;
        }

        /**
        \brief Returns the size in bytes of one pixel.
        \return The size in bytes of a pixel.
         */
        inline size_t getPixelBytes() const {
            return pixel_bytes;
        }

        /**
        \brief Returns the size in bytes of a row.
        \return The size in bytes of a row.
         */
        inline size_t getWidthBytes() const {
            return pixel_bytes * size.width;
        }

        /**
        \brief Return the informations about the Mat into a \ref String.
        \return A string containing the Mat informations.
         */
        String getInfos();

        /**
        \brief Defines whether the Mat is initialized or not.
        \return True if current Mat has been allocated (by the constructor or therefore).
         */
        inline bool isInit() const {
            return init;
        }

        /**
        \brief Returns whether the Mat is the owner of the memory it access.

        If not, the memory won't be freed if the Mat is destroyed.
        \return True if the Mat is owning its memory, else false.
         */
        inline bool isMemoryOwner() const {
            return memory_owner;
        }

        /**
        \brief Duplicates Mat by copy (deep copy).
        \param src : the reference to the Mat to copy.
         This function copies the data array(s), it mark the new Mat as the memory owner.
         */
        ERROR_CODE clone(const Mat &src);

        /**
        \brief Moves Mat data to another Mat.

        This function gives the attribute of the current Mat to the specified one. (No copy).
        \param dst : the reference to the Mat to move.
        \note : the current Mat is then no more usable since its loose its attributes.
         */
        ERROR_CODE move(Mat &dst);

        /**
        \brief Swaps the content of the provided Mat (only swaps the pointers, no data copy).

        This function swaps the pointers of the given Mat.
        \param mat1 : the first mat.
        \param mat2 : the second mat.
         */
        static void swap(Mat &mat1, Mat &mat2);
    };


    ///@cond
    class SL_CORE_EXPORT Rotation;
    class SL_CORE_EXPORT Translation;
    class SL_CORE_EXPORT Orientation;
    class SL_CORE_EXPORT Transform;
    ///@endcond

    /**
    \class Rotation
    \ingroup PositionalTracking_group
    \brief Designed to contain rotation data of the positional tracking. It inherits from the generic \ref Matrix3f
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Rotation : public Matrix3f {
    public:
        /**
        \brief empty Rotation default constructor.
         */
        Rotation();

        /**
        \brief Rotation copy constructor (deep copy).
        \param rotation : the Rotation to copy.
         */
        Rotation(const Rotation &rotation);

        /**
        \brief Rotation copy constructor (deep copy).
        \param mat : the mat to copy.
         */
        Rotation(const Matrix3f &mat);

        /**
        \brief Rotation constructor from an Orientation.

        It converts the Orientation representation to the Rotation one.
        \param orientation : the Orientation to be used.
         */
        Rotation(const Orientation &orientation);

        /**
        \brief Creates a Rotation representing the 3D rotation of angle around an arbitrary 3D axis.
        \param angle : the rotation angle in rad.
        \param axis : the 3D axis to rotate around.
         */
        Rotation(const float angle, const Translation &axis);

        /**
        \brief Sets the Rotation from an Orientation.
        \param orientation : the Orientation containing the rotation to set.
         */
        void setOrientation(const Orientation &orientation);

        /**
        \brief Returns the Orientation corresponding to the current Rotation.
        \return The rotation of the current orientation.
         */
        Orientation getOrientation() const;

        /**
        \brief Returns the 3x1 rotation vector obtained from 3x3 rotation matrix using Rodrigues formula.
        \return The rotation vector.
         */
        float3 getRotationVector();

        /**
        \brief Sets the Rotation from a rotation vector (using Rodrigues' transformation).
        \param vec_rot : the  Rotation Vector.
         */
        void setRotationVector(const float3 &vec_rot);

        /**
        \brief Convert the Rotation as Euler angles
        \param radian : Define if the angle in is radian or degree
        \return The Euler angles, as a \ref float3 representing the rotations around the X, Y and Z axes. (YZX convention)
         */
        float3 getEulerAngles(bool radian = true) const;

        /**
        \brief Sets the Rotation from the Euler angles.
        \param euler_angles : The Euler angles, as a \ref float3
        \param radian : Define if the angle in is radian or degree
         */
        void setEulerAngles(const float3 &euler_angles, bool radian = true);
    };

    /**
    \class Translation
    \ingroup PositionalTracking_group
    \brief Designed to contain translation data of the positional tracking.

    \ref Translation is a vector as  [tx, ty, tz].
    You can access the data with the 't' ptr or by element name as :
    tx, ty, tz  <-> | 0 1 2 |
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Translation : public float3 {
    public:
        /**
        \brief empty Translation default constructor.
         */
        Translation();

        /**
        \brief Translation copy constructor (deep copy).
        \param translation : the Translation to copy.
         */
        Translation(const Translation &translation);

        /**
        \brief Translation constructor.
        \param t1 : the x translation.
        \param t2 : the y translation.
        \param t3 : the z translation.
         */
        Translation(float t1, float t2, float t3);

        /**
        \brief Translation constructor.
        \param in : vector.
         */
        Translation(float3 in);

        /**
        \brief Multiplication operator by an Orientation.
        \param mat : Orientation.
        \return The current Translation after being multiplied by the orientation.
         */
        Translation operator*(const Orientation &mat) const;

        /**
        \brief Normalizes the current translation.
         */
        void normalize();

        /**
        \brief Get the normalized version of a given Translation.
        \param tr : the Translation to be used.
        \return An other Translation object, which is equal to tr.normalize.
         */
        static Translation normalize(const Translation &tr);

        /**
        \brief Get the value at specific position in the Translation.
        \param x : the position of the value
        \return The value at the x position.
         */
        float &operator()(int x);
    };

    /**
    \class Orientation
    \ingroup PositionalTracking_group
    \brief Designed to contain orientation (quaternion) data of the positional tracking.

    \ref Orientation is a vector defined as [ox, oy, oz, ow].
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Orientation : public float4 {
    public:
        /**
        \brief empty Orientation default constructor.
         */
        Orientation();

        /**
        \brief Orientation copy constructor (deep copy).
        \param orientation : the Orientation to copy.
         */
        Orientation(const Orientation &orientation);

        /**
        \brief Orientation copy constructor (deep copy).
        \param in : the vector to copy.

        Set in the following order : [ox, oy, oz, ow].
         */
        Orientation(const float4 &in);

        /**
        \brief Orientation constructor from an Rotation.

        It converts the Rotation representation to the Orientation one.
        \param rotation : the Rotation to be used.
         */
        Orientation(const Rotation &rotation);

        /**
        \brief Orientation constructor from a vector represented by two Translation.
        \param tr1 : the first point of the vector.
        \param tr2 : the second point of the vector.
         */
        Orientation(const Translation &tr1, const Translation &tr2);

        /**
        \brief Returns the value at specific position in the Orientation.
        \param x : the position of the value
        \return The value at the x position.
         */
        float operator()(int x);

        /**
        \brief Multiplication operator by an Orientation.
        \param orientation : the orientation.
        \return The current orientation after being multiplied by the other orientation.
         */
        Orientation operator*(const Orientation &orientation) const;

        /**
        \brief Sets the orientation from a Rotation.
        \param rotation : the Rotation to be used.
         */
        void setRotationMatrix(const Rotation &rotation);

        /**
        \brief Returns the current orientation as a Rotation.
        \return The rotation computed from the orientation data.
         */
        Rotation getRotationMatrix() const;

        /**
        \brief Sets the orientation from a Rotation.
        \param rotation : the Rotation to be used.

        \deprecated See \ref setRotationMatrix
         */

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use setRotationMatrix instead") /* @endcond*/
        inline void setRotation(const Rotation &rotation) {
            setRotationMatrix(rotation);
        }

        /**
        \brief Returns the current orientation as a Rotation.
        \return The rotation computed from the orientation data.

        \deprecated See \ref getRotationMatrix
         */

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use getRotationMatrix instead") /* @endcond*/
        inline Rotation getRotation() const {
            return getRotationMatrix();
        }

        /**
        \brief Sets the current Orientation to identity.
         */
        void setIdentity();

        /**
        \brief Creates an Orientation initialized to identity.
        \return An identity Orientation.
         */
        static Orientation identity();

        /**
        \brief Fills the current Orientation with zeros.
         */
        void setZeros();

        /**
        \brief Creates an Orientation filled with zeros.
        \return An Orientation filled with zeros.
         */
        static Orientation zeros();

        /**
         * \brief Normalizes the current Orientation.
         */
        void normalise();

        /**
        \brief Creates the normalized version of an existing Orientation.
        \param orient : the Orientation to be used.
        \return The normalized version of the Orientation.
         */
        static Orientation normalise(const Orientation &orient);
    };

    /**
    \class Transform
    \ingroup PositionalTracking_group
    \brief Designed to contain translation and rotation data of the positional tracking.

    It contains the orientation as well. It can be used to create any type of Matrix4x4 or \ref Matrix4f that must be specifically used for handling a rotation and position information (OpenGL, Tracking...)
    It inherits from the generic \ref Matrix4f
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Transform : public Matrix4f {
    public:
        /**
        \brief Transform default constructor.
         */
        Transform();

        /**
        \brief Transform copy constructor (deep copy).
         * \param motion : the Transform to copy.
         */
        Transform(const Transform &motion);

        /**
        \brief Transform copy constructor (deep copy).
        \param mat : the Matrix4f to copy.
         */
        Transform(const Matrix4f &mat);

        /**
        \brief Transform constructor from a Rotation and a Translation.
        \param rotation : the Rotation to copy.
        \param translation : the Translation to copy.
         */
        Transform(const Rotation &rotation, const Translation &translation);

        /**
        \brief Transform constructor from an Orientation and a Translation.
        \param orientation : the Orientation to copy.
        \param translation : the Translation to copy.
         */
        Transform(const Orientation &orientation, const Translation &translation);

        /**
        \brief Sets the rotation of the current Transform from an Rotation.
        \param rotation : the Rotation to be used.
         */
        void setRotationMatrix(const Rotation &rotation);

        /**
        \brief Returns the Rotation of the current Transform.
        \return The Rotation created from the Transform values.
        \warning The given Rotation contains a copy of the Transform values. Not references.
         */
        Rotation getRotationMatrix() const;

        /**
        \brief Sets the rotation of the current Transform from an Rotation.
        \param rotation : the Rotation to be used.

        \deprecated See \ref setRotationMatrix
         */

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use setRotationMatrix instead") /*@endcond*/
        inline void setRotation(const Rotation &rotation) {
            setRotationMatrix(rotation);
        }

        /**
        \brief Returns the Rotation of the current Transform.
        \return The Rotation created from the Transform values.
        \warning The given Rotation contains a copy of the Transform values. Not references.

        \deprecated See \ref getRotationMatrix
         */

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use getRotationMatrix instead") /*@endcond*/
        inline Rotation getRotation() const {
            return getRotationMatrix();
        }

        /**
        \brief Sets the translation of the current Transform from an Translation.
        \param translation : the Translation to be used.
         */
        void setTranslation(const Translation &translation);

        /**
        \brief Returns the Translation of the current Transform.
        \return The Translation created from the Transform values.
        \warning The given Translation contains a copy of the Transform values. Not references.
         */
        Translation getTranslation() const;

        /**
        \brief Sets the orientation of the current Transform from an Orientation.
        \param orientation : the Orientation to be used.
         */
        void setOrientation(const Orientation &orientation);

        /**
        \brief Returns the Orientation of the current Transform.
        \return The Orientation created from the Transform values.
        \warning The given Orientation contains a copy of the Transform values. Not references.
         */
        Orientation getOrientation() const;

        /**
        \brief Returns the 3x1 rotation vector obtained from 3x3 rotation matrix using Rodrigues formula.
        \return The rotation vector.
         */
        float3 getRotationVector();

        /**
        \brief Sets the Rotation 3x3 of the Transform with a 3x1 rotation vector (using Rodrigues' transformation).
        \param vec_rot : vector that contains the rotation value for each axis (rx,ry,rz).
         */
        void setRotationVector(const float3 &vec_rot);

        /**
        \brief Convert the Rotation of the Transform as Euler angles
        \param radian : Define if the angle in is radian or degree
        \return The Euler angles, as a \ref float3 representing the rotations around the X, Y and Z axes. (YZX convention)
         */
        float3 getEulerAngles(bool radian = true) const;

        /**
        \brief Sets the Rotation of the Transform from the Euler angles.
        \param euler_angles : The Euler angles, as a \ref float3
        \param radian : Define if the angle in is radian or degree
         */
        void setEulerAngles(const float3 &euler_angles, bool radian = true);
    };

    /**
    \struct CameraParameters
    \ingroup Depth_group
    \brief Intrinsic parameters of a camera.

    Those information about the camera will be returned by \ref Camera::getCameraInformation().
    
    \note Similar to the CalibrationParameters, those parameters are taken from the settings file (SNXXX.conf) and are modified during the sl::Camera::open call when running a self-calibration).
    Those parameters given after sl::Camera::open call, represent the camera matrix corresponding to rectified or unrectified images.
    \note When filled with rectified parameters, fx,fy,cx,cy must be the same for Left and Right Camera once sl::Camera::open has been called. Since distortion is corrected during rectification, distortion should not be considered on rectified images.

     */
    struct CameraParameters {
        float fx; /**< Focal length in pixels along x axis. */
        float fy; /**< Focal length in pixels along y axis. */
        float cx; /**< Optical center along x axis, defined in pixels (usually close to width/2). */
        float cy; /**< Optical center along y axis, defined in pixels (usually close to height/2). */
        double disto[5]; /**< Distortion factor : [ k1, k2, p1, p2, k3 ]. Radial (k1,k2,k3) and Tangential (p1,p2) distortion.*/
        float v_fov; /**< Vertical field of view, in degrees. */
        float h_fov; /**< Horizontal field of view, in degrees.*/
        float d_fov; /**< Diagonal field of view, in degrees.*/
        Resolution image_size; /** size in pixels of the images given by the camera.*/
    };

    /**
    \struct CalibrationParameters
    \ingroup Depth_group
    \brief Intrinsic and Extrinsic parameters of the camera (translation and rotation).

    Those information about the camera will be returned by \ref Camera::getCameraInformation().

    \note The calibration/rectification process, called during sl::Camera::open, is using the raw parameters defined in the SNXXX.conf file, where XXX is the ZED Serial Number.
    \n Those values may be adjusted or not by the Self-Calibration to get a proper image alignment. After sl::Camera::open is done (with or without Self-Calibration activated) success, most of the stereo parameters (except Baseline of course) should be 0 or very close to 0.
    \n It means that images after rectification process (given by retrieveImage()) are aligned as if they were taken by a "perfect" stereo camera, defined by the new CalibrationParameters.
    \warning CalibrationParameters are returned in COORDINATE_SYSTEM::IMAGE, they are not impacted by the InitParameters::coordinate_system
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ CalibrationParameters {
        CameraParameters left_cam; /**< Intrinsic parameters of the left camera  */
        CameraParameters right_cam; /**< Intrinsic parameters of the right camera  */
        Transform stereo_transform; /**< Left to Right camera transform, expressed in user coordinate system and unit (defines by InitParameters). */

        float getCameraBaseline();

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use stereo_transform.getRotationMatrix instead")/*@endcond*/ float3 R; /**< \deprecated see stereo_transform, Rotation on its own (using Rodrigues' transformation) of the right sensor. The left is considered as the reference. Defined as 'tilt', 'convergence' and 'roll'. Using a \ref Rotation, you can use \ref Rotation::setRotationVector(R) to convert into other representations.*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use stereo_transform.getTranslation instead")/*@endcond*/ float3 T; /**< \deprecated see stereo_transform,  Translation between the two sensors. T.x is the distance between the two cameras (baseline) in the sl::UNIT chosen during sl::Camera::open (mm, cm, meters, inches...).*/
    };

    /**
    \struct SensorParameters
    \ingroup Sensors_group
    \brief Structure containing information about a single sensor available in the current device

    Those information about the camera sensors are available int the \ref CameraInformation struct returned by \ref Camera::getCameraInformation().

    \note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
     */
    struct SensorParameters {
        sl::SENSOR_TYPE type; /**< The type of the sensor as \ref DEVICE_SENSORS*/
        float resolution; /**< The resolution of the sensor. */
        float sampling_rate; /**< The sampling rate (or ODR) of the sensor. */
        sl::float2 range; /**< The range values of the sensor. MIN: `range.x`, MAX: `range.y` */
        float noise_density; /**< also known as white noise, given as continous (frequency independant). Units will be expressed in sensor_unit/√(Hz). `NAN` if the information is not available */
        float random_walk; /**< derived from the Allan Variance, given as continous (frequency independant). Units will be expressed in sensor_unit/s/√(Hz).`NAN` if the information is not available */
        sl::SENSORS_UNIT sensor_unit; /**< The string relative to the measurement unit of the sensor. */
        bool isAvailable;
    }; ///@}

    /**
    \struct SensorsConfiguration
    \ingroup Sensors_group
    \brief Structure containing information about all the sensors available in the current device

    Those information about the camera sensors are available int the \ref CameraInformation struct returned by \ref Camera::getCameraInformation().

    \note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
     */
    struct SensorsConfiguration {
        unsigned int firmware_version = 0; /**< The firmware version of the sensor module, 0 if no sensors are available (ZED camera model). */
        sl::Transform camera_imu_transform; /**< IMU to Left camera transform matrix, that contains rotation and translation between IMU frame and camera frame. */
        sl::SensorParameters accelerometer_parameters; /**< Configuration of the accelerometer device */
        sl::SensorParameters gyroscope_parameters; /**< Configuration of the gyroscope device */
        sl::SensorParameters magnetometer_parameters; /**< Configuration of the magnetometer device */
        sl::SensorParameters barometer_parameters; /**< Configuration of the barometer device */

        bool isSensorAvailable(const sl::SENSOR_TYPE &sensor_type); /**< Check if a sensor type is available on the device */
    }; ///@}

    /**
    \struct CameraConfiguration
    \ingroup Core_group
    \brief Structure containing information about the camera sensor

    Those information about the camera are available int the \ref CameraInformation struct returned by \ref Camera::getCameraInformation().

    \note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
    \warning CalibrationParameters are returned in COORDINATE_SYSTEM::IMAGE, they are not impacted by the InitParameters::coordinate_system
     */
    struct CameraConfiguration {
        CalibrationParameters calibration_parameters; /**< Intrinsic and Extrinsic stereo parameters for rectified/undistorded images (default).  */
        CalibrationParameters calibration_parameters_raw; /**< Intrinsic and Extrinsic stereo parameters for original images (unrectified/distorded). */
        unsigned int firmware_version = 0; /**< The internal firmware version of the camera. */
        float fps = 0; /**< The camera capture FPS */
        Resolution resolution; /**< The camera resolution */
    }; ///@}

    /**
    \struct CameraInformation
    \ingroup Core_group
    \brief Structure containing information of a single camera (serial number, model, input type, etc.)

    Those information about the camera will be returned by \ref Camera::getCameraInformation().

    \note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
     */
    struct CameraInformation {
        unsigned int serial_number = 0; /**< The serial number of the camera.  */
        sl::MODEL camera_model = sl::MODEL::LAST; /**< The model of the camera (ZED, ZED-M or ZED2). */
        sl::INPUT_TYPE input_type = sl::INPUT_TYPE::LAST; /**< Input type used in SDK. */
        CameraConfiguration camera_configuration; /**< Camera configuration as defined in \ref CameraConfiguration. */
        SensorsConfiguration sensors_configuration; /**< Device Sensors configuration as defined in \ref SensorsConfiguration. */

        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use camera_configuration.calibration_parameters instead")/*@endcond*/ CalibrationParameters calibration_parameters; /**< \deprecated see CameraConfiguration::calibration_parameters, Intrinsic and Extrinsic stereo parameters for rectified/undistorded images (default).*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use camera_configuration.calibration_parameters_raw instead")/*@endcond*/ CalibrationParameters calibration_parameters_raw; /**< \deprecated see CameraConfiguration::calibration_parameters_raw, Intrinsic and Extrinsic stereo parameters for original images (unrectified/distorded).*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use sensors_configuration.camera_imu_transform instead")/*@endcond*/ sl::Transform camera_imu_transform; /**< \deprecated see SensorsConfiguration::camera_imu_transform, IMU to Left camera transform matrix, that contains rotation and translation between IMU frame and camera frame.*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use camera_configuration.firmware_version instead")/*@endcond*/ unsigned int camera_firmware_version = 0; /**< \deprecated CameraConfiguration::firmware_version, Firmware version of the camera.*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use sensors_configuration.firmware_version instead")/*@endcond*/ unsigned int sensors_firmware_version = 0; /**< \deprecated see SensorsConfiguration::firmware_version, Firmware version of the sensors of ZED-M or ZED2.*/
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use camera_configuration.fps instead")/*@endcond*/ float camera_fps = 0; /**< \deprecated see CameraConfiguration::fps, camera frame rate. */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("Use camera_configuration.resolution instead")/*@endcond*/ Resolution camera_resolution; /**< \deprecated see CameraConfiguration::resolution, camera resolution. */
    }; ///@}

    /**
    \class Pose
    \ingroup PositionalTracking_group
    \brief Contains positional tracking data which gives the position and orientation of the ZED in 3D space.

    Different representations of position and orientation can be retrieved, along with timestamp and pose confidence.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Pose {
        friend class CameraMemberHandler;
        friend class Camera;
        //ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        \brief Default constructor which creates an empty Pose (identity).
         */
        Pose();

        /**
        \brief Pose constructor with deep copy.
         */
        Pose(const Pose &pose);

        /**
        \brief Pose constructor with deep copy.
         */
        Pose(const Transform &pose_data, unsigned long long timestamp = 0, int confidence = 0);

        /**
        \brief Pose destructor.
         */
        ~Pose();

        /**
        \brief Returns the translation from the pose.
        \return The (3x1) translation vector.
         */
        Translation getTranslation();

        /**
        \brief Returns the orientation from the pose.
        \return The (4x1) orientation vector.
         */
        Orientation getOrientation();

        /**
        \brief Returns the rotation (3x3) from the pose.
        \return The (3x3) rotation matrix.
         */
        Rotation getRotationMatrix();

        /**
        \brief Returns the rotation (3x3) from the pose.
        \return The (3x3) rotation matrix.
        \deprecated See \ref getRotationMatrix
         */

        /*@cond SHOWHIDDEN*/ SL_DEPRECATED("use getRotationMatrix instead") /*@endcond*/
        inline Rotation getRotation() {
            return getRotationMatrix();
        }

        /**
        \brief Returns the rotation (3x1 rotation vector obtained from 3x3 rotation matrix using Rodrigues formula) from the pose.
        \return The (3x1) rotation vector.
         */
        float3 getRotationVector();

        /**
        \brief Convert the Rotation of the Transform as Euler angles
        \param radian : Define if the angle in is radian or degree. default : true.
        \return The Euler angles, as a \ref float3 representing the rotations around the X, Y and Z axes. (YZX convention)
         */
        float3 getEulerAngles(bool radian = true);

        /**
        4x4 Matrix which contains the rotation (3x3) and the translation. Orientation is extracted from this transform as well.
         */
        Transform pose_data;

        /**
        Timestamp of the pose. This timestamp should be compared with the camera timestamp for synchronization.
         */
        sl::Timestamp timestamp;

        /**
        Confidence/Quality of the pose estimation for the target frame.
        \n A confidence metric of the tracking [0-100], 0 means that the tracking is lost, 100 means that the tracking can be fully trusted.
         */
        int pose_confidence;

        /**
        \brief 6x6 Pose covariance of translation (the first 3 values) and rotation in so3 (the last 3 values)
        
        \note Computed only if sl::PositionalTrackingParameters::enable_area_memory is disabled.
         */
        float pose_covariance[36];

        /**
        boolean that indicates if tracking is activated or not. You should check that first if something wrong.
         */
        bool valid;

        /**
        twist and twist covariance of the camera
        available in reference camera
         */
        float twist[6];

        float twist_covariance[36];
    };

    /**
    \class SensorsData
    \ingroup Sensors_group
    \brief Contains all sensors data (except image sensors) to be used for positional tracking or environment study.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ SensorsData {

        /**
        \enum CAMERA_MOTION_STATE
        \ingroup Sensors_group
        \brief Lists different states of the camera motion
         */
        enum class CAMERA_MOTION_STATE {
            STATIC, /**< The camera is static. */
            MOVING, /**< The camera is moving. */
            FALLING, /**< The camera is falling. */
            LAST
        };

        /**
        \class BarometerData
        \ingroup Sensors_group
        \brief Contains Barometer sensor data.
         */
        struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ BarometerData {
            /**
            Defines if the sensor is available in your camera.
             */
            bool is_available;

            /**
            Defines the sensors data timestamp
             */
            sl::Timestamp timestamp;

            /**
            Barometer ambient air pressure in hPa
             */
            float pressure;


            /**
            Relative altitude from first camera position (at open() time)
             */
            float relative_altitude;

            /**
             Realtime data acquisition rate [Hz]
             */
            float effective_rate;
        };

        /**
        \class TemperatureData
        \ingroup Sensors_group
        \brief Contains sensors temperatures data.
         */
        struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ TemperatureData {

            /**
            \enum SENSOR_LOCATION
            \ingroup Sensors_group
            \brief Defines the location of each sensor for \ref TemperatureData
             */
            enum class SENSOR_LOCATION {
                IMU, /**< The IMU sensor location */
                BAROMETER, /**< The Barometer sensor location */
                ONBOARD_LEFT, /**< The Temperature sensor left location */
                ONBOARD_RIGHT, /**< The Temperature sensor right location */
                LAST
            };

            ERROR_CODE get(SENSOR_LOCATION location, float& temperature);

            std::map<SENSOR_LOCATION, float> temperature_map;
        };

        /**
        \class MagnetometerData
        \ingroup Sensors_group
        \brief Contains Magnetometer sensor data.
         */
        struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ MagnetometerData {
            /**
            Defines if the sensor is available in your camera.
             */
            bool is_available;

            /**
            Defines the sensors data timestamp
             */

            sl::Timestamp timestamp;

            /**
            (3x1) Vector for magnetometer raw values (uncalibrated)
            In other words, the current magnetic field (uT), along with the x, y, and z axes.
             */
            sl::float3 magnetic_field_uncalibrated;

            /**
            (3x1) Vector for magnetometer values (using factory calibration)
            In other words, the current magnetic field (uT), along with the x, y, and z axes.
             */
            sl::float3 magnetic_field_calibrated;

            /**
             Realtime data acquisition rate [Hz]
             */
            float effective_rate;
        };

        /**
        \class IMUData
        \ingroup Sensors_group
        \brief Contains IMU sensor data.
         */
        struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ IMUData {
            /**
            Defines if the sensor is available in your camera.
             */
            bool is_available;

            /**
            Defines the sensors data timestamp
             */

            sl::Timestamp timestamp;

            /**
            IMU pose (IMU 6-dof fusion)
             */
            sl::Transform pose;

            /**
            (3x3) 3x3 Covariance matrix for pose orientation (x,y,z axes)
             */
            sl::Matrix3f pose_covariance;

            /**
            (3x1) Vector for angular velocity of the gyroscope, given in deg/s. Values are corrected from bias, scale and misalignment
            In other words, the current velocity at which the sensor is rotating around the x, y, and z axes.
            \note Those values can be directly ingested in a IMU fusion algorithm to extract quaternion
             */
            sl::float3 angular_velocity;

            /**
            (3x1) Vector for linear acceleration of the gyroscope given in m/s^2. Values are corrected from bias, scale and misalignment
            In other words, the current acceleration of the sensor, along with the x, y, and z axes.
            \note Those values can be directly ingested in a IMU fusion algorithm to extract quaternion
             */
            sl::float3 linear_acceleration;


            /**
            (3x1) Vector for angular velocity of the gyroscope, given in deg/s, uncorrected from imu calibration.
            In other words, the current velocity at which the sensor is rotating around the x, y, and z axes.
            \note those values are the exact raw values from the IMU
             */
            sl::float3 angular_velocity_uncalibrated;

            /**
            (3x1) Vector for linear acceleration of the accelerometer, given in m/s^2, uncorrected from imu calibration.
            In other words, the current acceleration of the sensor, along with the x, y, and z axes.
            \note those values are the exact raw values from the IMU
             */
            sl::float3 linear_acceleration_uncalibrated;


            /**
            (3x3) 3x3 Covariance matrix for the angular velocity of the gyroscope
             */
            sl::Matrix3f angular_velocity_covariance;

            /**
            (3x3) 3x3 Covariance matrix for the linear acceleration of the accelerometer
             */
            sl::Matrix3f linear_acceleration_covariance;

            /**
             Realtime data acquisition rate [Hz]
             */
            float effective_rate;
        };

        /**
        \brief Default constructor which creates an empty SensorsData (identity).
         */
        SensorsData();

        /**
        \brief SensorsData constructor with deep copy.
         */
        SensorsData(const SensorsData &data);

        /**
        \brief Defines the \ref BarometerData .
         */
        BarometerData barometer;

        /**
        \brief Defines the \ref TemperatureData .
         */
        TemperatureData temperature;

        /**
        \brief Defines the \ref MagnetometerData .
         */
        MagnetometerData magnetometer;

        /**
        \brief Defines the \ref IMUData .
         */
        IMUData imu;

        ////////////////// Detection //////////////////////
        /**
        Indicates if the camera is static, moving or falling
         */
        CAMERA_MOTION_STATE camera_moving_state;


        ////////////////// Sync //////////////////////

        /**
        Indicates if the Sensors data has been taken during a frame capture on sensor.
        If value is 1, SensorsData has been taken during the same time than a frame has been acquired by the left sensor (the time precision is linked to the IMU rate, therefore 800Hz == 1.3ms)
        If value is 0, the data has not been taken during a frame acquisition.
         */
        int image_sync_trigger;
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const sl::SensorsData::CAMERA_MOTION_STATE &camera_moving_state);

    inline std::ostream &operator<<(std::ostream &os, const sl::SensorsData::CAMERA_MOTION_STATE &camera_moving_state) {
        return os << toString(camera_moving_state);
    }

    String SL_CORE_EXPORT toString(const sl::SensorsData::TemperatureData::SENSOR_LOCATION& sensor_loc);

    inline std::ostream& operator<<(std::ostream& os, const sl::SensorsData::TemperatureData::SENSOR_LOCATION& sensor_loc) {
        return os << toString(sensor_loc);
    }
    ///@endcond



    /*!
    \brief Compute the rotation matrix from the gravity vector : the rotation can used to find the world rotation from the gravity of an IMU
    \param axis_to_align : define the axis to align with the gravity, for instance : to align the "y" axis, axis_to_align = (0, 1, 0)'
    \param gravity_vector : the gravity vector, acceleration set by an IMU
    \return Rotation : rotation matrix, useful for Camera::detectFloorPlane as a gravity prior
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Rotation computeRotationMatrixFromGravity(sl::float3 axis_to_align, sl::float3 gravity_vector);

    /*!
    \brief Get the coordinate transform conversion matrix to change coordinate system.
    \param coord_system_src : the source coordinate system.
    \param coord_system_dst : the destination coordinate system.
    \return Matrix3f : transformation matrix, to apply to a \ref float3 point simply multiply by this matrix (pt_coord_dst = tf_matrix * pt_coord_src).
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Matrix3f getCoordinateTransformConversion3f(COORDINATE_SYSTEM coord_system_src, COORDINATE_SYSTEM coord_system_dst);

    /*!
    \brief Get the coordinate transform conversion matrix to change coordinate system.
    \param coord_system_src : the source coordinate system.
    \param coord_system_dst : the destination coordinate system.
    \return Matrix4f : transformation matrix, to apply to a \ref float4 point simply multiply by this matrix (pt_coord_dst = tf_matrix * pt_coord_src).
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Matrix4f getCoordinateTransformConversion4f(COORDINATE_SYSTEM coord_system_src, COORDINATE_SYSTEM coord_system_dst);

    /*!
    \brief Change the coordinate system of a matrix.
    \param floatMat : (in/out) matrix to transform, can be either a \ref MAT_TYPE::F32_C4 (the fourth value will be ignored as it contained the color information) or a \ref MAT_TYPE::F32_C3.
    \param coord_system_src : the current coordinate system of floatMat.
    \param coord_system_dst : the destination coordinate system for floatMat.
    \param mem : define which memory should be transformed from floatMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertCoordinateSystem(Mat &floatMat, COORDINATE_SYSTEM coord_system_src, COORDINATE_SYSTEM coord_system_dst, MEM mem = MEM::CPU);

    /*!
    \brief Change the coordinate system of a transform matrix.
    \param motionMat : (in/out) matrix to transform
    \param coord_system_src : the current coordinate system of motionMat.
    \param coord_system_dst : the destination coordinate system for motionMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertCoordinateSystem(Transform &motionMat, COORDINATE_SYSTEM coord_system_src, COORDINATE_SYSTEM coord_system_dst);

    /*!
    \brief Get the unit factor to change units.
    \param unit_src : the source coordinate system.
    \param unit_dst : the destination coordinate system.
    \return float : unit scale (pt_coord_dst = factor * pt_coord_src).
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ float getUnitScale(UNIT unit_src, UNIT unit_dst);

    /*!
    \brief Change the unit of a matrix.
    \param floatMat : (in/out) matrix to transform, can be either a \ref MAT_TYPE::F32_C4 (the fourth value will be ignored as it contained the color information), \ref MAT_TYPE::F32_C3 or a \ref MAT_TYPE::F32_C1.
    \param unit_src : the current unit of floatMat.
    \param unit_dst : the destination unit for floatMat.
    \param mem : define which memory should be transformed from floatMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertUnit(Mat &floatMat, UNIT unit_src, UNIT unit_dst, MEM mem = MEM::CPU);

    /*!
    \brief Change the unit (of the translations) of a transform matrix.
    \param motionMat : (in/out) matrix to transform
    \param unit_src : the current unit of motionMat.
    \param unit_dst : the destination unit for motionMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE::FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertUnit(Transform &motionMat, UNIT unit_src, UNIT unit_dst);

    /**
    \enum OBJECT_CLASS
    \ingroup Object_group
    \brief Lists available object class
     */
    enum class OBJECT_CLASS {
        PERSON = 0, /**< For people detection */
        VEHICLE = 1, /**< For vehicles detection. It can be cars, trucks, buses, motorcycles etc */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const OBJECT_CLASS &object_class);

    inline std::ostream &operator<<(std::ostream &os, const OBJECT_CLASS &object_class) {
        return os << toString(object_class);
    }
    ///@endcond

    /**
    \enum OBJECT_TRACKING_STATE
    \ingroup Object_group
    \brief Lists available object tracking state
     */
    enum class OBJECT_TRACKING_STATE {
        OFF, /**< The tracking is not yet initialized, the object ID is not usable */
        OK, /**< The object is tracked */
        SEARCHING, /**< The object couldn't be detected in the image and is potentially occluded, the trajectory is estimated */
        TERMINATE, /**< This is the last searching state of the track, the track will be deleted in the next retreiveObject */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const OBJECT_TRACKING_STATE &object_track_state);

    inline std::ostream &operator<<(std::ostream &os, const OBJECT_TRACKING_STATE &object_track_state) {
        return os << toString(object_track_state);
    }
    ///@endcond

    /**
    \enum OBJECT_ACTION_STATE
    \ingroup Object_group
    \brief Lists available object action state
     */
    enum class OBJECT_ACTION_STATE {
        IDLE = 0, /**< The object is staying static. */
        MOVING = 1, /**< The object is moving. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const OBJECT_ACTION_STATE &object_action_state);

    inline std::ostream &operator<<(std::ostream &os, const OBJECT_ACTION_STATE &object_action_state) {
        return os << toString(object_action_state);
    }
    ///@endcond

    /**
    \ingroup Object_group
    \brief Contains data of a detected object such as its \ref bounding_box, \ref label, \ref id and its 3D \ref position.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ObjectData {
    public:
    ObjectData();

    ~ObjectData();

        /**
        \brief Object identification number, used as a reference when tracking the object through the frames
        \note Is set to -1 if the obeject is not currently tracked.
         */
        int id;

        /**
        \brief Object label. Identify the object type
         */
        OBJECT_CLASS label;

        /**
        \brief Defines the object tracking state
         */
        OBJECT_TRACKING_STATE tracking_state;

        /**
        \brief Defines the object action state
         */
        OBJECT_ACTION_STATE action_state;

        /**
        \brief Defines the object 3D centroid in the reference frame selected in \ref RuntimeParameters::measure3D_reference_frame and given to the \ref Camera::grab() function.
         */
        sl::float3 position;

        /**
        \brief Defines the object 3D velocity
         */
        sl::float3 velocity;


        /**
        \brief the covariance matrix of the 3d position, represented by its upper triangular matrix value
        * \code
             = [p0, p1, p2]
               [p1, p3, p4]
               [p2, p4, p5]
          \endcode
          where pi is position_covariance[i]
        */
        float position_covariance[6];
        /**
         * \brief 2D bounding box of the person represented as four 2D points starting at the top left corner and rotation clockwise.
         * \code
             A ------ B
             | Object |
             D ------ C
         \endcode
         */
        std::vector<sl::uint2> bounding_box_2d;

        /**
        \brief Defines for the bounding_box_2d the pixels which really belong to the object (set to 255) and those of the background (set to 0).
         \warning : The mask information is available only for tracked objects that have a valid depth.
         */
        sl::Mat mask;

        /**
        \brief Defines the detection confidence value of the object. 
         * A lower confidence value means the object might not be localized perfectly or the label (OBJECT_CLASS) is uncertain
         */
        float confidence;

        /**
         * \brief 3D bounding box of the person represented as eight 3D points
         * \code
               1 ------ 2
              /        /|
             0 ------ 3 |
             | Object | 6
             |        |/
             4 ------ 7
         \endcode
         \note Only available if ObjectDetectionParameters::enable_tracking is activated
         */
        std::vector<sl::float3> bounding_box;

        /**
         * \brief 3D object dimensions: width, height, length
        \note Only available if ObjectDetectionParameters::enable_tracking is activated
         */
        sl::float3 dimensions;

        /**
         * \brief A set of useful points representing the human body, expressed in 2D, respect to the original image resolution.
         We use a classic 18 points representation, the points semantic and order is given by BODY_PARTS.
          \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
          \warning in some cases, eg. body partially out of the image, some keypoint can not be detected, they will have negatives coordinates.
         */
        std::vector<sl::float2> keypoint_2d;

        /**
         * \brief A set of useful points representing the human body, expressed in 3D.
         We use a classic 18 points representation, the points semantic and order is given by BODY_PARTS.
          \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
          \warning in some cases, eg. body partially out of the image or missing depth data, some keypoint can not be detected, they will have non finite values.
         */
        std::vector<sl::float3> keypoint;

        /**
         * \brief bounds the head with four 2D points.
          \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
         */
        std::vector<sl::uint2> head_bounding_box_2d;

        /**
         * \brief bounds the head with eight 3D points.
          \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
         */
        std::vector<sl::float3> head_bounding_box;

        /**
         * \brief 3D head centroid.
          \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
         */
        sl::float3 head_position;
    };

    /**
    \ingroup Object_group
    \brief Contains the result of the object detection module.
    The detected objects are listed in \ref object_list.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Objects {
    public:
        /**
        \brief Defines the timestamp corresponding to the frame acquisition. 
         * This value is especially useful for the async mode to synchronize the data.
         */
        sl::Timestamp timestamp;

        /**
        \brief The list of detected objects
         */
        std::vector<sl::ObjectData> object_list;

        /**
        \brief Defined if the object list has already been retrieved or not.
         */
        bool is_new;

        /**
        \brief Defined if both the object tracking and the world orientation has been setup.
         */
        bool is_tracked;

        /**
        \brief Function that look for a given object ID in the current object list and return the object associated if found and a status
         \param objectData [out] : the object corresponding to the given ID if found
         \param objectDataId [in] : the input object ID
         * 
         \return True if found False otherwise
         */
        bool getObjectDataFromId(sl::ObjectData &objectData, int objectDataId);
    };

    /**
    \ingroup Object_group
     * \brief semantic and order of human body keypoints.
     */
    enum class BODY_PARTS {
        NOSE = 0,
        NECK = 1,
        RIGHT_SHOULDER = 2,
        RIGHT_ELBOW= 3,
        RIGHT_WRIST = 4,
        LEFT_SHOULDER = 5,
        LEFT_ELBOW = 6,
        LEFT_WRIST = 7,
        RIGHT_HIP = 8,
        RIGHT_KNEE = 9,
        RIGHT_ANKLE = 10,
        LEFT_HIP = 11,
        LEFT_KNEE = 12,
        LEFT_ANKLE = 13,
        RIGHT_EYE = 14,
        LEFT_EYE = 15,
        RIGHT_EAR = 16,
        LEFT_EAR = 17,
        LAST = 18
    };

    /**
    \ingroup Object_group
     * \brief return associated index of each BODY_PART
     */
    inline int getIdx(BODY_PARTS part) {
        return static_cast<int>(part);
    }

    /**
    \ingroup Object_group
     * \brief Links of human body keypoints, usefull for display.
     */
    const std::vector<std::pair< BODY_PARTS, BODY_PARTS>> BODY_BONES{
        {BODY_PARTS::NOSE, BODY_PARTS::NECK},
        {BODY_PARTS::NECK, BODY_PARTS::RIGHT_SHOULDER},
        {BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::RIGHT_ELBOW},
        {BODY_PARTS::RIGHT_ELBOW, BODY_PARTS::RIGHT_WRIST},
        {BODY_PARTS::NECK, BODY_PARTS::LEFT_SHOULDER},
        {BODY_PARTS::LEFT_SHOULDER, BODY_PARTS::LEFT_ELBOW},
        {BODY_PARTS::LEFT_ELBOW, BODY_PARTS::LEFT_WRIST},
        {BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::RIGHT_HIP},
        {BODY_PARTS::RIGHT_HIP, BODY_PARTS::RIGHT_KNEE},
        {BODY_PARTS::RIGHT_KNEE, BODY_PARTS::RIGHT_ANKLE},
        {BODY_PARTS::LEFT_SHOULDER, BODY_PARTS::LEFT_HIP},
        {BODY_PARTS::LEFT_HIP, BODY_PARTS::LEFT_KNEE},
        {BODY_PARTS::LEFT_KNEE, BODY_PARTS::LEFT_ANKLE},
        {BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::LEFT_SHOULDER},
        {BODY_PARTS::RIGHT_HIP, BODY_PARTS::LEFT_HIP},
        {BODY_PARTS::NOSE, BODY_PARTS::RIGHT_EYE},
        {BODY_PARTS::RIGHT_EYE, BODY_PARTS::RIGHT_EAR},
        {BODY_PARTS::NOSE, BODY_PARTS::LEFT_EYE},
        {BODY_PARTS::LEFT_EYE, BODY_PARTS::LEFT_EAR}
    };
}
#endif /* __CORE_HPP__ */
/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 * 
 * 1. Definitions
 * 
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2 or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera, ZED 2 camera and the ZED Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2 or ZED Mini camera.
 * 
 * 2. License
 * 
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * 
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2 or ZED Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2
 *       or ZED Mini camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2 or ZED Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2 or ZED Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2 or ZED Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 * 
 * 3. Warranty
 * 
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2 or ZED Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 * 
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * 
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 * 
 * 5. Choice of Law
 * 
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __MESH_HPP__
#define __MESH_HPP__

#include <vector>


#if defined(_WIN32)
#ifdef CORE_COMPILATION
#define SL_SCANNING_EXPORT __declspec(dllexport)
#else
#define SL_SCANNING_EXPORT
#endif
#elif __GNUC__
#define SL_SCANNING_EXPORT __attribute__((visibility("default")))
#else
#define SL_SCANNING_EXPORT
#endif

namespace sl {

    /**
    \enum MESH_FILE_FORMAT
    \ingroup SpatialMapping_group
    \brief Lists available mesh file formats.
     */
    enum class MESH_FILE_FORMAT {
        PLY, /**< Contains only vertices and faces.*/
        PLY_BIN, /**< Contains only vertices and faces, encoded in binary.*/
        OBJ, /**< Contains vertices, normals, faces and textures informations if possible.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MESH_FILE_FORMAT &mesh_frmt);

    inline std::ostream &operator<<(std::ostream &os, const MESH_FILE_FORMAT &mesh_frmt) {
        return os << toString(mesh_frmt);
    }
    ///@endcond

    /**
    \enum MESH_TEXTURE_FORMAT
    \ingroup SpatialMapping_group
    \brief Lists available mesh texture formats.
     */
    enum class MESH_TEXTURE_FORMAT {
        RGB, /**< The texture has 3 channels.*/
        RGBA, /**< The texture has 4 channels.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MESH_TEXTURE_FORMAT &text_frmt);

    inline std::ostream &operator<<(std::ostream &os, const MESH_TEXTURE_FORMAT &text_frmt) {
        return os << toString(text_frmt);
    }
    ///@endcond

    /**
    \class MeshFilterParameters
    \ingroup SpatialMapping_group
    \brief Defines the behavior of the sl::Mesh::filter function.

    A default constructor is enabled and set to its default parameters.
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ MeshFilterParameters {
    public:

        /**
        \enum FILTER.
        \ingroup SpatialMapping_group
        \brief Lists available mesh filtering intensity.
         */
        enum class MESH_FILTER {
            LOW, /**< Clean the mesh by closing small holes and removing isolated faces.*/
            MEDIUM, /**< Soft decimation and smoothing.*/
            HIGH, /**< Decimate the number of triangles and apply a soft smooth.*/
            LAST
        };

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        MeshFilterParameters(MESH_FILTER mesh_filtering = MESH_FILTER::LOW) {
            set(mesh_filtering);
        }

        /**
        \brief Sets the filtering intensity
        \param filtering_ : the desired \ref FILTER.
         */
        void set(MESH_FILTER mesh_filtering = MESH_FILTER::LOW) {
            filtering = mesh_filtering;
        }

        MESH_FILTER filtering = MESH_FILTER::LOW;

        /**
        \brief Saves the current bunch of parameters into a file.
        \param filename : the path to the file in which the parameters will be stored.
        \return true if the file was successfully saved, otherwise false.
         */
        bool save(String filename);

        /**
        \brief Loads the values of the parameters contained in a file.
        \param filename : the path to the file from which the parameters will be loaded.
        \return true if the file was successfully loaded, otherwise false.
         */
        bool load(String filename);
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MeshFilterParameters::MESH_FILTER &mesh_filter);

    inline std::ostream &operator<<(std::ostream &os, const MeshFilterParameters::MESH_FILTER &mesh_filter) {
        return os << toString(mesh_filter);
    }
    ///@endcond

    /*@cond SHOWHIDDEN*/
    enum class MESH_CREATION {
        NONE,
        LIVE,
        LOAD
    };

    class SL_CORE_EXPORT TextureImagePool {
    public:
        TextureImagePool();
        ~TextureImagePool();

        inline bool isEmpty() {
            return v.size() == 0;
        }

        std::vector<std::pair<sl::Mat *, sl::Transform>> v;
        void clear();
        void init(sl::Resolution res);
        void emplace_back();

    private:
        sl::Resolution resolution;
    };
    /*@endcond*/

    /**
    \class Chunk
    \ingroup SpatialMapping_group
    \brief Represents a sub-mesh, it contains local vertices and triangles.

    Vertices and normals have the same size and are linked by id stored in triangles.

    \note uv contains data only if your mesh have textures (by loading it or after calling \ref applyTexture).
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ Chunk {
    public:
        /**
        \brief Default constructor which creates an empty \ref Chunk.
         */
        Chunk();

        /**
        \brief \ref Chunk destructor.
         */
        ~Chunk();

        /**
        Vertices are defined by a 3D point {x,y,z}.
         */
        std::vector<float3> vertices;

        /**
        Triangles (or faces) contains the index of its three vertices. It corresponds to the 3 vertices of the triangle {v1, v2, v3}.
         */
        std::vector<uint3> triangles;

        /**
        Normals are defined by three components, {nx, ny, nz}. Normals are defined for each vertices.
         */
        std::vector<float3> normals;

        /**
        UVs defines the 2D projection of each vertices onto the Texture.
        \n Values are normalized [0;1], starting from the bottom left corner of the texture (as requested by opengl).
        \n In order to display a textured mesh you need to bind the Texture and then draw each triangles by picking its uv values.

        \note Contains data only if your mesh have textures (by loading it or calling applytexture).
         */
        std::vector<float2> uv;

        /**
        Timestamp of the latest update.
         */
        unsigned long long timestamp;

        /**
        3D centroid of the chunk.
         */
        float3 barycenter;

        /**
        True if the chunk has been updated by an inner process.
         */
        bool has_been_updated;

        /**
        \brief Clears all chunk data.
         */
        void clear();
    };

    /**
    \class Mesh
    \ingroup SpatialMapping_group
    \brief A mesh contains the geometric (and optionally texture) data of the scene captured by spatial mapping.

    By default the mesh is defined as a set of chunks, this way we update only the data that has to be updated avoiding a time consuming remapping process every time a small part of the Mesh is updated.
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ Mesh {
        ///@cond
        friend class SpatialMappingHandler;
        ///@endcond

    public:
        typedef std::vector<size_t> chunkList;

        /**
        \brief Default constructor which creates an empty Mesh.
         */
        Mesh();

        /**
        \brief Mesh destructor.
         */
        ~Mesh();

        /**
        contains the list of chunks
         */
        std::vector<Chunk> chunks;

        /**
        Vertices are defined by a 3D point {x,y,z}.
         */
        std::vector<float3> vertices;

        /**
        Triangles (or faces) contains the index of its three vertices. It corresponds to the 3 vertices of the triangle {v1, v2, v3}.
         */
        std::vector<uint3> triangles;

        /**
        Normals are defined by three components, {nx, ny, nz}. Normals are defined for each vertices.
         */
        std::vector<float3> normals;

        /**
        UVs defines the 2D projection of each vertices onto the Texture.
        \n Values are normalized [0;1], starting from the bottom left corner of the texture (as requested by opengl).
        \n In order to display a textured mesh you need to bind the Texture and then draw each triangles by picking its uv values.

        \note Contains data only if your mesh have textures (by loading it or calling \ref applyTexture).
         */
        std::vector<float2> uv;

        /**
        Texture of the Mesh.

        \note Contains data only if your mesh have textures (by loading it or calling \ref applyTexture).
         */
        Mat texture;

        /**
        \brief Defines the [] operator to directly access the desired chunk.
         */
        Chunk &operator[](int index);

        /**
        \brief Computes the total number of triangles stored in all chunks.
        \return The number of triangles stored in all chunks.
         */
        size_t getNumberOfTriangles();

        /**
        \brief Updates \ref vertices / \ref normals / \ref triangles / \ref uv from chunks' data pointed by the given chunkList.
        \param IDs : the index of chunks which will be concatenated. default : (empty).

        \note If the given chunkList is empty, all chunks will be used to update the current Mesh.
         */
        void updateMeshFromChunkList(chunkList IDs = chunkList());

        /**
        \brief Computes the list of visible chunk from a specific point of view.
        \param world_reference_pose : the point of view, given in world reference.
        \return The list of visible chunks.
         */
        chunkList getVisibleList(Transform camera_pose);

        /**
        \brief Computes the list of chunks which are close to a specific point of view.
        \param world_reference_position : the point of view, given in world reference.
        \param radius : the radius in defined \ref UNIT.
        \return The list of chunks close to the given point.
         */
        chunkList getSurroundingList(Transform camera_pose, float radius);

        /**
        \brief Filters the mesh.

        The resulting mesh in smoothed, small holes are filled and small blobs of non connected triangles are deleted.

        \param mesh_filter_params : defines the filtering parameters, for more info checkout the \ref MeshFilterParameters documentation. default : preset.
        \param update_chunk_only : if set to false the mesh data (vertices/normals/triangles) are updated otherwise only the chunk's data are updated. default : false.
        \return True if the filtering was successful, false otherwise.

        \note The filtering is a costly operation, its not recommended to call it every time you retrieve a mesh but at the end of your spatial mapping process.
         */
        bool filter(MeshFilterParameters mesh_filter_params = MeshFilterParameters(), bool update_chunk_only = false);

        /**
        \brief Applies texture to the mesh.

        By using this function you will get access to \ref uv, and \ref texture.
        The number of triangles in the mesh may slightly differ before and after calling this functions due to missing texture information.
        There is only one texture for the mesh, the uv of each chunks are expressed for it in its entirety.
        Vectors of vertices/normals and uv have now the same size.

        \param texture_format : define the number of channels desired for the computed texture. default : MESH_TEXTURE_FORMAT::RGB.
        \return True if the texturing was successful, false otherwise.

        \note This function can be called as long as you do not start a new spatial mapping process, due to shared memory.
        \note This function can require a lot of computation time depending on the number of triangles in the mesh. Its recommended to call it once a the end of your spatial mapping process.

        \warning The save_texture parameter in SpatialMappingParameters must be set as true when enabling the spatial mapping to be able to apply the textures.
        \warning The mesh should be filtered before calling this function since \ref filter will erase the textures, the texturing is also significantly slower on non-filtered meshes.
         */
        bool applyTexture(MESH_TEXTURE_FORMAT texture_format = MESH_TEXTURE_FORMAT::RGB);

        /**
        \brief Merges currents chunks.

        This can be used to merge chunks into bigger sets to improve rendering process.

        \param faces_per_chunk : define the new number of faces per chunk (useful for Unity that doesn't handle chunks over 65K vertices).

        \warning You should not use this function during spatial mapping process because mesh updates will revert this changes.
         */
        void mergeChunks(int faces_per_chunk);

        /**
        \brief Estimates the gravity vector.

        This function looks for a dominant plane in the whole mesh considering that it is the floor (or a horizontal plane).
        This can be used to find the gravity and then create realistic physical interactions.

        \return The gravity vector.
         */
        sl::float3 getGravityEstimate();

        /**
        \brief Compute the indices of boundaries vertices.

        \return The indices of boundaries vertices.
         */
        std::vector<int> getBoundaries();

        /**
        \brief Saves the current Mesh into a file.
        \param filename : the path and filename of the mesh.
        \param type : defines the file type (extension). default : MESH_FILE_OBJ.
        \param IDs : (by default empty) Specify a set of chunks to be saved, if none provided all chunks are saved. default : (empty).
        \return True if the file was successfully saved, false otherwise.

        \note Only \ref MESH_FILE_OBJ support textures data.
        \note This function operates on the Mesh not on the chunks. This way you can save different parts of your Mesh (update your Mesh with \ref updateMeshFromChunkList).
         */
        bool save(String filename, MESH_FILE_FORMAT type = MESH_FILE_FORMAT::OBJ, chunkList IDs = chunkList());

        /**
        \brief Loads the mesh from a file.
        \param filename : the path and filename of the mesh (do not forget the extension).
        \param update_chunk_only : if set to false the mesh data (vertices/normals/triangles) are updated otherwise only the chunk's data are updated. default : false.
        \return True if the loading was successful, false otherwise.

        \note Updating the Mesh is time consuming, consider using only Chunks for better performances.
         */
        bool load(String filename, bool update_chunk_only = false);

        /**
        \brief Clears all the data.
         */
        void clear();

    private:
        void init();

        std::shared_ptr<TextureImagePool> p_im_pool;
        sl::CameraParameters cam_param;
        float min_d, max_d;
        size_t memory;
        bool face_order;
        int id_mesher; /** < used to clear the mesh based on the number of the current mesher, if different clear it */
        MESH_CREATION state;
    };

    /**
    \enum PLANE_TYPE
    \brief List available plane type detected from the orientation
     */
    enum class PLANE_TYPE {
        HORIZONTAL,
        VERTICAL,
        UNKNOWN,
        LAST
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const PLANE_TYPE &type);

    inline std::ostream &operator<<(std::ostream &os, const PLANE_TYPE &type) {
        return os << toString(type);
    }
    ///@endcond

    /**
    \class Plane
    \brief A plane defined by a point and a normal, or a plane equation
     * Other elements can be extracted such as the mesh, the 3D bounds...
     \note The plane measurement are expressed in REFERENCE_FRAME defined by the RuntimeParameters measure3D_reference_frame
     */

    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ Plane {
        ///@cond SHOWHIDDEN
        friend class PlaneCandidate;
        friend class PlaneDetectorHandler;
        ///@endcond
    public:

        Plane();
        ~Plane();

        void clear();

        /**
         * The plane type define the plane orientation : vertical or horizontal.
         \note It is deduced from the gravity vector and is therefore only available with the ZED-M.
          The ZED will give PLANE_TYPE::UNKNOWN for every planes.
         */
        PLANE_TYPE type = PLANE_TYPE::UNKNOWN;


        /**
        \brief Get the plane normal vector
        \return Plane normal vector, with normalized components
         */
        sl::float3 getNormal();

        /**
        \brief Get the plane center point
        \return Plane center point
         */
        sl::float3 getCenter();

        /**
        \brief Get the plane pose relative to the global reference frame
        \return A transformation matrix (rotation and translation) which give the plane pose.
         * Can be used to transform the global reference frame center (0,0,0) to the plane center
         */
        Transform getPose();

        /**
        \brief Get the width and height of the bounding rectangle around the plane contours
        \return Width and height of the bounding plane contours

        \warning This value is expressed in the plane reference frame
         */
        sl::float2 getExtents();

        /**
        \brief Get the plane equation
        \return Plane equation, in the form : ax+by+cz=d, the returned values are (a,b,c,d)
         */
        sl::float4 getPlaneEquation();

        /**
        \brief Get the polygon bounds of the plane
        \return Vector of 3D points forming a polygon bounds corresponding to the current visible limits of the plane
         */
        std::vector<sl::float3> getBounds();

        /**
        \brief Compute and return the mesh of the bounds polygon
        \return A mesh representing the plane delimited by the visible bounds
         */
        sl::Mesh extractMesh();

        /**
        \brief Get the distance between the input point and the projected point alongside the normal vector onto the plane.
         * This corresponds to the closest point on the plane.
        \param The point to project into the plane
        \return The Euclidean distance between the input point and the projected point
         */
        float getClosestDistance(sl::float3 point = sl::float3(0, 0, 0));

    private:
        std::vector<sl::float3> bounds3D_RuCu;
        std::vector<sl::float2> bounds2D;
        sl::float2 sizeRect;
        sl::Transform planePose_RuCu;

        sl::float4 e_RcCi;
        sl::Transform Rt_RcCiRuCi;
        sl::Transform pose_RcCi;

        sl::float4 planeEquation_RuCu;
        sl::Mesh mesh_RuCu;

        float coef, unit_factor;
        sl::Transform transform_Ci2Cu;
        bool isFloor;
        void computeBoundingRect(std::vector<sl::uint2> &bounds2D_RcCi, sl::float4 param);
    };

    /**
    \class PointCloudChunk
    \ingroup SpatialMapping_group
    \brief Represents a sub fused point cloud, it contains local vertices and colors.

    Vertices and normals have the same size.
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ PointCloudChunk {
    public:
        /**
        \brief Default constructor which creates an empty \ref PointCloudChunk.
         */
        PointCloudChunk();

        /**
        \brief \ref PointCloudChunk destructor.
         */
        ~PointCloudChunk();

        /**
        Vertices are defined by a colored 3D point {x, y, z, rgba}.
         */
        std::vector<float4> vertices;

        /**
        Normals are defined by three components, {nx, ny, nz}.
         */
        std::vector<float3> normals;

        /**
        Timestamp of the latest update.
         */
        unsigned long long timestamp;

        /**
        3D centroid of the chunk.
         */
        float3 barycenter;

        /**
        True if the chunk has been updated by an inner process.
         */
        bool has_been_updated;

        /**
        \brief Clears all chunk data.
         */
        void clear();
    };

    /**
    \class FusedPointCloud
    \ingroup SpatialMapping_group
    \brief A fused point cloud contains both geometric and color data of the scene captured by spatial mapping.

    By default the fused point cloud is defined as a set of point cloud chunks, this way we update only the required data, avoiding a time consuming remapping process every time a small part of the fused point cloud is changed.
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ FusedPointCloud {
        ///@cond
        friend class SpatialMappingHandler;
        ///@endcond

    public:
        typedef std::vector<size_t> chunkList;

        /**
        \brief Default constructor which creates an empty FusedPointCloud.
         */
        FusedPointCloud();

        /**
        \brief FusedPointCloud destructor.
         */
        ~FusedPointCloud();

        /**
        contains the list of chunks
         */
        std::vector<PointCloudChunk> chunks;

        /**
        Vertices are defined by colored 3D points {x, y, z, rgba}.
         */
        std::vector<float4> vertices;

        /**
        Normals are defined by three components, {nx, ny, nz}. Normals are defined for each vertices.
         */
        std::vector<float3> normals;

        /**
        \brief Defines the [] operator to directly access the desired chunk.
         */
        PointCloudChunk &operator[](int index);

        /**
        \brief Computes the total number of triangles stored in all chunks.
        \return The number of points stored in all chunks.
         */
        size_t getNumberOfPoints();

        /**
        \brief Updates \ref vertices / \ref normals / \ref colors from chunks' data pointed by the given chunkList.
        \param IDs : the index of chunks which will be concatenated. default : (empty).

        \note If the given chunkList is empty, all chunks will be used.
         */
        void updateFromChunkList(chunkList IDs = chunkList());

        /**
        \brief Saves the current fused point cloud into a file.
        \param filename : the path and filename of the mesh.
        \param type : defines the file type (extension). default : MESH_FILE_OBJ.
        \param IDs : (by default empty) Specify a set of chunks to be saved, if none provided all chunks are saved. default : (empty).
        \return True if the file was successfully saved, false otherwise.

        \note Only \ref MESH_FILE_OBJ support textures data.
        \note This function operates on the fused point cloud not on the chunks. This way you can save different parts of your fused point cloud (update with \ref updateFromChunkList).
         */
        bool save(String filename, MESH_FILE_FORMAT type = MESH_FILE_FORMAT::OBJ, chunkList IDs = chunkList());

        /**
        \brief Loads the fused point cloud from a file.
        \param filename : the path and filename of the fused point cloud (do not forget the extension).
        \param update_chunk_only : if set to false the fused point cloud data (vertices/normals) are updated otherwise only the chunk's data are updated. default : false.
        \return True if the loading was successful, false otherwise.

        \note Updating the fused point cloud is time consuming, consider using only chunks for better performances.
         */
        bool load(String filename, bool update_chunk_only = false);

        /**
        \brief Clears all the data.
         */
        void clear();

    private:
        void init();

        sl::CameraParameters cam_param;
        float min_d, max_d;
        size_t memory;
        int id_mesher; /** < used to clear the mesh based on the number of the current mesher, if different clear it */
        MESH_CREATION state;
    };


}

#endif /* MESH_HPP_ */
/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 * 
 * 1. Definitions
 * 
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2 or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera, ZED 2 camera and the ZED Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2 or ZED Mini camera.
 * 
 * 2. License
 * 
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * 
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2 or ZED Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2
 *       or ZED Mini camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2 or ZED Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2 or ZED Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2 or ZED Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 * 
 * 3. Warranty
 * 
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2 or ZED Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 * 
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * 
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 * 
 * 5. Choice of Law
 * 
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__


#if defined _WIN32
#if defined(SL_SDK_COMPIL)
#define SL_SDK_EXPORT __declspec(dllexport)
#else
#define SL_SDK_EXPORT __declspec(dllimport)
#endif
#elif __GNUC__
#define SL_SDK_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

// SDK VERSION NUMBER
#define ZED_SDK_MAJOR_VERSION 3
#define ZED_SDK_MINOR_VERSION 2
#define ZED_SDK_PATCH_VERSION 2
#define ZED_SDK_BUILD_ID "20566_4b0928a4"

#define ZED_SDK_VERSION_ATTRIBUTE private: uint32_t _zed_sdk_major_version = ZED_SDK_MAJOR_VERSION, _zed_sdk_minor_version = ZED_SDK_MINOR_VERSION, _zed_sdk_patch_version = ZED_SDK_PATCH_VERSION;

/**
 * \ingroup Core_group
 * \brief Dynamic version verification: Returns the ZED SDK version currently installed on the computer.
 * The major, minor, patch parameters will be filled by reference.
 * \return -1 if the SDK wasn't found, -2 if the version wasn't found, 0 if success.
 */
int /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ getZEDSDKRuntimeVersion(int &major, int& minor, int& patch);

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

    /// \defgroup Video_group Video Module
    /// \defgroup Depth_group Depth Sensing Module
    /// \defgroup Core_group Core Module
    /// \defgroup SpatialMapping_group Spatial Mapping Module
    /// \defgroup PositionalTracking_group Positional Tracking Module
    /// \defgroup Object_group Object Detection Module
    /// \defgroup Sensors_group Sensors Module

    /**
    \enum SIDE
    \ingroup Video_group
    \ingroup Enumerations
    \brief defines left,right,both to distinguish between left and right or both sides
     */
    enum class SIDE {
        LEFT, /**< Left side only.*/
        RIGHT, /**< Right side only.*/
        BOTH /**< Left and Right side.*/
    };

    /**
    \enum RESOLUTION
    \ingroup Video_group
    \ingroup Enumerations
    \brief Represents the available resolution defined in the \ref cameraResolution list.
    \note The VGA resolution does respect the 640*480 standard to better fit the camera sensor (672*376 is used).
     */
    enum class RESOLUTION {
        HD2K, /**< 2208*1242, available framerates: 15 fps.*/
        HD1080, /**< 1920*1080, available framerates: 15, 30 fps.*/
        HD720, /**< 1280*720, available framerates: 15, 30, 60 fps.*/
        VGA, /**< 672*376, available framerates: 15, 30, 60, 100 fps.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const RESOLUTION &resolution);

    inline ::std::ostream &operator<<(::std::ostream &os, const RESOLUTION &resolution) {
        return os << toString(resolution);
    }
    ///@endcond

    /**
    \enum VIDEO_SETTINGS
    \ingroup Video_group
    \brief Lists available camera settings for the ZED camera (contrast, hue, saturation, gain...).
    \warning GAIN and EXPOSURE are linked in auto/default mode (see \ref sl::Camera::setCameraSettings).
    \brief Each enum defines one of those settings.
     */
    enum class VIDEO_SETTINGS {
        BRIGHTNESS, /**< Defines the brightness control. Affected value should be between 0 and 8.*/
        CONTRAST, /**< Defines the contrast control. Affected value should be between 0 and 8.*/
        HUE, /**< Defines the hue control. Affected value should be between 0 and 11.*/
        SATURATION, /**< Defines the saturation control. Affected value should be between 0 and 8.*/
        SHARPNESS, /**< Defines the digital sharpening control. Affected value should be between 0 and 8.*/
        GAMMA, /** < Defines the ISP gamma control. Affected value should be between 1 and 9.*/
        GAIN, /**< Defines the gain control. Affected value should be between 0 and 100 for manual control.*/
        EXPOSURE, /**< Defines the exposure control. Affected value should be between 0 and 100 for manual control.\n The exposition is mapped linearly in a percentage of the following max values. Special case for the setExposure(0) that corresponds to 0.17072ms.\n The conversion to milliseconds depends on the framerate: <ul><li>15fps setExposure(100) -> 19.97ms</li><li>30fps setExposure(100) -> 19.97ms</li><li>60fps setExposure(100) -> 10.84072ms</li><li>100fps setExposure(100) -> 10.106624ms</li></ul>*/
        AEC_AGC, /**< Defines if the Gain and Exposure are in automatic mode or not. Setting a Gain or Exposure through @GAIN or @EXPOSURE values will automatically set this value to 0.*/
        AEC_AGC_ROI, /**< Defines the region of interest for automatic exposure/gain computation. To be used with overloaded @setCameraSettings/@getCameraSettings functions.*/
        WHITEBALANCE_TEMPERATURE, /**< Defines the color temperature value. Setting a value will automatically set @WHITEBALANCE_AUTO to 0. Affected value should be between 2800 and 6500 with a step of 100.*/
        WHITEBALANCE_AUTO, /**< Defines if the White balance is in automatic mode or not*/
        LED_STATUS, /**< Defines the status of the camera front LED. Set to 0 to disable the light, 1 to enable the light. Default value is on. Requires Camera FW 1523 at least.*/
        LAST
    };

    const int VIDEO_SETTINGS_VALUE_AUTO = -1;

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const VIDEO_SETTINGS &camSettings);

    inline ::std::ostream &operator<<(::std::ostream &os, const VIDEO_SETTINGS &camSettings) {
        return os << toString(camSettings);
    }
    ///@endcond

    /**
    \enum DEPTH_MODE
    \ingroup Depth_group
    \brief Lists available depth computation modes.
     */
    enum class DEPTH_MODE {
        NONE, /**< This mode does not compute any depth map. Only rectified stereo images will be available.*/
        PERFORMANCE, /**< Computation mode optimized for speed.*/
        QUALITY, /**< Computation mode designed for challenging areas with untextured surfaces.*/
        ULTRA, /**< Computation mode favorising edges and sharpness. Requires more GPU memory and computation power.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const DEPTH_MODE &depthMode);

    inline ::std::ostream &operator<<(::std::ostream &os, const DEPTH_MODE &depthMode) {
        return os << toString(depthMode);
    }
    ///@endcond

    /**
    \enum SENSING_MODE
    \ingroup Depth_group
    \brief Lists available depth sensing modes.
     */
    enum class SENSING_MODE {
        STANDARD, /**< This mode outputs ZED standard depth map that preserves edges and depth accuracy.
                               * Applications example: Obstacle detection, Automated navigation, People detection, 3D reconstruction, measurements.*/
        FILL, /**< This mode outputs a smooth and fully dense depth map.
                           * Applications example: AR/VR, Mixed-reality capture, Image post-processing.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const SENSING_MODE &sensingMode);

    inline ::std::ostream &operator<<(::std::ostream &os, const SENSING_MODE &sensingMode) {
        return os << toString(sensingMode);
    }
    ///@endcond

    /**
    \enum MEASURE
    \ingroup Depth_group
    \brief Lists retrievable measures.
     */
    enum class MEASURE {
        DISPARITY, /**< Disparity map. Each pixel contains 1 float. sl::MAT_TYPE::F32_C1.*/
        DEPTH, /**< Depth map. Each pixel contains 1 float. sl::MAT_TYPE::F32_C1.*/
        CONFIDENCE, /**< Certainty/confidence of the depth map. Each pixel contains 1 float. sl::MAT_TYPE::F32_C1.*/
        XYZ, /**< Point cloud. Each pixel contains 4 float (X, Y, Z, not used). sl::MAT_TYPE::F32_C4.*/
        XYZRGBA, /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color.  sl::MAT_TYPE::F32_C4.*/
        XYZBGRA, /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color.  sl::MAT_TYPE::F32_C4.*/
        XYZARGB, /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color.  sl::MAT_TYPE::F32_C4.*/
        XYZABGR, /**< Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color.  sl::MAT_TYPE::F32_C4.*/
        NORMALS, /**< Normals vector. Each pixel contains 4 float (X, Y, Z, 0).  sl::MAT_TYPE::F32_C4.*/
        DISPARITY_RIGHT, /**< Disparity map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE::F32_C1.*/
        DEPTH_RIGHT, /**< Depth map for right sensor. Each pixel contains 1 float. sl::MAT_TYPE::F32_C1.*/
        XYZ_RIGHT, /**< Point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, not used). sl::MAT_TYPE::F32_C4.*/
        XYZRGBA_RIGHT, /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color. sl::MAT_TYPE::F32_C4.*/
        XYZBGRA_RIGHT, /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color. sl::MAT_TYPE::F32_C4.*/
        XYZARGB_RIGHT, /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color. sl::MAT_TYPE::F32_C4.*/
        XYZABGR_RIGHT, /**< Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color. sl::MAT_TYPE::F32_C4.*/
        NORMALS_RIGHT, /**< Normals vector for right view. Each pixel contains 4 float (X, Y, Z, 0).  sl::MAT_TYPE::F32_C4.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const MEASURE &measure);

    inline ::std::ostream &operator<<(::std::ostream &os, const MEASURE &measure) {
        return os << toString(measure);
    }
    ///@endcond

    /**
    \enum VIEW
    \ingroup Video_group
    \brief Lists available views.
     */
    enum class VIEW {
        LEFT, /**< Left RGBA image. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4.  */
        RIGHT, /**< Right RGBA image. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        LEFT_GRAY, /**< Left GRAY image. Each pixel contains 1 usigned char. sl::MAT_TYPE::U8_C1. */
        RIGHT_GRAY, /**< Right GRAY image. Each pixel contains 1 usigned char. sl::MAT_TYPE::U8_C1. */
        LEFT_UNRECTIFIED, /**< Left RGBA unrectified image. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        RIGHT_UNRECTIFIED, /**< Right RGBA unrectified image. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        LEFT_UNRECTIFIED_GRAY, /**< Left GRAY unrectified image. Each pixel contains 1 usigned char. sl::MAT_TYPE::U8_C1. */
        RIGHT_UNRECTIFIED_GRAY, /**< Right GRAY unrectified image. Each pixel contains 1 usigned char. sl::MAT_TYPE::U8_C1. */
        SIDE_BY_SIDE, /**< Left and right image (the image width is therefore doubled). Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        DEPTH, /**< Color rendering of the depth. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. Use \ref MEASURE "MEASURE::DEPTH" with \ref Camera.retrieveMeasure() to get depth values.*/
        CONFIDENCE, /**< Color rendering of the depth confidence. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        NORMALS, /**< Color rendering of the normals. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        DEPTH_RIGHT, /**< Color rendering of the right depth mapped on right sensor, sl::MAT_TYPE::U8_C4. */
        NORMALS_RIGHT, /**< Color rendering of the normals mapped on right sensor. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE::U8_C4. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const VIEW &view);

    inline ::std::ostream &operator<<(::std::ostream &os, const VIEW &view) {
        return os << toString(view);
    }
    ///@endcond

    /**
    \enum TIME_REFERENCE
    \ingroup Video_group
    \brief Lists specific and particular timestamps
     */
    enum class TIME_REFERENCE {
        IMAGE, /**< Defines the timestamp at the time the frame has been extracted from USB stream. */
        CURRENT, /**<  Defines the timestamp at the time of the function call. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const TIME_REFERENCE &time_reference);

    inline ::std::ostream &operator<<(::std::ostream &os, const TIME_REFERENCE &time_reference) {
        return os << toString(time_reference);
    }
    ///@endcond

    /**
    \enum POSITIONAL_TRACKING_STATE
    \ingroup PositionalTracking_group
    \brief Lists the different states of positional tracking.
     */
    enum class POSITIONAL_TRACKING_STATE {
        SEARCHING, /**< The camera is searching for a previously known position to locate itself.*/
        OK, /**< Positional tracking is working normally.*/
        OFF, /**< Positional tracking is not enabled.*/
        FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720))*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const POSITIONAL_TRACKING_STATE &track_state);

    inline ::std::ostream &operator<<(::std::ostream &os, const POSITIONAL_TRACKING_STATE &track_state) {
        return os << toString(track_state);
    }
    ///@endcond

    /**
    \enum AREA_EXPORTING_STATE
    \ingroup SpatialMapping_group
    \brief Lists the different states of spatial memory area export.
     */
    enum class AREA_EXPORTING_STATE {
        SUCCESS, /**< The spatial memory file has been successfully created.*/
        RUNNING, /**< The spatial memory is currently written.*/
        NOT_STARTED, /**< The spatial memory file exportation has not been called.*/
        FILE_EMPTY, /**< The spatial memory contains no data, the file is empty.*/
        FILE_ERROR, /**< The spatial memory file has not been written because of a wrong file name.*/
        SPATIAL_MEMORY_DISABLED, /**< The spatial memory learning is disable, no file can be created.*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const AREA_EXPORTING_STATE &area_export);

    inline ::std::ostream &operator<<(::std::ostream &os, const AREA_EXPORTING_STATE &area_export) {
        return os << toString(area_export);
    }
    ///@endcond

    /**
    \enum REFERENCE_FRAME
    \ingroup PositionalTracking_group
    \brief Defines which type of position matrix is used to store camera path and pose.
     */
    enum class REFERENCE_FRAME {
        WORLD, /**< The transform of sl::Pose will contains the motion with reference to the world frame (previously called PATH).*/
        CAMERA, /**< The transform of sl::Pose will contains the motion with reference to the previous camera frame (previously called POSE).*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const REFERENCE_FRAME &ref_frame);

    inline ::std::ostream &operator<<(::std::ostream &os, const REFERENCE_FRAME &ref_frame) {
        return os << toString(ref_frame);
    }
    ///@endcond

    /**
    \ingroup SpatialMapping_group
    \brief Gives the spatial mapping state.
     */
    enum class SPATIAL_MAPPING_STATE {
        INITIALIZING, /**< The spatial mapping is initializing.*/
        OK, /**< The depth and tracking data were correctly integrated in the fusion algorithm.*/
        NOT_ENOUGH_MEMORY, /**< The maximum memory dedicated to the scanning has been reach, the mesh will no longer be updated.*/
        NOT_ENABLED, /**< Camera::enableSpatialMapping() wasn't called (or the scanning was stopped and not relaunched).*/
        FPS_TOO_LOW, /**< Effective FPS is too low to give proper results for spatial mapping. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720), spatial mapping low resolution)*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_SDK_EXPORT toString(const SPATIAL_MAPPING_STATE &mapping_state);

    inline ::std::ostream &operator<<(::std::ostream &os, const SPATIAL_MAPPING_STATE &mapping_state) {
        return os << toString(mapping_state);
    }
    ///@endcond

    /**
    \ingroup Video_group
    \brief Lists available compression modes for SVO recording.
    \brief sl::SVO_COMPRESSION_MODE::LOSSLESS is an improvement of previous lossless compression (used in ZED Explorer), even if size may be bigger, compression time is much faster.
     */
    enum class SVO_COMPRESSION_MODE {
        LOSSLESS, /**< PNG/ZSTD (lossless) CPU based compression : avg size = 42% (of RAW).*/
        H264, /**< H264(AVCHD) GPU based compression : avg size = 1% (of RAW). Requires a NVIDIA GPU*/
        H265, /**< H265(HEVC) GPU based compression: avg size = 1% (of RAW). Requires a NVIDIA GPU, Pascal architecture or newer*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SVO_COMPRESSION_MODE &svo_compression);

    inline ::std::ostream &operator<<(::std::ostream &os, const SVO_COMPRESSION_MODE &svo_compression) {
        return os << toString(svo_compression);
    }
    ///@endcond

    /**
    \ingroup Video_group
    \brief Recording structure that contains information about SVO.
     */
    struct RecordingStatus {

        RecordingStatus() {
            is_recording = false;
            is_paused = false;
            status = false;
            current_compression_time = 0;
            current_compression_ratio = 0;
            average_compression_time = 0;
            average_compression_ratio = 0;
        }

        bool is_recording; /**< Recorder status, true if enabled */
        bool is_paused; /**< Recorder status, true if the pause is enabled */
        bool status; /**< Status of current frame. True for success or false if the frame couldn't be written in the SVO file.*/
        double current_compression_time; /**< Compression time for the current frame in ms.*/
        double current_compression_ratio; /**< Compression ratio (% of raw size) for the current frame.*/
        double average_compression_time; /**< Average compression time in ms since beginning of recording.*/
        double average_compression_ratio; /**< Average compression ratio (% of raw size) since beginning of recording.*/
    };

    /**
    \ingroup Video_group
    \brief Lists available compression modes for SVO recording.
     */
    enum  FLIP_MODE : int {
        OFF = 0, /**<  default behavior.*/
        ON = 1, /**< Images and camera sensors data are flipped, useful when your camera is mounted upside down.*/
        AUTO = 2, /**< in live mode: use the camera orientation (if an IMU is available) to set the flip mode, in SVO mode, read the state of this enum when recorded*/
        LAST
    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const FLIP_MODE& flip_mode);

    inline ::std::ostream& operator<<(::std::ostream& os, const FLIP_MODE& flip_mode) {
        return os << toString(flip_mode);
    }
    ///@endcond
};

#endif /*__DEFINES_HPP__*/
/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 * 
 * 1. Definitions
 * 
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2 or ZED Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED camera, ZED 2 camera and the ZED Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2 or ZED Mini camera.
 * 
 * 2. License
 * 
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 * 
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2 or ZED Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2
 *       or ZED Mini camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2 or ZED Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2 or ZED Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2 or ZED Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 * 
 * 3. Warranty
 * 
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2 or ZED Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 * 
 * 4. EXCLUSION OF CERTAIN DAMAGES
 * 
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 * 
 * 5. Choice of Law
 * 
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <cuda.h>

// Stereolabs namespace
namespace sl {

    ///@cond
    class SL_SDK_EXPORT Camera;
    ///@endcond

    /**
    \class InitParameters
    \ingroup Video_group
    \brief Holds the options used to initialize the \ref Camera object. \n
    Once passed to the \ref Camera::open() function, these settings will be set for the entire execution life time of the \ref Camera. \n
    You can get further information in the detailed description bellow.\n

    This structure allows you to select multiple parameters for the \ref Camera such as the selected camera, its resolution, depth mode, coordinate system, and unit, of measurement.
    Once filled with the desired options, it should be passed to the \ref Camera::open function.

    \code
    #include <sl/Camera.hpp>
    using namespace sl;
    int main(int argc, char **argv) {
        Camera zed; // Create a ZED camera object

        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = false; // Disable verbose mode
        init_params.camera_resolution = RESOLUTION::HD1080; // Use HD1080 video mode
        init_params.camera_fps = 30; // Set fps at 30
        // Other parameters are left to their default values

        // Open the camera
        ERROR_CODE err = zed.open(init_params);
        if (err != SUCCESS)
            exit(-1);

        // Close the camera
        zed.close();
        return 0;
    }
    \endcode
    With its default values, it opens the ZED camera in live mode at \ref RESOLUTION "RESOLUTION::HD720" and sets the depth mode to \ref DEPTH_MODE "DEPTH_MODE::PERFORMANCE".\n
    You can customize it to fit your application.
    The parameters can also be saved and reloaded using its \ref save() and \ref load() functions.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ InitParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        Define the chosen camera resolution. Small resolutions offer higher framerate and lower computation time.\n
        In most situations, the \ref RESOLUTION "RESOLUTION::HD720" at 60 fps is the best balance between image quality and framerate.\n
        Available resolutions are listed here: \ref RESOLUTION.
        \n default : \ref RESOLUTION "RESOLUTION::HD720"
         */
        RESOLUTION camera_resolution;

        /**
        Requested camera frame rate. If set to 0, the highest FPS of the specified \ref camera_resolution will be used.\n
        See \ref RESOLUTION for a list of supported framerates.
        \n default : 0
        \note If the requested camera_fps is unsupported, the closest available FPS will be used.
         */
        int camera_fps;

        /**
        If you are using the camera upside down, setting this parameter to true will cancel its rotation. The images will be horizontally flipped.
        \n default : FLIP_MODE::AUTO
         * From ZED SDK 3.2 a new FLIP_MODE enum was introduced to add the automatic flip mode detection based on the IMU gravity detection.
         */
        int camera_image_flip;

        /**
        At initialization, the \ref Camera runs a self-calibration process that corrects small offsets from the device's factory calibration.\n
        A drawback is that calibration parameters will slightly change from one run to another, which can be an issue for repeatability.\n
        If set to true, self-calibration will be disabled and calibration parameters won't be optimized.\n
        default : false
        \note In most situations, self calibration should remain enabled.

         */
        bool camera_disable_self_calib;

        /**
        By default, the SDK only computes a single depth map, aligned with the left camera image.\n
        This parameter allows you to enable the \ref MEASURE "MEASURE::DEPTH_RIGHT" and other \ref MEASURE "MEASURE::<XXX>_RIGHT" at the cost of additional computation time.\n

        For example, mixed reality pass-through applications require one depth map per eye, so this parameter can be activated.
        \n default : false
         */
        bool enable_right_side_measure;

        /**
        When playing back an SVO file, each call to \ref Camera::grab() will extract a new frame and use it.\n
        However, this ignores the real capture rate of the images saved in the SVO file.\n
        Enabling this parameter will bring the SDK closer to a real simulation when playing back a file by using the images' timestamps. However, calls to \ref Camera::grab() will return an error when trying to play to fast, and frames will be dropped when playing too slowly.

        \n default : false
         */
        bool svo_real_time_mode;

        /**
        The SDK offers several \ref DEPTH_MODE options offering various level of performance and accuracy.
        \n This parameter allows you to set the \ref DEPTH_MODE that best matches your needs.
        \n default : \ref DEPTH_MODE "DEPTH_MODE::PERFORMANCE"
         */
        DEPTH_MODE depth_mode;

        /**
        Regions of the generated depth map can oscillate from one frame to another. These oscillations result from a lack of texture (too homogeneous) on an object and by image noise.
        \n This parameter enables a stabilization filter that reduces these oscillations.
        \n default : true
        \note The stabilization uses the positional tracking to increase its accuracy, so the Positional Tracking module will be enabled automatically when set to true.\n
        Notice that calling \ref Camera::enablePositionalTracking with your own parameters afterward is still possible.
         */
        int depth_stabilization;

        /**
        This parameter allows you to specify the minimum depth value (from the camera) that will be computed, measured in the \ref UNIT you define.
        \n In stereovision (the depth technology used by the camera), looking for closer depth values can have a slight impact on performance and memory consumption.
        \n On most of modern GPUs, performance impact will be low. However, the impact of memory footprint will be visible.
        \n In cases of limited computation power, increasing this value can provide better performance.
        \n default : (-1) corresponding to 700 mm for a ZED/ZED2 and 300 mm for ZED Mini.

        \note With a ZED camera you can decrease this value to 300 mm whereas you can set it to 100 mm using a ZED Mini and 200 mm for a ZED2. In any case this value cannot be greater than 3 meters.
        \note Specific value (0) : This will set the depth minimum distance to the minimum authorized value :
                                   - 300mm for ZED
                                   - 100mm for ZED-M
                                   - 200mm for ZED2
         */
        float depth_minimum_distance;

        /**
          When estimating the depth, the SDK uses this upper limit to turn higher values into \ref TOO_FAR ones.
         The current maximum distance that can be computed in the defined \ref UNIT.
        
        \note Changing this value has no impact on performance and doesn't affect the positional tracking nor the spatial mapping. (Only the depth, point cloud, normals)
         */
        float depth_maximum_distance;

        /**
        This parameter allows you to select the unit to be used for all metric values of the SDK. (depth, point cloud, tracking, mesh, and others).
        \n default : \ref UNIT "UNIT::MILLIMETER"
         */
        UNIT coordinate_units;

        /**
        Positional tracking, point clouds and many other features require a given \ref COORDINATE_SYSTEM to be used as reference.
        This parameter allows you to select the \ref COORDINATE_SYSTEM use by the \ref Camera to return its measures.
        \n This defines the order and the direction of the axis of the coordinate system.
        \n default : \ref COORDINATE_SYSTEM "COORDINATE_SYSTEM::IMAGE"
         */
        COORDINATE_SYSTEM coordinate_system;

        /**
        By default the SDK will use the most powerful NVIDIA graphics card found.
        However, when running several applications, or using several cameras at the same time, splitting the load over available GPUs can be useful.
        This parameter allows you to select the GPU used by the \ref Camera using an ID from 0 to n-1 GPUs in your PC.
        \n default : -1
        \note A non-positive value will search for all CUDA capable devices and select the most powerful.
         */
        CUdevice sdk_gpu_id;

        /**
        This parameters allows you to enable the verbosity of the SDK to get a variety of runtime information in the console.
        When developing an application, enabling verbose mode can help you understand the current SDK behavior.
        \n However, this might not be desirable in a shipped version.
        \n default : false
        \note The verbose messages can also be exported into a log file. See \ref sdk_verbose_log_file for more.
         */
        bool sdk_verbose;

        /**
        When \ref sdk_verbose is enabled, this parameter allows you to redirect both the SDK verbose messages and your own application messages to a file.
        \n default : (empty) Should contain the path to the file to be written. A file will be created if missing.

        \note Setting this parameter to any value will redirect all std::cout calls of the entire program. This means that your own std::cout calls will be redirected to the log file.
        \note This parameter can be particularly useful for creating a log system, and with Unreal or Unity applications that don't provide a standard console output.
        \warning The log file won't be clear after successive executions of the application. This means that it can grow indefinitely if not cleared.
         */
        String sdk_verbose_log_file;

        /**
        If your application uses another CUDA-capable library, giving its CUDA context to the SDK can be useful when sharing GPU memories.
        \n This parameter allows you to set the CUDA context to be used by the SDK.
        \n Leaving this parameter empty asks the SDK to create its own context.
        \n default : (empty)

        \note When creating you own CUDA context, you have to define the device you will use. Do not forget to also specify it on \ref sdk_gpu_id.
        \note <b>On Jetson</b>, you have to set the flag CU_CTX_SCHED_YIELD, during CUDA context creation.
        \note You can also let the SDK create its own context, and use \ref Camera::getCUDAContext() to use it.
         */
        CUcontext sdk_cuda_ctx;

        /**
        The SDK can handle different input types:
          - Select a camera by its ID (/dev/video<i>X</i> on Linux, and 0 to N cameras connected on Windows)
          - Select a camera by its serial number
          - Open a recorded sequence in the SVO file format
          - Open a streaming camera from its IP address and port
		  
        This parameter allows you to select to desired input. It should be used like this:

        \code
        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = True; // Enable verbose mode
        init_params.input.setFromCameraID(0); // Selects the camera with ID = 0
        \endcode

        \code
        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = True; // Enable verbose mode
        init_params.input.setFromSerialNumber(1010); // Selects the camera with serial number = 1010
        \endcode

        \code
        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = True; // Enable verbose mode
        init_params.input.setFromSVOFile("/path/to/file.svo"); // Selects the and SVO file to be read
        \endcode

        \code
        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = True; // Enable verbose mode
        init_params.input.setFromStream("192.168.1.42"); // Selects the IP address of the streaming camera. A second optional parameter is available for port selection.
        \endcode

        \n Available cameras and their ID/serial can be listed using \ref Camera::getDeviceList and \ref Camera::getStreamingDeviceList.
        \n Each \ref Camera will create its own memory (CPU and GPU), therefore the number of ZED used at the same time can be limited by the configuration of your computer. (GPU/CPU memory and capabilities)
        \n default : (empty)
        \n See \ref InputType for complementary information.
         */
        InputType input;

        /**
        Set the optional path where the SDK has to search for the settings files (SN<XXXX>.conf files). Those file contains the calibration of the camera.
        \n default : (empty). The SNXXX.conf file will be searched in the default directory (/usr/local/zed/settings/ for Linux or C:/ProgramData/stereolabs/settings for Windows)
        \note if a path is specified and no files has been found, the SDK will search on the default path (see default) for the *.conf file.
        \note Automatic download of conf file (through ZED Explorer or the installer) will still download the files on the default path. If you want to use another path by using this entry, make sure to copy the file in the proper location.

        \code
        InitParameters init_params; // Set initial parameters
        std::string home=getenv("HOME"); //get /home/user as string using getenv()
        std::string path= home+"/Documents/settings/"; //assuming /home/<user>/Documents/settings/SNXXXX.conf exists. Otherwise, it will be searched in /usr/local/zed/settings/
        init_params.optional_settings_path =sl::String(path.c_str());
        \endcode
         */
        String optional_settings_path;


        /**
        Force the motion sensors opening of the ZED 2 / ZED-M to open the camera.
        \n default : false.
        \n If set to false, the SDK will try to <b>open and use</b> the IMU (second USB device on USB2.0) and will open the camera successfully even if the sensors failed to open.
        \n This can be used for example when using a USB3.0 only extension cable (some fiber extension for example).
        \n This parameter only impacts the LIVE mode.
        \n If set to true, the camera will fail to open if the sensors cannot be opened. This parameter should use when the IMU data must be available, such as Object Detection module or when the gravity is needed.
        \note This setting is not taken into account for ZED camera since it does not include sensors.
         */
        bool sensors_required;

        /**
        Enable or Disable the Enhanced Contrast Technology, to improve image quality.
        \n default : true.
        \n If set to true, iamge enhancement will be activated in camera ISP. Otherwise, the image will not be enhanced by the IPS.
        \n This only works for firmware version starting from 1523 and up.
         */
        bool enable_image_enhancement;

        /**
        \brief Default constructor. All the parameters are set to their default and optimized values.
         */
        InitParameters(RESOLUTION camera_resolution_ = RESOLUTION::HD720,
                int camera_fps_ = 0,
                bool svo_real_time_mode_ = false,
#ifdef _SL_JETSON_
                DEPTH_MODE depth_mode_ = DEPTH_MODE::PERFORMANCE,
#else
                DEPTH_MODE depth_mode_ = DEPTH_MODE::ULTRA,
#endif
                UNIT coordinate_units_ = UNIT::MILLIMETER,
                COORDINATE_SYSTEM coordinate_system_ = COORDINATE_SYSTEM::IMAGE,
                bool sdk_verbose_ = false,
                int sdk_gpu_id_ = -1,
                float depth_minimum_distance_ = -1.,
                float depth_maximum_distance_ = -1.,
                bool camera_disable_self_calib_ = false,
                int camera_image_flip_ = FLIP_MODE::AUTO,
                bool enable_right_side_measure_ = false,
                String sdk_verbose_log_file_ = String(),
                int depth_stabilization_ = 1,
                CUcontext sdk_cuda_ctx_ = CUcontext(),
                InputType input_type = InputType(),
                String optional_settings_path_ = String(),
                bool sensors_required_ = false,
                bool enable_image_enhancement_ = true)
        : camera_resolution(camera_resolution_)
        , camera_fps(camera_fps_)
        , svo_real_time_mode(svo_real_time_mode_)
        , depth_mode(depth_mode_)
        , coordinate_units(coordinate_units_)
        , coordinate_system(coordinate_system_)
        , sdk_verbose(sdk_verbose_)
        , sdk_gpu_id(sdk_gpu_id_)
        , depth_minimum_distance(depth_minimum_distance_)
        , depth_maximum_distance(depth_maximum_distance_)
        , camera_disable_self_calib(camera_disable_self_calib_)
        , camera_image_flip(camera_image_flip_)
        , enable_right_side_measure(enable_right_side_measure_)
        , sdk_verbose_log_file(sdk_verbose_log_file_)
        , depth_stabilization(depth_stabilization_)
        , sdk_cuda_ctx(sdk_cuda_ctx_)
        , input(input_type)
        , optional_settings_path(optional_settings_path_)
        , sensors_required(sensors_required_)
        , enable_image_enhancement(enable_image_enhancement_) {
        }

        /**
        This function saves the current set of parameters into a file to be reloaded with the \ref load() function.
        \param filename : the path to the file in which the parameters will be stored.
        \return True if file was successfully saved, otherwise false.

        \code
        InitParameters init_params; // Set initial parameters
        init_params.sdk_verbose = True; // Enable verbose mode
        init_params.input.setFromSVOFile("/path/to/file.svo"); // Selects the and SVO file to be read
        init_params.save("initParameters.conf"); // Export the parameters into a file
        \endcode

         */
        bool save(String filename);

        /**
        This function set the other parameters from the values contained in a previously \ref save() "saved" file.
        \param filename : the path to the file from which the parameters will be loaded.
        \return True if the file was successfully loaded, otherwise false.

        \code
        InitParameters init_params; // Set initial parameters
        init_params.load("initParameters.conf"); // Load the init_params from a previously exported file
        \endcode

        \note As the InitParameters files can be easily modified manually (using a text editor) this functions allows you to test various settings without re-compiling your application.
         */
        bool load(String filename);
    };
    /** @} */

    /**
    \class RuntimeParameters.
    \ingroup Depth_group
    \brief Parameters that defines the behavior of the \ref grab.

    Default values are enabled.
    \n You can customize it to fit your application and then save it to create a preset that can be loaded for further executions.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ RuntimeParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        Defines the algorithm used for depth map computation, more info : \ref SENSING_MODE definition.
        \n default : \ref SENSING_MODE::STANDARD
         */
        SENSING_MODE sensing_mode;
        /**
        Provides 3D measures (point cloud and normals) in the desired reference frame (default is REFERENCE_FRAME::CAMERA)
        \n default : \ref REFERENCE_FRAME::CAMERA
         */
        REFERENCE_FRAME measure3D_reference_frame;

        /**
        Defines if the depth map should be computed.
        \n If false, only the images are available.
        \n default : true
         */
        bool enable_depth;

        /**
        Threshold to reject depth values based on their confidence.

        Each depth pixel has a corresponding confidence. (\ref MEASURE "MEASURE::CONFIDENCE")
        \n A lower value means more confidence and precision (but less density). An upper value reduces filtering (more density, less certainty).
        \n - \b setConfidenceThreshold(100) will allow values from \b 0 to \b 100. (no filtering)
        \n - \b setConfidenceThreshold(90) will allow values from \b 10 to \b 100. (filtering lowest confidence values)
        \n - \b setConfidenceThreshold(30) will allow values from \b 70 to \b 100. (keeping highest confidence values and lowering the density of the depth map)

        The value should be in [1,100].         
        \n By default, the confidence threshold is set at 100, meaning that no depth pixel will be rejected.

         */
        int confidence_threshold = 100;

        /**
        Threshold to reject depth values based on their textureness confidence.
        \deprecated see texture_confidence_threshold.
         */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED("use texture_confidence_threshold instead")/*@endcond*/
        int textureness_confidence_threshold = 100;

        /**
        Threshold to reject depth values based on their texture confidence.

        A lower value means more confidence and precision (but less density). An upper value reduces filtering (more density, less certainty).
        The value should be in [1,100].
        By default, the confidence threshold is set at 100, meaning that no depth pixel will be rejected.

         */
        int texture_confidence_threshold = 100;

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        RuntimeParameters(SENSING_MODE sensing_mode_ = SENSING_MODE::STANDARD,
                bool enable_depth_ = true, int confidence_threshold_ = 100,
                int texture_confidence_threshold_ = 100,
                REFERENCE_FRAME measure3D_reference_frame_ = REFERENCE_FRAME::CAMERA)
        : sensing_mode(sensing_mode_)
        , enable_depth(enable_depth_)
        , confidence_threshold(confidence_threshold_)
        , texture_confidence_threshold(texture_confidence_threshold_)
        , measure3D_reference_frame(measure3D_reference_frame_) {
        }

        /**
        \brief Saves the current set of parameters into a file.
        \param filename : the path to the file in which the parameters will be stored.
        \return true if the file was successfully saved, otherwise false.
         */
        bool save(String filename);

        /**
        \brief Loads the values of the parameters contained in a file.
        \param filename : the path to the file from which the parameters will be loaded.
        \return true if the file was successfully loaded, otherwise false.
         */
        bool load(String filename);
    };
    /** @} */

    /**
    \class PositionalTrackingParameters
    \ingroup PositionalTracking_group
    \brief Parameters for positional tracking initialization.

    A default constructor is enabled and set to its default parameters.
    \n You can customize it to fit your application and then save it to create a preset that can be loaded for further executions.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ PositionalTrackingParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        Position of the camera in the world frame when the camera is started. By default, it should be identity.
        \n Use this \ref Transform to place the camera frame in the world frame.
        \n default: Identity matrix.

        \note The camera frame (which defines the reference frame for the camera) is by default positioned at the world frame when tracking is started.
         */
        Transform initial_world_transform;

        /**
        This mode enables the camera to remember its surroundings. This helps correct positional tracking drift, and can be helpful for positioning
        different cameras relative to one other in space.
        \n default: true

        \warning: This mode requires more resources to run, but greatly improves tracking accuracy. We recommend leaving it on by default.
         */
        bool enable_area_memory;

        /**
        This mode enables smooth pose correction for small drift correction.
        \n default: false
         */
        bool enable_pose_smoothing;

        /**
        This mode initializes the tracking to be aligned with the floor plane to better position the camera in space.
        \n default: false
        \note: This launches floor plane detection in the background until a suitable floor plane is found.
        The tracking is in POSITIONAL_TRACKING_STATE::SEARCHING state.

        \warning: This features work best with the ZED-M since it needs an IMU to classify the floor.
         * The ZED needs to look at the floor during initialization for optimum results.
         */
        bool set_floor_as_origin;

        /**
        Area localization file that describes the surroundings, saved from a previous tracking session.
        \n default: (empty)

        \note Loading an area file will start a search phase, during which the camera will try to position itself in the previously learned area.

        \warning: The area file describes a specific location. If you are using an area file describing a different location, the tracking function will continuously search for a position and may not find a correct one.
        \warning The '.area' file can only be used with the same depth mode (\ref MODE) as the one used during area recording.
         */
        String area_file_path;

        /**
        This setting allows you to enable or disable IMU fusion. When set to false, only the optical odometry will be used.
        \n default: true
        \note This setting has no impact on the tracking of a ZED camera; only the ZED Mini uses a built-in IMU.
         */
        bool enable_imu_fusion;

        /**
        This setting allows you define the camera as static. If true, it will not move in the environment. This allows you to set its position using initial_world_transform.
        \n All SDK functionalities requiring positional tracking will be enabled.
        \n Camera::getPosition() will return the value set as initial_world_transform for the PATH, and identity as the POSE.
         */
        bool set_as_static;

        /**
        \brief  Default constructor. Sets all parameters to their default and optimized values.
         */
        PositionalTrackingParameters(Transform init_pos = Transform(), bool _enable_memory = true, bool _enable_pose_smoothing = false, String _area_path = String(),
                bool _set_floor_as_origin = false, bool _enable_imu_fusion = true, bool _set_as_static = false)
        : initial_world_transform(init_pos)
        , enable_area_memory(_enable_memory)
        , enable_pose_smoothing(_enable_pose_smoothing)
        , area_file_path(_area_path)
        , set_floor_as_origin(_set_floor_as_origin)
        , enable_imu_fusion(_enable_imu_fusion)
        , set_as_static(_set_as_static) {
        }

        /**
        \brief Saves the current set of parameters into a file.
        \param filename: the path to the file in which the parameters will be stored.
        \return true if the file was successfully saved, otherwise false.
         */
        bool save(String filename);

        /**
        \brief Loads the values of the parameters contained in a file.
        \param filename: the path to the file from which the parameters will be loaded.
        \return true if the file was successfully loaded, otherwise false.
         */
        bool load(String filename);
    };

    /**
    \class SpatialMappingParameters
    \ingroup SpatialMapping_group
    \brief Sets the spatial mapping parameters.

    Instantiating with the default constructor sets all parameters to their default values.
    \n You can customize these values to fit your application, and then save them to a preset to be loaded in future executions.

    \note Users can adjust these parameters as they see fit.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ SpatialMappingParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        typedef std::pair<float, float> interval;

        /**
        \enum SPATIAL_MAP_TYPE
        \ingroup SpatialMapping_group
        \brief Lists the types of spatial maps that can be created.
         */
        enum class SPATIAL_MAP_TYPE {
            MESH, /**< Represent a surface with faces, 3D points are linked by edges, no color information.*/
            FUSED_POINT_CLOUD /**< Geometry is represented by a set of 3D colored points.*/
        };

        /**
        \enum MAPPING_RESOLUTION
        \ingroup SpatialMapping_group
        \brief List the spatial mapping resolution presets.
         */
        enum class MAPPING_RESOLUTION {
            HIGH, /**< Create a detail geometry, requires lots of memory.*/
            MEDIUM, /**< Smalls variations in the geometry will disappear, useful for big object*/
            LOW /**< Keeps only huge variations of the geometry , useful outdoor.*/
        };

        /**
        \enum MAPPING_RANGE
        \ingroup SpatialMapping_group
        \brief Lists the spatial mapping depth range presets.
         */
        enum class MAPPING_RANGE {
            SHORT, /**< Only depth close to the camera will be used during spatial mapping.*/
            MEDIUM, /**< Medium depth range.*/
            LONG, /**< Takes into account objects that are far, useful outdoor.*/
            AUTO /**< Depth range will be computed based on current Camera states and parameters.*/
        };

        /**
        \brief Default constructor. Sets all parameters to their default and optimized values.
         */
        SpatialMappingParameters(MAPPING_RESOLUTION resolution = MAPPING_RESOLUTION::MEDIUM,
                MAPPING_RANGE range = MAPPING_RANGE::AUTO,
                int max_memory_usage_ = 2048,
                bool save_texture_ = false,
                bool use_chunk_only_ = false,
                bool reverse_vertex_order_ = false,
                SPATIAL_MAP_TYPE map_type = SPATIAL_MAP_TYPE::MESH
                );

        /**
        \brief Returns the resolution corresponding to the given \ref MAPPING_RESOLUTION preset.
        \param mapping_resolution: The desired \ref MAPPING_RESOLUTION. Default: \ref MAPPING_RESOLUTION::HIGH.
        \return The resolution in meters.
         */
        static float get(MAPPING_RESOLUTION mapping_resolution = MAPPING_RESOLUTION::MEDIUM);

        /**
        \brief Sets the resolution corresponding to the given \ref MAPPING_RESOLUTION preset.
        \param mapping_resolution: The desired \ref MAPPING_RESOLUTION.  Default: \ref MAPPING_RESOLUTION::HIGH.
         */
        void set(MAPPING_RESOLUTION mapping_resolution = MAPPING_RESOLUTION::MEDIUM);

        /**
        \brief  Returns the maximum value of depth corresponding to the given \ref MAPPING_RANGE presets.
        \param mapping_range: The desired \ref MAPPING_RANGE. Default: \ref MAPPING_RANGE::MEDIUM.
        \return The maximum value of the depth.
         */
        static float get(MAPPING_RANGE mapping_range = MAPPING_RANGE::MEDIUM);

        /**
        \brief Sets the maximum value of the depth corresponding to the given \ref MAPPING_RANGE preset.
        \param mapping_range: The desired \ref MAPPING_RANGE. Default: \ref MAPPING_RANGE.MEDIUM.
         */
        void set(MAPPING_RANGE mapping_range = MAPPING_RANGE::MEDIUM);

        /**
        \brief  Returns the recommended maximum depth value for the given \ref MAPPING_RESOLUTION preset.
        \param mapping_resolution: The desired \ref MAPPING_RESOLUTION.
        \param camera: The Camera object that will run the spatial mapping.
        \return The maximum value of the depth in meters.
         */
        static float getRecommendedRange(MAPPING_RESOLUTION mapping_resolution, Camera& camera);

        /**
        \brief  Returns the recommended maximum depth value for the given \ref MAPPING_RESOLUTION preset.
        \param resolution_meters: The desired resolution in meters.
        \param camera: The Camera object that will run the spatial mapping.
        \return The maximum value of the depth in meters.
         */
        static float getRecommendedRange(float resolution_meters, Camera& camera);

        /**
        \brief Spatial mapping resolution in meters. Should fit \ref allowed_resolution.
         */
        float resolution_meter = 0.05f;

        /**
        \brief The resolutions allowed by the spatial mapping.
        \n allowed_resolution.first is the minimum value allowed.
        \n allowed_resolution.second is the maximum value allowed.
         */
        static const interval allowed_resolution;

        /**
        \brief Depth range in meters.
        Can be different from the value set by \ref setDepthMaxRangeValue.
        \n Set to 0 by default. In this case, the range is computed from resolution_meter
        and from the current internal parameters to fit your application.
         */
        float range_meter = 0.f;

        /**
        \brief Range of the maximum depth value allowed by spatial mapping.
        \n allowed_range.first is the minimum value allowed.
        \n allowed_range.second is the maximum value allowed.
         */
        static const interval allowed_range;

        /**
        \brief Set to true if you want to be able to apply the texture to your mesh after its creation.

        \note This option will consume more memory.
        \note This option is only available for SPATIAL_MAP::TYPE_MESH
         */
        bool save_texture = false;

        /**
        \brief Set to false if you want to ensure consistency between the mesh and its inner chunk data.

        \note Updating the mesh is time-consuming. Setting this to true results in better performance.
         */
        bool use_chunk_only = false;

        /**
        \brief The maximum CPU memory (in megabytes) allocated for the meshing process.
         */
        int max_memory_usage = 2048;

        /**
        \brief Specify if the order of the vertices of the triangles needs to be inverted. If your display process does not handle front and back face culling, you can use this to correct it.

        \note This option is only available for SPATIAL_MAP::TYPE_MESH
         */
        bool reverse_vertex_order = false;

        /**
        \brief The type of spatial map to be created. This dictates the format that will be used for the mapping(e.g. mesh, point cloud). See \ref SPATIAL_MAP_TYPE
         */
        SPATIAL_MAP_TYPE map_type = SPATIAL_MAP_TYPE::MESH;

        /**
        \brief Saves the current set of parameters into a file.
        \param filename: the path to the file in which the parameters will be stored.
        \return Ttrue if the file was successfully saved. false otherwise.
         */
        bool save(String filename);

        /**
        \brief Loads the values of the parameters contained in a file.
        \param filename: the path to the file from which the parameters will be loaded.
        \return true if the file was successfully loaded. false otherwise.
         */
        bool load(String filename);
    };

    /**
    \class StreamingParameters
    \ingroup Video_group
    \brief Sets the streaming parameters.

    The default constructor sets all parameters to their default settings.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/StreamingParameters {
        /**
        \brief Defines the codec used for streaming.
        \warning If HEVC is used, make sure the receiving host is compatible with H265 decoding (Pascal NVIDIA card or newer). If not, prefer to use H264 since every compatible NVIDIA card supports H264 decoding
         */
        STREAMING_CODEC codec = STREAMING_CODEC::H265;

        /**
        \brief Defines the port used for streaming.
        \warning Port must be an even number. Any odd number will be rejected.
        \warning Port must be opened.
         */
        unsigned short port = 30000;

        /**
        \brief Defines the streaming bitrate in Kbits/s
         * 
         * 
         *  | STREAMING_CODEC  | Resolution   | FPS   | bitrate (kbps) |
         *  |------------------|--------------|-------|----------------|
         *  | H264             |  HD2K        |   15  |     8500       |
         *  | H264             |  HD1080      |   30  |    12500       |
         *  | H264             |  HD720       |   60  |     7000       |
         *  | H265             |  HD2K        |   15  |     7000       |
         *  | H265             |  HD1080      |   30  |    11000       |
         *  | H265             |  HD720       |   60  |     6000       |

        \note Available range : [1000 - 30000]
         */
        unsigned int bitrate = 8000;

        /**
        \brief Defines the gop size in number of frames.
        \note if value is set to -1, the gop size will last at maximum 2 seconds, depending on camera fps.
        \note The gop size determines the maximum distance between IDR/I-frames. Very high GOP size will result in slightly more efficient compression, especially on static scenes. But latency will increase.
        \note maximum value: 256
         */
        int gop_size = -1;

        /**
        \brief Enable/Disable adaptive bitrate
        \note Bitrate will be adjusted depending the number of packet dropped during streaming.
        \note if activated, bitrate can vary between [bitrate/4, bitrate]
        \Warning Currently, the adaptive bitrate only works when "sending" device is a NVIDIA Jetson (X1,X2,Xavier,Nano)
         */
        bool adaptative_bitrate = false;

        /**
        \brief Defines a single chunk size
        \note Stream buffers are divided in X number of chunk where each chunk is "chunk_size" bytes long.
        \note Default value is 16084. You can lower this value if network generates a lot of packet lost : this will
        generates more chunk for a single image, but each chunk sent will be lighter to avoid inside-chunk corruption.
        Increasing this value can decrease latency.
        \note Available range : [4096 - 65000]
         */
        unsigned short chunk_size = 16084;


        /**
         \brief defines the target framerate for the streaming output.
         \warning This framerate must be below or equal to the camera framerate. Allowed framerates are 15,30, 60 or 100 if possible.
         Any other values will be discarded and camera FPS will be taken.
         \default 0 means that the camera framerate will be taken
         */
        unsigned int target_framerate  = 0;


        /**
        \brief Default constructor. Set all parameters to their default values
         */
        StreamingParameters(STREAMING_CODEC codec_ = STREAMING_CODEC::H265,
                unsigned short port_ = 30000,
                unsigned int bitrate_ = 8000,
                int gop_size_ = -1,
                bool adaptative_bitrate_ = false,
                unsigned short chunk_size_ = 32768,
                unsigned int target_framerate_ =0
                );
    };

    /**
        \class RecordingParameters
        \ingroup Video_group
        \brief Sets the recording parameters.

        The default constructor sets all parameters to their default settings.

        \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/RecordingParameters {
        /**
        \brief filename of the SVO file.
         */
        String video_filename;

        /**
        \brief compression_mode : can be one of the \ref SVO_COMPRESSION_MODE enum
         */
        SVO_COMPRESSION_MODE compression_mode = SVO_COMPRESSION_MODE::H264;

        /**
         \brief bitrate :  override default bitrate of the SVO file, in KBits/s. Only works if \ref SVO_COMPRESSION_MODE is H264 or H265.
         \default : 0 means default values (depends on the resolution)
         \note Available range : 0 or [1000 - 60000]
         */
        unsigned int bitrate = 0;

        /**
         \brief defines the target framerate for the recording module.
         \warning This framerate must be below or equal to the camera framerate and camera framerate must be a multiple of the target framerate. It means that
         it must respect camera_framerate%target_framerate==0
         Allowed framerates are 15,30, 60 or 100 if possible.
         Any other values will be discarded and camera FPS will be taken.
         \default 0 means that the camera framerate will be taken
         */
        unsigned int target_framerate  = 0;

        /**
        \brief In case of streaming input, if set to false, it will avoid decoding/re-encoding and convert directly streaming input into a SVO file.
        This saves a encoding session and can be especially useful on NVIDIA Geforce cards where the number of encoding session is limited.
        \note compression_mode, target_framerate and bitrate will be ignored in this mode.
         */
        bool transcode_streaming_input = false;





        /**
        \brief Default constructor. Set all parameters to their default values
         */
        RecordingParameters(String video_filename_ = "myRecording.svo",
                SVO_COMPRESSION_MODE compression_mode_ = SVO_COMPRESSION_MODE::H264,
                unsigned int target_framerate_ = 0,
                unsigned int bitrate_ = 0,
                bool transcode_streaming_input_=false) :
        video_filename(video_filename_)
        , compression_mode(compression_mode_)
        , bitrate(bitrate_)
        , target_framerate(target_framerate_)
        , transcode_streaming_input(transcode_streaming_input_){
        }
    };

    /**
    \class ObjectDetectionParameters
    \ingroup Object_group
    \brief Sets the object detection parameters.

    The default constructor sets all parameters to their default settings.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ObjectDetectionParameters {
        /**
        \brief Defines if the object detection is synchronized to the image or runs in a separate thread.
        If set to true, the detection is run for every grab, otherwise, the thread runs at its own speed, which can lead to new detection once in a while.
         */
        bool image_sync = true;

        /**
        \brief Defines if the object detection will track objects across images flow
         */
        bool enable_tracking = true;

        /**
        \brief Defines if the mask object will be computed
         */
        bool enable_mask_output = false;


        /**
        \brief Enable human pose estimation with skeleton keypoints output
         */
        DETECTION_MODEL detection_model = DETECTION_MODEL::MULTI_CLASS_BOX;

        /**
        \brief Default constructor. Set all parameters to their default values
         */
        ObjectDetectionParameters(bool image_sync_ = true, bool enable_tracking_ = true,
                bool enable_mask_output_ = false, DETECTION_MODEL detection_model = DETECTION_MODEL::MULTI_CLASS_BOX);
    };

    /**
    \class ObjectDetectionRuntimeParameters
    \ingroup Object_group
    \brief Sets the object detection runtime parameters.

    The default constructor sets all parameters to their default settings.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ObjectDetectionRuntimeParameters {
        /**
        \brief Defines the confidence threshold: interval between 1 and 99. A confidence of 1 meaning a low
         *  threshold, more uncertain objects and 99 very few but very precise objects.
         * If the scene contains a lot of objects, increasing the confidence can slightly speed up the process, since every object instances are tracked.
         */
        float detection_confidence_threshold;

        /**
        \brief Defines which object type to detect and track, by default (empty vector) everything.
         * Fewer objects type can slightly speed up the process, since every objects are tracked.
         */
        std::vector<OBJECT_CLASS> object_class_filter;

        /**
        \brief Default constructor. Set all parameters to their default values
         */
        ObjectDetectionRuntimeParameters(float detection_confidence_threshold = 50, std::vector<OBJECT_CLASS> object_class_filter = std::vector<OBJECT_CLASS> ());
    };



    class CameraMemberHandler;

    /**
    \class Camera
    \ingroup Video_group
    \brief This class is the main interface with the camera and the SDK features, such as: video, depth, tracking, mapping, and more.
    \n Find more information in the detailed description below.\n


     A standard program will use the \ref Camera class like this:
     \code
     #include <sl/Camera.hpp>

     using namespace sl;

     int main(int argc, char **argv) {

             // --- Initialize a Camera object and open the ZED
             // Create a ZED camera object
             Camera zed;

             // Set configuration parameters
             InitParameters init_params;
             init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode
             init_params.camera_fps = 60; // Set fps at 60

             // Open the camera
             ERROR_CODE err = zed.open(init_params);
             if (err != SUCCESS) {
                     std::cout << toString(err) << std::endl;
                     exit(-1);
             }

             sl::RuntimeParameters runtime_param;
             runtime_param.sensing_mode = SENSING_MODE::STANDARD;

             // --- Main loop grabing images and depth values
             // Capture 50 frames and stop
             int i = 0;
             Mat image, depth;
             while (i < 50) {
                     // Grab an image
                     if (zed.grab(runtime_param) == SUCCESS) { // A new image is available if grab() returns SUCCESS

                             //Display a pixel color
                             zed.retrieveImage(image, VIEW::LEFT); // Get the left image
                             sl::uchar4 centerRGB;
                             image.getValue<sl::uchar4>(image.getWidth() / 2, image.getHeight() / 2, &centerRGB);
                             std::cout << "Image " << i << " center pixel R:" << (int)centerRGB.r << " G:" << (int)centerRGB.g << " B:" << (int)centerRGB.b << std::endl;

                             //Display a pixel depth
                             zed.retrieveMeasure(depth, MEASURE::DEPTH); // Get the depth map
                             float centerDepth;
                             depth.getValue<float>(depth.getWidth() / 2, depth.getHeight() / 2, &centerDepth);
                             std::cout << "Image " << i << " center depth:" << centerDepth << std::endl;

                             i++;
                     }
             }

             // --- Close the Camera
             zed.close();
             return 0;
     }
     \endcode
     *
     */
    class /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ Camera {
        friend CameraMemberHandler;
        friend SpatialMappingParameters;
        ZED_SDK_VERSION_ATTRIBUTE

    public:
        /**
        \brief Default constructor which creates an empty Camera object.
        \n Parameters will be set when calling \ref open() "open(init_param)" with the desired \ref InitParameters .

        The Camera object can be created like this:
        \code
        Camera zed;
        \endcode
        or
        \code
        Camera* zed = new Camera();
        \endcode
         */
        Camera();

        /**
        \brief The Camera destructor will call the \ref close() function and clear the memory previously allocated by the object.
         */
        ~Camera();

        /**
        \brief Opens the ZED camera from the provided \ref InitParameters.\n
        This function will also check the hardware requirements and run a self-calibration.

        \param init_parameters : a structure containing all the initial parameters. default : a preset of InitParameters.
        \return An error code giving information about the internal process. If \ref ERROR_CODE "SUCCESS" is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.

        Here is the proper way to call this function:
        \code
        Camera zed; // Create a ZED camera object

        InitParameters init_params; // Set configuration parameters
        init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode
        init_params.camera_fps = 60; // Set fps at 60

        // Open the camera
        ERROR_CODE err = zed.open(init_params);
        if (err != SUCCESS) {
                std::cout << toString(err) << std::endl; // Display the error
                exit(-1);
        }
        \endcode

        \note If you are having issues opening a camera, the diagnostic tool provided in the SDK can help you identify to problems.
        \n If this function is called on an already opened camera, \ref close() will be called.
        \n <b>Windows:</b> C:\\Program Files (x86)\\ZED SDK\\tools\\ZED Diagnostic.exe
        \n <b>Linux:</b> /usr/local/zed/tools/ZED Diagnostic
         */
        ERROR_CODE open(InitParameters init_parameters = InitParameters());


        /**
        \brief Returns the init parameters used. Correspond to the structure send when the \ref open() function was called.

        \return \ref InitParameters containing the parameters used to initialize the \ref Camera object.
         */
        InitParameters getInitParameters();

        /**
        \brief Reports if the camera has been successfully opened. It has the same behavior as checking if \ref open() returns \ref ERROR_CODE "SUCCESS".
        \return true if the ZED is already setup, otherwise false.
         */
        inline bool isOpened() {
            return opened;
        }

        /**
        \brief If \ref open() has been called, this function will close the connection to the camera (or the SVO file) and free the corresponding memory.

        If \ref open() wasn't called or failed, this function won't have any effects.
        \note If an asynchronous task is running within the \ref Camera object, like \ref saveAreaMap(), this function will wait for its completion.
        \n The \ref open() function can then be called if needed.

        \warning If the CUDA context was created by \ref open(), this function will destroy it. Please make sure to delete your GPU \ref sl::Mat objects before the context is destroyed.
         */
        void close();


        /**
        \brief This function will grab the latest images from the camera, rectify them, and compute the \ref retrieveMeasure() "measurements" based on the \ref RuntimeParameters provided (depth, point cloud, tracking, etc.)
        \n As measures are created in this function, its execution can last a few milliseconds, depending on your parameters and your hardware.
        \n The exact duration will mostly depend on the following parameters:
           - \ref InitParameters.enable_right_side_measure : Activating this parameter increases computation time
           - \ref InitParameters.depth_stabilization : Stabilizing the depth requires an additional computation load as it enables tracking
           - \ref InitParameters.camera_resolution : Lower resolutions are faster to compute
           - \ref InitParameters.depth_mode : \ref DEPTH_MODE "PERFORMANCE" will run faster than \ref DEPTH_MODE "ULTRA"
           - \ref enablePositionalTracking() : Activating the tracking is an additional load
           - \ref RuntimeParameters.sensing_mode : \ref SENSING_MODE "STANDARD" mode will run faster than \ref SENSING_MODE "FILL" mode, which needs to estimate the depth of occluded pixels.
           - \ref RuntimeParameters.enable_depth : Avoiding the depth computation must be faster. However, it is required by most SDK features (tracking, spatial mapping, plane estimation, etc.)

        \n This function is meant to be called frequently in the main loop of your application.

        \note Since ZED SDK 3.0, this function is blocking. It means that grab() will wait until a new frame is detected and available.
        If no new frames is available until timeout is reached, grab() will return \ref ERROR_CODE::CAMERA_NOT_DETECTED .

        \param rt_parameters : a structure containing all the runtime parameters. default : a preset of \ref RuntimeParameters.
        \return Returning \ref ERROR_CODE "SUCCESS" means that no problem was encountered. Returned errors can be displayed using \ref toString(error)

        \code
        // Set runtime parameters after opening the camera
        RuntimeParameters runtime_param;
        runtime_param.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode

        Mat image;
        while (true) {
            // Grab an image
            if (zed.grab(runtime_param) == SUCCESS) { // A new image is available if grab() returns SUCCESS
                zed.retrieveImage(image, VIEW::LEFT); // Get the left image

                // Use the image for your application
            }
        }
        \endcode
         */
        ERROR_CODE grab(RuntimeParameters rt_parameters = RuntimeParameters());


        /**
        \brief Returns the runtime parameters used. Correspond to the structure send when the \ref grab() function was called.

        \return \ref RuntimeParameters containing the parameters that defines the behavior of the \ref grab.
         */
        RuntimeParameters getRuntimeParameters();


        /**
        \brief Returns the calibration parameters, serial number and other information about the camera being used.

        As calibration parameters depend on the image resolution, you can provide a custom resolution as a parameter to get scaled information.
        \n When reading an SVO file, the parameters will correspond to the camera used for recording.

        \param image_size : You can specify a size different from default image size to get the scaled camera information. default = (0,0) meaning original image size (given by \ref getCameraInformation().camera_configuration.resolution ).
        \return \ref CameraInformation containing the calibration parameters of the ZED, as well as serial number and firmware version.

        \note The returned parameters might vary between two execution due to the \ref InitParameters.camera_disable_self_calib "self-calibration" being ran in the \ref open() method.
         */
        CameraInformation getCameraInformation(Resolution image_size = Resolution(0, 0));

        /**
        \brief Gets the Camera-created CUDA context for sharing it with other CUDA-capable libraries. This can be useful for sharing GPU memories.

        If you're looking for the opposite mechanism, where an existing CUDA context is given to the \ref Camera, please check \ref InitParameters.sdk_cuda_ctx

        \return The CUDA context used for GPU calls.
         */
        CUcontext getCUDAContext();

        ///@{
        /// @name Video
        // -----------------------------------------------------------------
        //                         Video :
        // -----------------------------------------------------------------

        /**
        \brief Retrieves images from the camera (or SVO file).

        Multiple images are available along with a view of various measures for display purposes.
        \n Available images and views are listed \ref VIEW "here".
        \n As an example, \ref VIEW "VIEW::DEPTH" can be used to get a gray-scale version of the depth map, but the actual depth values can be retrieved using \ref retrieveMeasure().
        \n
        \n <b>Memory</b>
        \n By default, images are copied from GPU memory to CPU memory (RAM) when this function is called.
        \n If your application can use GPU images, using the <b>type</b> parameter can increase performance by avoiding this copy.
        \n If the provided \ref Mat object is already allocated  and matches the requested image format, memory won't be re-allocated.
        \n
        \n <b>Image size</b>
        \n By default, images are returned in the resolution provided by \ref getCameraInformation().camera_configuration.resolution.
        \n However, you can request custom resolutions. For example, requesting a smaller image can help you speed up your application.

        \param mat : \b [out] the \ref Mat to store the image.
        \param view  : defines the image you want (see \ref VIEW). default : \ref VIEW "VIEW::LEFT".
        \param type : whether the image should be provided in CPU or GPU memory. default : \ref MEM "MEM::CPU."
        \param image_size : if specified, define the resolution of the output mat. If set to \ref Resolution "Resolution(0,0)" , the ZED resolution will be taken. default : (0,0).
        \return \ref "SUCCESS" if the method succeeded,
                \ref ERROR_CODE "ERROR_CODE::INVALID_FUNCTION_PARAMETERS" if the view mode requires a module not enabled (VIEW::DEPTH with DEPTH_MODE::NONE for example),
                \ref ERROR_CODE "ERROR_CODE::INVALID_RESOLUTION" if the width/height is higher than the input resolution (width,height) or the side by side input resolution (width x 2,height) for side by side view mode,
                \ref ERROR_CODE "ERROR_CODE::FAILURE" if another error occurred.

        \note As this function retrieves the images grabbed by the \ref grab() function, it should be called afterward.

        \code
        Mat leftImage, depthView; //create sl::Mat objects to store the images
        while (true) {
        // Grab an image
                if (zed.grab() == SUCCESS) { // A new image is available if grab() returns SUCCESS
                        zed.retrieveImage(leftImage, VIEW::LEFT); // Get the rectified left image
                        zed.retrieveImage(depthView, VIEW::DEPTH); // Get a grayscale preview of the depth map

                        //Display the center pixel colors
                        sl::uchar4 leftCenter;
                        leftImage.getValue<sl::uchar4>(leftImage.getWidth() / 2, leftImage.getHeight() / 2, &leftCenter);
                        std::cout << "leftImage center pixel R:" << (int)leftCenter.r << " G:" << (int)leftCenter.g << " B:" << (int)leftCenter.b << std::endl;

                        sl::uchar4 depthCenter;
                        depthView.getValue<sl::uchar4>(depthView.getWidth() / 2, depthView.getHeight() / 2, &depthCenter);
                        std::cout << "depthView center pixel R:" << (int)depthCenter.r << " G:" << (int)depthCenter.g << " B:" << (int)depthCenter.b << std::endl;
                }
        }
        \endcode
         */
        ERROR_CODE retrieveImage(Mat& mat, VIEW view = VIEW::LEFT, MEM type = MEM::CPU, Resolution image_size = Resolution(0, 0));

        /**
        \brief Returns the current value of the requested \ref VIDEO_SETTINGS "camera setting". (gain, brightness, hue, exposure, etc.)

        Possible values (range) of each setting are available \ref VIDEO_SETTINGS "here".

        \param setting : the requested setting.
        \return The current value for the corresponding setting. Returns -1 if encounters an error.

        \code
        int gain = zed.getCameraSettings(VIDEO_SETTINGS::GAIN);
        std::cout << "Current gain value: " << gain << std::endl;
        \endcode

        \note Works only if the camera is open in live mode. (Settings aren't exported in the SVO file format)
         */
        int getCameraSettings(VIDEO_SETTINGS settings);

        /**
        \brief Overloaded function for @VIDEO_SETTINGS::AEC_AGC_ROI which takes a Rect as parameter

        \param setting : must be set at @VIDEO_SETTINGS::AEC_AGC_ROI, otherwise the function will have no impact.
        \param [out] roi : Rect that defines the current target applied for AEC/AGC.
        \return @ERROR_CODE::SUCCESS if ROI has been applied. Other @ERROR_CODE otherwise.

        \note Works only if the camera is open in live mode with @VIDEO_SETTINGS::AEC_AGC_ROI. It will return @ERROR_CODE::INVALID_FUNCTION_CALL or @ERROR_CODE::INVALID_FUNCTION_PARAMETERS otherwise.
         */
        ERROR_CODE getCameraSettings(VIDEO_SETTINGS settings, Rect& roi, sl::SIDE side = sl::SIDE::BOTH);

        /**
        \brief Sets the value of the requested \ref VIDEO_SETTINGS "camera setting". (gain, brightness, hue, exposure, etc.)

        Possible values (range) of each setting are available \ref VIDEO_SETTINGS "here".

        \param settings : the setting to be set.
        \param value : the value to set, default : auto mode

        \code
        //Set the gain to 50
        zed.setCameraSettings(VIDEO_SETTINGS::GAIN, 50);
        \endcode

        \warning Setting \ref VIDEO_SETTINGS::EXPOSURE or \ref VIDEO_SETTINGS::GAIN to default will automatically sets the other to default.

        \note Works only if the camera is open in live mode.
         */
        void setCameraSettings(VIDEO_SETTINGS settings, int value = VIDEO_SETTINGS_VALUE_AUTO);


        /**
        \brief Overloaded function for @VIDEO_SETTINGS::AEC_AGC_ROI which takes a Rect as parameter

        \param setting : must be set at @VIDEO_SETTINGS::AEC_AGC_ROI, otherwise the function will have no impact.
        \param roi : Rect that defines the target to be applied for AEC/AGC computation. Must be given according to camera resolution.
        \return @ERROR_CODE::SUCCESS if ROI has been applied. Other @ERROR_CODE otherwise.

        \note Works only if the camera is open in live mode with @VIDEO_SETTINGS::AEC_AGC_ROI. It will return @ERROR_CODE::INVALID_FUNCTION_CALL or @ERROR_CODE::INVALID_FUNCTION_PARAMETERS otherwise.
         */
        ERROR_CODE setCameraSettings(VIDEO_SETTINGS settings, Rect roi, sl::SIDE side = sl::SIDE::BOTH, bool reset = false);



        /**
        \brief Returns the current framerate at which the \ref grab() method is successfully called.

        The returned value is based on the difference of camera \ref getTimestamp() "timestamps" between two successful grab() calls.

        \return The current SDK framerate

        \warning The returned framerate (number of images grabbed per second) can be lower than \ref InitParameters.camera_fps if the \ref grab() function runs slower than the image stream or is called too often.

        \code
        int current_fps = zed.getCurrentFPS();
        std::cout << "Current framerate: " << current_fps << std::endl;
        \endcode
         */
        float getCurrentFPS();

        /**
        \brief Returns the timestamp in the requested \ref TIME_REFERENCE.

        - When requesting the \ref TIME_REFERENCE "TIME_REFERENCE::IMAGE" timestamp, the UNIX nanosecond timestamp of the latest \ref grab() "grabbed" image will be returned.
        \n This value corresponds to the time at which the entire image was available in the PC memory. As such, it ignores the communication time that corresponds to 2 or 3 frame-time based on the fps (ex: 33.3ms to 50ms at 60fps).

        - When requesting the \ref TIME_REFERENCE "TIME_REFERENCE::CURRENT" timestamp, the current UNIX nanosecond timestamp is returned.

        \n This function can also be used when playing back an SVO file.

        \param reference_time : The selected \ref TIME_REFERENCE.

        \return The timestamp in nanosecond. 0 if not available (SVO file without compression).

        \note As this function returns UNIX timestamps, the reference it uses is common across several \ref Camera instances.
        \n This can help to organized the grabbed images in a multi-camera application.

        \code
        Timestamp last_image_timestamp = zed.getTimestamp(TIME_REFERENCE::IMAGE);
        Timestamp current_timestamp = zed.getTimestamp(TIME_REFERENCE::CURRENT);

        std::cout << "Latest image timestamp: " << last_image_timestamp << "ns from Epoch." << std::endl;
        std::cout << "Current timestamp: " << current_timestamp << "ns from Epoch." << std::endl;
        \endcode
         */
        Timestamp getTimestamp(sl::TIME_REFERENCE reference_time);

        /**
        \brief Returns the number of frames dropped since \ref grab() was called for the first time.

        A dropped frame corresponds to a frame that never made it to the grab function.
        \n This can happen if two frames were extracted from the camera when grab() is called. The older frame will be dropped so as to always use the latest (which minimizes latency).

        \return The number of frames dropped since the first \ref grab() call.
         */
        unsigned int getFrameDroppedCount();

        /**
        \brief Returns the current playback position in the SVO file.

        The position corresponds to the number of frames already read from the SVO file, starting from 0 to n.
        \n Each \ref grab() call increases this value by one (except when using \ref InitParameters.svo_real_time_mode).

        \return The current frame position in the SVO file. Returns -1 if the SDK is not reading an SVO.

        \note Works only if the camera is open in SVO playback mode.
        \see setSVOPosition() for an example.
         */
        int getSVOPosition();

        /**
        \brief Sets the playback cursor to the desired frame number in the SVO file.

        This function allows you to move around within a played-back SVO file. After calling, the next call to \ref grab() will read the provided frame number.

        \param frame_number : the number of the desired frame to be decoded.

        \note Works only if the camera is open in SVO playback mode.

        \code
        #include <sl/Camera.hpp>

        using namespace sl;

        int main(int argc, char **argv) {

                // Create a ZED camera object
                Camera zed;

                // Set configuration parameters
                InitParameters init_params;
                init_params.input.setFromSVOFile("path/to/my/file.svo");

                // Open the camera
                ERROR_CODE err = zed.open(init_params);
                if (err != SUCCESS) {
                        std::cout << toString(err) << std::endl;
                        exit(-1);
                }

                // Loop between frame 0 and 50
                int i = 0;
                Mat leftImage;
                while (zed.getSVOPosition() < zed.getSVONumberOfFrames()-1) {

                        std::cout << "Current frame: " << zed.getSVOPosition() << std::endl;

                        // Loop if we reached frame 50
                        if (zed.getSVOPosition() == 50)
                                zed.setSVOPosition(0);

                        // Grab an image
                        if (zed.grab() == SUCCESS) {
                                zed.retrieveImage(leftImage, VIEW::LEFT); // Get the rectified left image

                                // Use the image in your application
                        }
                }

                // Close the Camera
                zed.close();
                return 0;
        }
        \endcode
         */
        void setSVOPosition(int frame_number);

        /**
        \brief Returns the number of frames in the SVO file.
        \return The total number of frames in the SVO file (-1 if the SDK is not reading a SVO).

        \note Works only if the camera is open in SVO reading mode.
        \see setSVOPosition() for an example.
         */
        int getSVONumberOfFrames();

        ///@{
        /// @name Depth Sensing
        // -----------------------------------------------------------------
        //                         Depth functions:
        // -----------------------------------------------------------------

        /**
        \brief Computed measures, like depth, point cloud, or normals, can be retrieved using this method.

        \n Multiple measures are available after a \ref grab() call. A full list is available \ref MEASURE "here".
        \n
        \n <b>Memory</b>
        \n By default, images are copied from GPU memory to CPU memory (RAM) when this function is called.
        \n If your application can use GPU images, using the \b type parameter can increase performance by avoiding this copy.
        \n If the provided \ref Mat object is already allocated and matches the requested image format, memory won't be re-allocated.
        \n
        \n <b>Measure size</b>
        \n By default, measures are returned in the resolution provided by \ref getCameraInformation().camera_configuration.resolution .
        \n However, custom resolutions can be requested. For example, requesting a smaller measure can help you speed up your application.


        \param mat : \b [out] the \ref Mat to store the measures.
        \param measure : defines the measure you want. (see \ref MEASURE), default : \ref MEASURE "MEASURE::DEPTH"
        \param type : the type of the memory of provided mat that should by used. default : MEM::CPU.
        \param image_size : if specified, define the resolution of the output mat. If set to \ref Resolution "Resolution(0,0)" , the ZED resolution will be taken. default : (0,0).
        \return \ref "SUCCESS" if the method succeeded,
                \ref ERROR_CODE "ERROR_CODE::INVALID_FUNCTION_PARAMETERS" if the view mode requires a module not enabled (VIEW::DEPTH with DEPTH_MODE::NONE for example),
                \ref ERROR_CODE "ERROR_CODE::INVALID_RESOLUTION" if the width/height is higher than camera.getCameraInformation().camera_configuration.resolution or camera.getCameraInformation().camera_configuration.resolution x 2 for side by side view mode,
                \ref ERROR_CODE "ERROR_CODE::FAILURE" if another error occurred.

        \note As this function retrieves the measures computed by the \ref grab() function, it should be called after.
        \n
        \n Measures containing "RIGHT" in their names, requires \ref InitParameters.enable_right_side_measure to be enabled.

        \code
        Mat depthMap, pointCloud;
                sl::Resolution resolution = zed.getCameraInformation().camera_configuration.resolution ;
        int x = resolution.width / 2; // Center coordinates
        int y = resolution.height / 2;

        while (true) {
                if (zed.grab() == SUCCESS) { // Grab an image

                        zed.retrieveMeasure(depthMap, MEASURE::DEPTH, MEM::CPU); // Get the depth map
                        // Read a depth value
                        float centerDepth = 0;
                        depthMap.getValue<float>(x, y, &centerDepth, MEM::CPU); // each depth map pixel is a float value
                        if (isnormal(centerDepth)) { // + Inf is "too far", -Inf is "too close", Nan is "unknown/occlusion"
                                std::cout << "Depth value at center: " << centerDepth << " " << init_params.coordinate_units << std::endl;
                        }


                        zed.retrieveMeasure(pointCloud, MEASURE::XYZRGBA, MEM::CPU);// Get the point cloud
                        // Read a point cloud value
                        sl::float4 pcValue;
                        pointCloud.getValue<sl::float4>(x, y, &pcValue); // each point cloud pixel contains 4 floats, so we are using a sl::float4
                        if (isnormal(pcValue.z)) {
                                std::cout << "Point cloud coordinates at center: X=" << pcValue.x << ", Y=" << pcValue.y << ", Z=" << pcValue.z << std::endl;
                                unsigned char color[sizeof(float)];
                                memcpy(color, &pcValue[3], sizeof(float));
                                std::cout << "Point cloud color at center: R=" << (int)color[0] << ", G=" << (int)color[1] << ", B=" << (int)color[2] << std::endl;
                        }
                }
        }
        \endcode
         */
        ERROR_CODE retrieveMeasure(Mat& mat, MEASURE measure = MEASURE::DEPTH, MEM type = MEM::CPU, Resolution image_size = Resolution(0, 0));

        ///@}

        ///@{
        /// @name Positional Tracking
        // -----------------------------------------------------------------
        //                        Positional Tracking functions:
        // -----------------------------------------------------------------

        /**
        \brief Initializes and starts the positional tracking processes.

        This function allows you to enable the position estimation of the SDK. It only has to be called once in the camera's lifetime.
        \n When enabled, the \ref getPosition "position" will be update at each grab call.
        \n Tracking-specific parameter can be set by providing \ref PositionalTrackingParameters to this function.

        \param tracking_parameters : A structure containing all the \ref PositionalTrackingParameters . default : a preset of \ref PositionalTrackingParameters.
        \return \ref ERROR_CODE "ERROR_CODE::FAILURE" if the \ref area_file_path file wasn't found, \ref ERROR_CODE "SUCCESS" otherwise.
        \warning The positional tracking feature benefits from a high framerate. We found HD720@60fps to be the best compromise between image quality and framerate.

        \code
        #include <sl/Camera.hpp>

        using namespace sl;

        int main(int argc, char **argv) {

                // --- Initialize a Camera object and open the ZED
                // Create a ZED camera object
                Camera zed;

                // Set configuration parameters
                InitParameters init_params;
                init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode
                init_params.camera_fps = 60; // Set fps at 60

                // Open the camera
                ERROR_CODE err = zed.open(init_params);
                if (err != SUCCESS) {
                        std::cout << toString(err) << std::endl;
                        exit(-1);
                }

                // Set tracking parameters
                PositionalTrackingParameters track_params;
                track_params.enable_area_memory = true;

                // Enable positional tracking
                err = zed.enablePositionalTracking(track_params);
                if (err != SUCCESS) {
                        std::cout << "Tracking error: " << toString(err) << std::endl;
                        exit(-1);
                }

                // --- Main loop
                while (true) {
                        if (zed.grab() == SUCCESS) { // Grab an image and computes the tracking
                                Pose cameraPose;
                                zed.getPosition(cameraPose, REFERENCE_FRAME::WORLD);
                                std::cout << "Camera position: X=" << cameraPose.getTranslation().x << " Y=" << cameraPose.getTranslation().y << " Z=" << cameraPose.getTranslation().z << std::endl;
                        }
                }

                // --- Close the Camera
                zed.close();
                return 0;
        }
        \endcode
         */
        ERROR_CODE enablePositionalTracking(PositionalTrackingParameters tracking_parameters = PositionalTrackingParameters());

        /**
        \brief Retrieves the estimated position and orientation of the camera in the specified \ref REFERENCE_FRAME "reference frame".

        \n Using \ref REFERENCE_FRAME "REFERENCE_FRAME::WORLD", the returned pose relates to the initial position of the camera. (\ref PositionalTrackingParameters.initial_world_transform )
        \n Using \ref REFERENCE_FRAME "REFERENCE_FRAME::CAMERA", the returned pose relates to the previous position of the camera.
        \n
        \n If the tracking has been initialized with \ref PositionalTrackingParameters.enable_area_memory to true (default), this function can return \ref POSITIONAL_TRACKING_STATE "POSITIONAL_TRACKING_STATE::SEARCHING".
        \n This means that the tracking lost its link to the initial referential and is currently trying to relocate the camera. However, it will keep on providing position estimations.

        \param camera_pose \b [out]: the pose containing the position of the camera and other information (timestamp, confidence)
        \param reference_frame : defines the reference from which you want the pose to be expressed. Default : \ref REFERENCE_FRAME "REFERENCE_FRAME::WORLD".
        \return The current \ref POSITIONAL_TRACKING_STATE "state" of the tracking process.

        \n Extract Rotation Matrix : camera_pose.getRotation();
        \n Extract Translation Vector: camera_pose.getTranslation();
        \n Convert to Orientation / quaternion : camera_pose.getOrientation();

        \note The position is provided in the \ref InitParameters.coordinate_system . See \ref COORDINATE_SYSTEM for its physical origin.
        \warning This function requires the tracking to be enabled. \ref enablePositionalTracking() .

        \code
        // --- Main loop
        while (true) {
                if (zed.grab() == SUCCESS) { // Grab an image and computes the tracking
                        Pose cameraPose;
                        zed.getPosition(cameraPose, REFERENCE_FRAME::WORLD);
                        std::cout << "Camera position: X=" << cameraPose.getTranslation().x << " Y=" << cameraPose.getTranslation().y << " Z=" << cameraPose.getTranslation().z << std::endl;
                        std::cout << "Camera Euler rotation: X=" << cameraPose.getEulerAngles().x << " Y=" << cameraPose.getEulerAngles().y << " Z=" << cameraPose.getEulerAngles().z << std::endl;
                        std::cout << "Camera Rodrigues rotation: X=" << cameraPose.getRotationVector().x << " Y=" << cameraPose.getRotationVector().y << " Z=" << cameraPose.getRotationVector().z << std::endl;
                        std::cout << "Camera quaternion orientation: X=" << cameraPose.getOrientation().x << " Y=" << cameraPose.getOrientation().y << " Z=" << cameraPose.getOrientation().z << " W=" << cameraPose.getOrientation().w << std::endl;
                        std::cout << std::endl;
                }
        }
        \endcode
         */
        POSITIONAL_TRACKING_STATE getPosition(Pose& camera_pose, REFERENCE_FRAME reference_frame = REFERENCE_FRAME::WORLD);

        /**
        \brief Saves the current area learning file. The file will contain spatial memory data generated by the tracking.

        If the tracking has been initialized with \ref PositionalTrackingParameters.enable_area_memory to true (default), the function allows you to export the spatial memory.
        \n Reloading the exported file in a future session with \ref PositionalTrackingParameters.area_file_path initialize the tracking within the same referential.
        \n This function is asynchronous, and only triggers the file generation. You can use \ref getAreaExportState() to get the export state.
        The positional tracking keeps running while exporting.

        \param area_file_path : save the spatial memory database in an '.area' file.
        \return \ref ERROR_CODE "ERROR_CODE::FAILURE" if the \ref area_file_path file wasn't found, \ref SUCCESS otherwise.
        \see getAreaExportState()

        \note Please note that this function will also flush the area database that was built / loaded.
        \warning If the camera wasn't moved during the tracking session, or not enough, the spatial memory won't be usable and the file won't be exported.
        \n The \ref getAreaExportState() function will return \ref AREA_EXPORTING_STATE "AREA_EXPORTING_STATE::NOT_STARTED".
        \n A few meters (~3m) of translation or a full rotation should be enough to get usable spatial memory.
        \n However, as it should be used for relocation purposes, visiting a significant portion of the environment is recommended before exporting.


        \code
        ...
                // --- Main loop
                while (true) {
                        if (zed.grab() == SUCCESS) { // Grab an image and computes the tracking
                                Pose cameraPose;
                                zed.getPosition(cameraPose, REFERENCE_FRAME::WORLD);
                        }
                }

                // Export the spatial memory for future sessions
                zed.saveAreaMap("office.area"); // The actual file will be created asynchronously.
                std::cout << zed.getAreaExportState() << std::endl;

                // --- Close the Camera
                zed.close(); // The close method will wait for the end of the file creation using getAreaExportState().
                return 0;
        }
        \endcode
         */
        ERROR_CODE saveAreaMap(String area_file_path);

        /**
        \brief Returns the state of the spatial memory export process.

        As \ref saveAreaMap() only starts the exportation, this function allows you to know when the exportation finished or if it failed.

        \return The current \ref AREA_EXPORTING_STATE "state" of the spatial memory export process
         */
        AREA_EXPORTING_STATE getAreaExportState();

        /**
        \brief Resets the tracking, and re-initializes the position with the given transformation matrix.
        \param path : Position of the camera in the world frame when the function is called. By default, it is set to identity.
        \return \ref ERROR_CODE "ERROR_CODE::FAILURE" if the \ref area_file_path file wasn't found, \ref ERROR_CODE "SUCCESS" otherwise.

        \note Please note that this function will also flush the accumulated or loaded spatial memory.
         */
        ERROR_CODE resetPositionalTracking(const Transform& path);

        /**
        \brief Disables the positional tracking.

        The positional tracking is immediately stopped. If a file path is given, \ref saveAreaMap(area_file_path) will be called asynchronously. See \ref getAreaExportState() to get the exportation state.
        \n If the tracking has been enabled, this function will automatically be called by \ref close() .

        \param area_file_path : if set, saves the spatial memory into an '.area' file. default : (empty)
        \n area_file_path is the name and path of the database, e.g. "path/to/file/myArea1.area".
        \note The '.area' database depends on the depth map SENSING_MODE chosen during the recording. The same mode must be used to reload the database.
         */
        void disablePositionalTracking(String area_file_path = "");

        /**
        \brief Tells if the tracking module is enabled
         */
        bool isPositionalTrackingEnabled();


        /**
        \brief Returns the positional tracking parameters used. Correspond to the structure send when the \ref enablePositionalTracking() function was called.

        \return \ref PositionalTrackingParameters containing the parameters used for positional tracking initialization.
         */
        PositionalTrackingParameters getPositionalTrackingParameters();


        // -----------------------------------------------------------------
        //                        Sensors functions, for ZED2 and ZED-M only (using IMU)
        // -----------------------------------------------------------------


        /**
        \brief Retrieves the Sensors (IMU,magnetometer,barometer) Data at a specific time reference

        Calling getSensosrData with \ref TIME_REFERENCE "TIME_REFERENCE::CURRENT" gives you the latest sensors data received. Getting all the data requires to call this function at high frame rate in a thread.
        \n Calling getSensorsData with \ref TIME_REFERENCE "TIME_REFERENCE::IMAGE" gives you the sensors data at the time of the latest image \ref grab() "grabbed".
        \n
        \n \ref SensorsData object contains the previous IMUData structure that was used in ZED SDK v2.X:
        \n For IMU data, the values are provided in 2 ways :
        \n <b>Time-fused</b> pose estimation that can be accessed using:
        \n   <ul><li>\ref data.imu.pose</li>
        \n <b>Raw values</b> from the IMU sensor:
        \n   <ul><li>\ref data.imu.angular_velocity, corresponding to the gyroscope</li>
        \n   <li>\ref data.imu.linear_acceleration, corresponding to the accelerometer</li></ul>
        \n both gyroscope and accelerometer are synchronized. The delta time between previous and current value can be calculated using <li>\ref data.imu.timestamp</li>

        \note : The IMU quaternion (fused data) is given in the specified COORDINATE_SYSTEM of InitParameters.

                \return \ref ERROR_CODE::SUCCESS if sensors data have been extracted,
                \ref ERROR_CODE::SENSORS_NOT_AVAILABLE if the camera model is a ZED,
                \ref ERROR_CODE::MOTION_SENSORS_REQUIRED if the camera model is correct but the sensors module is not opened.
                \ref ERROR_CODE::INVALID_FUNCTION_PARAMETERS if the reference_time is not valid. See Warning.
        \warning : In SVO reading mode, the TIME_REFERENCE::CURRENT is currently not available (yielding \ref ERROR_CODE "ERROR_CODE::INVALID_FUNCTION_PARAMETERS".
         * Only the quaternion data and barometer data (if available) at TIME_REFERENCE::IMAGE are available. Other values will be set to 0.
         
                \code
                sl::SensorsData sensors_data;
        if (zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT) == ERROR_CODE::SUCCESS) {
                cout << " - IMU:\n";
                cout << " \t Orientation: {" << sensors_data.imu.pose.getOrientation() << "}\n";
                cout << " \t Acceleration: {" << sensors_data.imu.linear_acceleration << "} [m/sec^2]\n";
                cout << " \t Angular Velocitiy: {" << sensors_data.imu.angular_velocity << "} [deg/sec]\n";
                cout << " - Magnetometer\n \t Magnetic Field: {" << sensors_data.magnetometer.magnetic_field_calibrated << "} [uT]\n";
                cout << " - Barometer\n \t Atmospheric pressure:" << sensors_data.barometer.pressure << " [hPa]\n";
       
                                // retrieves camera sensors temperature
                                cout << " - Temperature\n";
                                float temperature;
                                for (int s = 0; s < static_cast<int>(SensorsData::TemperatureData::SENSOR_LOCATION::LAST); s++) {
                                auto sensor_loc = static_cast<SensorsData::TemperatureData::SENSOR_LOCATION>(s);
                                // depending on your Camera model or its firmware, differents sensors can give thermal information
                                if (sensors_data.temperature.get(sensor_loc, temperature) == ERROR_CODE::SUCCESS)
                                cout << " \t " << sensor_loc << ": " << temperature << "C\n";
                                }
        }      
        \endcode
         */
        ERROR_CODE getSensorsData(SensorsData& data, TIME_REFERENCE reference_time);

        /**
        \brief Set an optional IMU orientation hint that will be used to assist the tracking during the next \ref grab().

        This function can be used to assist the positional tracking rotation while using a ZED Mini or a ZED 2.

        \note This function is only effective if a ZED Mini (ZED-M) or a ZED 2 is used.
        \n It needs to be called before the \ref grab() function.
        \param sl::Transform to be ingested into IMU fusion. Note that only the rotation is used.
        \return SUCCESS if the transform has been passed, \ref ERROR_CODE "ERROR_CODE::INVALID_FUNCTION_CALL" otherwise (such as when use with the ZED camera due to its lack of an IMU).
         */
        ERROR_CODE setIMUPrior(const sl::Transform& transform);

        //@}

        ///@{
        /// @name Spatial Mapping
        // -----------------------------------------------------------------
        //                         Spatial Mapping functions:
        // -----------------------------------------------------------------

        /**
        \brief Initializes and starts the spatial mapping processes.

        The spatial mapping will create a geometric representation of the scene based on both tracking data and 3D point clouds.
        \n The resulting output can be a \ref Mesh or a \ref FusedPointCloud. It can be be obtained by calling \ref extractWholeSpatialMap() or \ref retrieveSpatialMapAsync().
        Note that \ref retrieveSpatialMapAsync() should be called after \ref requestSpatialMapAsync().

        \param spatial_mapping_parameters : the structure containing all the specific parameters for the spatial mapping.
        \n Default: a balanced parameter preset between geometric fidelity and output file size. For more information, see the \ref SpatialMappingParameters documentation.
        \return \ref ERROR_CODE "SUCCESS" if everything went fine, \ref ERROR_CODE "ERROR_CODE::FAILURE" otherwise.

        \warning The tracking (\ref enablePositionalTracking() ) and the depth (\ref RuntimeParameters.enable_depth ) needs to be enabled to use the spatial mapping.
        \warning The performance greatly depends on the spatial_mapping_parameters.
        \ Lower SpatialMappingParameters.range_meter and SpatialMappingParameters.resolution_meter for higher performance.
        If the mapping framerate is too slow in live mode, consider using an SVO file, or choose a lower mesh resolution.

        \note This features uses host memory (RAM) to store the 3D map. The maximum amount of available memory allowed can be tweaked using the SpatialMappingParameters.
        \n Exeeding the maximum memory allowed immediately stops the mapping.

        \code
        #ifndef NDEBUG
        #error "Spatial mapping requires Release mode, not Debug."
        #endif

        #include <sl/Camera.hpp>

        using namespace sl;

        int main(int argc, char **argv) {

                // Create a ZED camera object
                Camera zed;

                // Set initial parameters
                InitParameters init_params;
                init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode (default fps: 60)
                init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system (The OpenGL one)
                init_params.coordinate_units = UNIT::METER; // Set units in meters

                // Open the camera
                ERROR_CODE err = zed.open(init_params);
                if (err != SUCCESS)
                        exit(-1);

                // Positional tracking needs to be enabled before using spatial mapping
                sl::PositionalTrackingParameters tracking_parameters;
                err = zed.enablePositionalTracking(tracking_parameters);
                if (err != SUCCESS)
                        exit(-1);

                // Enable spatial mapping
                sl::SpatialMappingParameters mapping_parameters;
                err = zed.enableSpatialMapping(mapping_parameters);
                if (err != SUCCESS)
                        exit(-1);

                // Grab data during 500 frames
                int i = 0;
                sl::Mesh mesh; // Create a mesh object
                while (i < 500) {
                        // For each new grab, mesh data is updated
                        if (zed.grab() == SUCCESS) {
                                // In the background, spatial mapping will use newly retrieved images, depth and pose to update the mesh
                                sl::SPATIAL_MAPPING_STATE mapping_state = zed.getSpatialMappingState();

                                // Print spatial mapping state
                                std::cout << "Images captured: " << i << " / 500  ||  Spatial mapping state: " << spatialMappingState2str(mapping_state) << std::endl;
                                i++;
                        }
                }
                std::cout << std::endl;

                // Extract, filter and save the mesh in a obj file
                std::cout << "Extracting Mesh ..." << std::endl;
                zed.extractWholeMesh(mesh); // Extract the whole mesh
                std::cout << "Filtering Mesh ..." << std::endl;
                mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW); // Filter the mesh (remove unnecessary vertices and faces)
                std::cout << "Saving Mesh in mesh.obj ..." << std::endl;
                mesh.save("mesh.obj"); // Save the mesh in an obj file


                // Disable tracking and mapping and close the camera
                zed.disableSpatialMapping();
                zed.disablePositionalTracking();
                zed.close();
                return 0;
        }
        \endcode
         */
        ERROR_CODE enableSpatialMapping(SpatialMappingParameters spatial_mapping_parameters = SpatialMappingParameters());

        /**
        \brief Returns the current spatial mapping state.

        As the spatial mapping runs asynchronously, this function allows you to get reported errors or status info.

        \return The current state of the spatial mapping process
        \see SPATIAL_MAPPING_STATE
         */
        SPATIAL_MAPPING_STATE getSpatialMappingState();

        // -----------------------------------------------------------------
        // Async functions of spatial map generation ( *Async())
        // -----------------------------------------------------------------
        /**
        \brief Starts the spatial map generation process in a non blocking thread from the spatial mapping process.

        The spatial map generation can take a long time depending on the mapping resolution and covered area. This function will trigger the generation of a mesh without blocking the program.
        You can get info about the current generation using \ref getSpatialMapRequestStatusAsync(), and retrieve the mesh using \ref retrieveSpatialMapAsync(...) .

        \note Only one mesh can be generated at a time. If the previous mesh generation is not over, new calls of the function will be ignored.

        \code
        zed.requestSpatialMapAsync();
        while (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::FAILURE) {
        //Mesh is still generating
        }

        if (zed.getSpatialMapRequestStatusAsync() == SUCCESS) {
        zed.retrieveSpatialMapAsync(mesh);
        std::cout << "Number of triangles in the mesh: " << mesh.getNumberOfTriangles() << std::endl;
        }
        \endcode
         */
        void requestSpatialMapAsync();

        /**
        \brief Returns the spatial map generation status. This status allows to know if the mesh can be retrieved by calling \ref retrieveSpatialMapAsync.
        \return \ref ERROR_CODE "SUCCESS" if the mesh is ready and not yet retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \n See \ref requestSpatialMapAsync() for an example.
         */
        ERROR_CODE getSpatialMapRequestStatusAsync();

        /**
        \brief Retrieves the current generated spatial map.

        After calling \ref requestSpatialMapAsync , this function allows you to retrieve the generated mesh. The mesh will only be available when \ref getMeshRequestStatusAsync() returns \ref ERROR_CODE "SUCCESS"

        \param mesh : \b [out] The mesh to be filled with the generated spatial map.
        \return \ref ERROR_CODE "SUCCESS" if the mesh is retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \note This function only updates the necessary chunks and adds the new ones in order to improve update speed.
        \warning You should not modify the mesh between two calls of this function, otherwise it can lead to corrupted mesh.

        \n See \ref requestSpatialMapAsync() for an example.
         */
        ERROR_CODE retrieveSpatialMapAsync(Mesh& mesh);

        /**
        \brief Retrieves the current generated spatial map.

        After calling \ref requestSpatialMapAsync , this function allows you to retrieve the generated fused point cloud. The fused point cloud will only be available when \ref getMeshRequestStatusAsync() returns \ref ERROR_CODE "SUCCESS"

        \param fpc : \b [out] The fused point cloud to be filled with the generated spatial map.
        \return \ref ERROR_CODE "SUCCESS" if the fused point cloud is retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \note This function only updates the necessary chunks and adds the new ones in order to improve update speed.
        \warning You should not modify the fused point cloud between two calls of this function, otherwise it can lead to a corrupted fused point cloud.

        \n See \ref requestSpatialMapAsync() for an example.
         */
        ERROR_CODE retrieveSpatialMapAsync(FusedPointCloud& fpc);

        // -----------------------------------------------------------------
        // Blocking (synchronous) function of spatial map generation
        // -----------------------------------------------------------------
        /**
        \brief Extracts the current spatial map from the spatial mapping process.

        If the object to be filled already contains a previous version of the mesh, only changes will be updated, optimizing performance.

        \param mesh : \b [out] The mesh to be filled with the generated spatial map.
        \return \ref SUCCESS if the mesh is filled and available, otherwise \ref ERROR_CODE::FAILURE.

        \warning This is a blocking function. You should either call it in a thread or at the end of the mapping process.
        \n The extraction can be long, calling this function in the grab loop will block the depth and tracking computation giving bad results.

        \n See \ref enableSpatialMapping() for an example.
         */
        ERROR_CODE extractWholeSpatialMap(Mesh& mesh);

        /**
        \brief Extracts the current spatial map from the spatial mapping process.

        If the object to be filled already contains a previous version of the fused point cloud, only changes will be updated, optimizing performance.

        \param fpc : \b [out] The fused point cloud to be filled with the generated spatial map.
        \return \ref SUCCESS if the fused point cloud is filled and available, otherwise \ref ERROR_CODE::FAILURE.

        \warning This is a blocking function. You should either call it in a thread or at the end of the mapping process.
        \n The extraction can be long, calling this function in the grab loop will block the depth and tracking computation giving bad results.

        \n See \ref enableSpatialMapping() for an example.
         */
        ERROR_CODE extractWholeSpatialMap(FusedPointCloud& fpc);

        /**
        \brief Pauses or resumes the spatial mapping processes.

        As spatial mapping runs asynchronously, using this function can pause its computation to free some processing power, and resume it again later.
        \n For example, it can be used to avoid mapping a specific area or to pause the mapping when the camera is static.

        \param status : if true, the integration is paused. If false, the spatial mapping is resumed.
         */
        void pauseSpatialMapping(bool status);

        /**
        \brief Disables the spatial mapping process.

        The spatial mapping is immediately stopped.
        \n If the mapping has been enabled, this function will automatically be called by \ref close().

        \note This function frees the memory allocated for the spatial mapping, consequently, the spatial map cannot be retrieved after this call.
         */
        void disableSpatialMapping();

        /**
        \brief Returns the spatial mapping parameters used. Correspond to the structure send when the \ref enableSpatialMapping() function was called.

        \return \ref SpatialMappingParameters containing the parameters used for spatial mapping intialization.
         */
        SpatialMappingParameters getSpatialMappingParameters();

        ///@}



        /**
        \brief Checks the plane at the given left image coordinates.

         This function gives the 3D plane corresponding to a given pixel in the latest left image \ref grab() "grabbed".
         \n The pixel coordinates are expected to be contained x=[0;width-1] and y=[0;height-1], where width/height are defined by the input resolution.

        \param coord : \b [in] The image coordinate. The coordinate must be taken from the full-size image
        \param plane : \b [out] The detected plane if the function succeeded
         \return \ref ERROR_CODE "SUCCESS" if a plane is found otherwise \ref ERROR_CODE "ERROR_CODE::PLANE_NOT_FOUND" .
         \note The reference frame is defined by the \ref RuntimeParameters.measure3D_reference_frame given to the \ref grab() function.
         */
        ERROR_CODE findPlaneAtHit(sl::uint2 coord, sl::Plane& plane);

        /**
        \brief Detect the floor plane of the scene

         This function analysis the latest image and depth to estimate the floor plane of the scene.
         \n It expects the floor plane to be visible and bigger than other candidate planes, like a table.

        \param floorPlane : \b [out] The detected floor plane if the function succeeded
        \param resetTrackingFloorFrame : \b [out] The transform to align the tracking with the floor plane. The initial position will then
         * be at ground height, with the axis align with the gravity. The positional tracking needs to be reset/enabled
         * with this transform as a parameter (PositionalTrackingParameters.initial_world_transform)
        \param floor_height_prior : \b [in] Prior set to locate the floor plane depending on the known camera
         * distance to the ground, expressed in the same unit as the ZED. If the prior is too far from the detected floor plane,
         * the function will return ERROR_CODE::PLANE_NOT_FOUND
        \param world_orientation_prior : \b [in] Prior set to locate the floor plane depending on the known camera
         * orientation to the ground. If the prior is too far from the detected floor plane,
         * the function will return ERROR_CODE::PLANE_NOT_FOUND
        \param floor_height_prior_tolerance : \b [in] Prior height tolerance, absolute value.
        \return \ref ERROR_CODE "SUCCESS" if the floor plane is found and matches the priors (if defined),
         * otherwise \ref ERROR_CODE "ERROR_CODE::PLANE_NOT_FOUND"
        \note The reference frame is defined by the sl:RuntimeParameters (measure3D_reference_frame)
         * given to the grab() function. The length unit is defined by sl:InitParameters (coordinate_units).
         * With the ZED, the assumption is made that the floor plane is the dominant plane in the scene. The ZED Mini uses the gravity as prior.
         */
        ERROR_CODE findFloorPlane(sl::Plane& floorPlane, sl::Transform& resetTrackingFloorFrame,
                float floor_height_prior = INVALID_VALUE, sl::Rotation world_orientation_prior = sl::Matrix3f::zeros(),
                float floor_height_prior_tolerance = INVALID_VALUE);

        ///@}

        ///@{
        /// @name Recording
        // -----------------------------------------------------------------
        //                 		Recording functions
        // -----------------------------------------------------------------

        /**
        \brief Creates an SVO file to be filled by \ref record().


        \n SVO files are custom video files containing the un-rectified images from the camera along with some meta-data like timestamps or IMU orientation (if applicable).
        \n They can be used to simulate a live ZED and test a sequence with various SDK parameters.
        \n Depending on the application, various compression modes are available. See \ref SVO_COMPRESSION_MODE.

        \param recording_parameters : Recording parameters such as filename and compression mode
        \return an \ref ERROR_CODE that defines if SVO file was successfully created and can be filled with images.

        \warning This function can be called multiple times during ZED lifetime, but if video_filename is already existing, the file will be erased.

        \code
        #include <sl/Camera.hpp>

        using namespace sl;

        int main(int argc, char **argv) {

                // Create a ZED camera object
                Camera zed;

                // Set initial parameters
                InitParameters init_params;
                init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode (default fps: 60)
                init_params.coordinate_units = UNIT::METER; // Set units in meters

                // Open the camera
                ERROR_CODE err = zed.open(init_params);
                if (err != SUCCESS) {
                        std::cout << toString(err) << std::endl;
                        exit(-1);
                }

                // Enable video recording
                err = zed.enableRecording(RecordingParameters("myVideoFile.svo", SVO_COMPRESSION_MODE::H264));
                if (err != SUCCESS) {
                        std::cout << toString(err) << std::endl;
                        exit(-1);
                }

                // Grab data during 500 frames
                int i = 0;
                while (i < 500) {
                        // Grab a new frame
                        if (zed.grab() == SUCCESS) {
                                // Record the grabbed frame in the video file
                                i++;
                        }
                }

                zed.disableRecording();
                std::cout << "Video has been saved ..." << std::endl;

                zed.close();
                return 0;
        }
        \endcode
         */
        ERROR_CODE enableRecording(RecordingParameters recording_parameters);

        /**
        \brief Get the recording information
        \return The recording state structure. For more details, see \ref RecordingStatus.
         */
        RecordingStatus getRecordingStatus();

        /**
        \brief Pauses or resumes the recording.

        \param status : if true, the recording is paused. If false, the recording is resumed.
         */
        void pauseRecording(bool status);

        /**
        \brief Disables the recording initiated by \ref enableRecording() and closes the generated file.

        \note This function will automatically be called by \ref close() if enableRecording() was called.

        See \ref enableRecording() for an example.

         */
        void disableRecording();

        /**
        \brief Returns the recording parameters used. Correspond to the structure send when the \ref enableRecording() function was called.

        \return \ref RecordingParameters containing the parameters used for recording initialization.
         */
        RecordingParameters getRecordingParameters();
        ///@}

        ///@{
        /// @name Streaming
        // -----------------------------------------------------------------
        //                 		Streaming functions
        // -----------------------------------------------------------------
        /**
        \brief Creates a streaming pipeline.
        \param streaming_parameters : the structure containing all the specific parameters for the streaming.
        \code
                #include <sl/Camera.hpp>

                using namespace sl;

                int main(int argc, char **argv) {

                    // Create a ZED camera object
                    Camera zed;

                    // Set initial parameters
                    InitParameters init_params;
                    init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode (default fps: 60)
                    // Open the camera
                    ERROR_CODE err = zed.open(init_params);
                    if (err != SUCCESS) {
                    std::cout << toString(err) << std::endl;
                            exit(-1);
                        }

                        // Enable video recording
                        sl::StreamingParameters stream_params;
                        stream_params.port = 30000;
                        stream_params.bitrate = 8000;
                        err = zed.enableStreaming(stream_params);
                        if (err != SUCCESS) {
                            std::cout << toString(err) << std::endl;
                            exit(-1);
                        }

                        // Grab data during 500 frames
                        int i = 0;
                        while (i < 500) {
                            // Grab a new frame
                            if (zed.grab() == SUCCESS) {
                                i++;
                            }
                        }

                        zed.disableStreaming();
                        zed.close();
                        return 0;
                    }
         \endcode
        \return an \ref ERROR_CODE that defines if the stream was started.
        \n Possible Error Code :
        \n * SUCCESS if streaming was successfully started
        \n * ERROR_CODE::INVALID_FUNCTION_CALL if open() was not successfully called before.
        \n * ERROR_CODE::FAILURE if streaming RTSP protocol was not able to start.
        \n * ERROR_CODE::NO_GPU_COMPATIBLE if streaming codec is not supported (in this case, use H264 codec).
         */
        ERROR_CODE enableStreaming(StreamingParameters streaming_parameters = StreamingParameters());

        /**
        \brief Disables the streaming initiated by \ref enableStreaming()
        \note This function will automatically be called by \ref close() if enableStreaming() was called.
        See \ref enableStreaming() for an example.
         */
        void disableStreaming();

        /**
        \brief Tells if the streaming is running (true) or still initializing (false)
         */
        bool isStreamingEnabled();


        /**
        \brief Returns the streaming parameters used. Correspond to the structure send when the \ref enableStreaming() function was called.

        \return \ref StreamingParameters containing the parameters used for streaming initialization.
         */
        StreamingParameters getStreamingParameters();

        ///@}





        ///@{
        /// @name Object Detection
        // -----------------------------------------------------------------
        //                         Object Detection functions:
        // -----------------------------------------------------------------

        /**
        \brief Initializes and starts the object detection module.

        The object detection module will detect and track vehicles and people in range of the camera, the full list of detectable objects is available in \ref OBJECT_CLASS.

        Detected objects can be retrieved using the \ref retrieveObjects() function.

        As detecting and tracking the objects is CPU and GPU-intensive, the module can be used synchronously or asynchronously using \ref ObjectDetectionParameters::image_sync.
        - <b>Synchronous:</b> the \ref retrieveObjects() function will be blocking during the detection.
        - <b>Asynchronous:</b> the detection is running in the background, and \ref retrieveObjects() will immediately return the last objects detected.

        \param object_detection_parameters : Structure containing all specific parameters for object detection.
        \n For more information, see the \ref ObjectDetectionParameters documentation.
        \return
        \ref SUCCESS if everything went fine,
        \ref ERROR_CODE::OBJECT_DETECTION_NOT_AVAILABLE if the AI model is missing or corrupted. In this case, the SDK needs to be reinstalled.
        \ref ERROR_CODE::OBJECT_DETECTION_MODULE_NOT_COMPATIBLE_WITH_CAMERA if the camera used does not have a IMU (ZED Camera). the IMU gives the gravity vector that helps in the 3D box localization. Therefore the Object detection module is available only for ZED-M and ZED2 camera model.
        \ref ERROR_CODE::SENSORS_NOT_DETECTED if the camera model is correct (ZED2) but the IMU is missing. It probably happens because InitParameters::disable_sensors was set to true.
        \ref ERROR_CODE::INVALID_FUNCTION_CALL if one of the ObjectDetection parameter is not compatible with other modules parameters (For example, depth mode has been set to NONE).
        \ref ERROR_CODE::FAILURE otherwise.
        \note This feature uses AI to locate objects and requires a powerful GPU. A GPU with at least 3GB of memory is recommended.

        \code
        #include <sl/Camera.hpp>
        using namespace sl;
        int main(int argc, char **argv) {
            // Create a ZED camera object
            Camera zed;

            // Open the camera
            ERROR_CODE err = zed.open();
            if (err != SUCCESS) {
                std::cout << toString(err) << std::endl;
                exit(-1);
            }

            // Set the object detection parameters
            ObjectDetectionParameters object_detection_params;
            skeleton_params.image_sync = true;

            // Enable the object detection
            err = zed.enableObjectDetection(object_detection_params);
            if (err != SUCCESS) {
                std::cout << toString(err) << std::endl;
                exit(-1);
            }

            // Grab an image and detect objects on it
            Objects objects;
            while (true) {
                if (zed.grab() == SUCCESS) {
                    zed.retrieveObjects(objects);
                    std::cout << objects.object_list.size() << " objects detected " << std::endl;
                    // Use the objects in your application
                }
            }

            // Close the Camera
            zed.disableObjectDetection();
            zed.close();
            return 0;
        }
        \endcode
         */
        ERROR_CODE enableObjectDetection(ObjectDetectionParameters object_detection_parameters = ObjectDetectionParameters());

        /**
        \brief Pauses or resumes the object detection processes.

        If the object detection has been enabled with  \ref ObjectDetectionParameters::image_sync set to false (running asynchronously), this function will pause processing.

        While in pause, calling this function with <i>status = false</i> will resume the object detection.
        The \ref retrieveObjects function will keep on returning the last objects detected while in pause.

        \param status : If true, object detection is paused. If false, object detection is resumed.
         */
        void pauseObjectDetection(bool status);

        /**
        \brief Disables the Object Detection process.

        The object detection module immediately stops and frees its memory allocations.
        If the object detection has been enabled, this function will automatically be called by \ref close().
         */
        void disableObjectDetection();


        /**
        \brief Retrieve objects detected by the object detection module

        This function returns the result of the object detection, whether the module is running synchronously or asynchronously.

        - <b>Asynchronous:</b> this function immediately returns the last objects detected. If the current detection isn't done, the objects from the last detection will be returned, and \ref Objects::is_new will be set to false.
        - <b>Synchronous:</b> this function executes detection and waits for it to finish before returning the detected objects.

        It is recommended to keep the same \ref Objects object as the input of all calls to this function. This will enable the identification and the tracking of every objects detected.

        \param objects : [in,out] The detected objects will be saved into this object. If the object already contains data from a previous detection, it will be updated, keeping a unique ID for the same person.
        \param parameters : [in] Object detection runtime settings, can be changed at each detection. In async mode, the parameters update is applied on the next iteration.
         *
        \return \ref SUCCESS if everything went fine, \ref ERROR_CODE::FAILURE otherwise

        \code
        Objects objects; // Unique Objects to be updated after each grab
        // --- Main loop
        while (true) {
            if (zed.grab() == SUCCESS) { // Grab an image from the camera
                zed.retrieveObjects(objects);
                for (auto object : objects.object_list) {
                    std::cout << object.label << std::endl;
                }
            }
        }
        \endcode
         *
         */
        ERROR_CODE retrieveObjects(Objects &objects, ObjectDetectionRuntimeParameters parameters = ObjectDetectionRuntimeParameters());

        /**
        \brief Returns the object detection parameters used. Correspond to the structure send when the \ref enableObjectDetection() function was called.

        \return \ref ObjectDetectionParameters containing the parameters used for object detection initialization.
         */
        ObjectDetectionParameters getObjectDetectionParameters();

        ///@}


        // -----------------------------------------------------------------
        //                         (static)
        // -----------------------------------------------------------------
        /**
        \brief Returns the version of the currently installed ZED SDK.
        \return The ZED SDK version as a string with the following format : MAJOR.MINOR.PATCH

        \code
        std::cout << Camera::getSDKVersion() << std::endl;
        \endcode
         */
        static String getSDKVersion();


        /**
        \brief Returns the version of the currently installed ZED SDK.
        \return The ZED SDK version
         */
        static void getSDKVersion(int &major, int &minor, int &patch);

        /**
        \brief List all the connected devices with their associated information.

        This function lists all the cameras available and provides their serial number, models and other information.

        \return The device properties for each connected camera

        \warning As this function returns an std::vector, it is only safe to use in Release mode (not Debug).
        \n This is due to a known compatibility issue between release (the SDK) and debug (your app) implementations of std::vector.

         */
        static std::vector<sl::DeviceProperties> getDeviceList();

        /**
        \brief List all the streaming devices with their associated information.

        \return The streaming properties for each connected camera

        \warning As this function returns an std::vector, it is only safe to use in Release mode (not Debug).
        \n This is due to a known compatibility issue between release (the SDK) and debug (your app) implementations of std::vector.

        \warning This function takes around 2seconds to make sure all network informations has been captured. Make sure to run this function in a thread.

         */
        static std::vector<sl::StreamingProperties> getStreamingDeviceList();


        /**
        \brief Performs an hardware reset of the ZED 2.
        \param Serial number of the camera to reset, or 0 to reset the first camera detected.
        \return \ref SUCCESS if everything went fine, \ref ERROR_CODE::CAMERA_NOT_DETECTED if no camera was detected, \ref ERROR_CODE::FAILURE  otherwise.
        \note This function only works for ZED 2 cameras.
        \warning This function will invalidate any sl::Camera object, since the device is rebooting.
         */
        static sl::ERROR_CODE reboot(int sn);

        /**
        \brief The \ref Camera object cannot be copied. Therfore, its copy constructor is disabled.
        If you need to share a Camera instance across several threads or object, please consider using a pointer.

        \see Camera()
         */
        Camera(const Camera&) = delete;
    private:
        CameraMemberHandler* h = 0;
        bool opened = false;

    };

    ///@cond SHOWHIDDEN
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SpatialMappingParameters::MAPPING_RESOLUTION& resolution);

    inline std::ostream& operator<<(std::ostream& os, const SpatialMappingParameters::MAPPING_RESOLUTION& resolution) {
        return os << toString(resolution);
    }

    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SpatialMappingParameters::MAPPING_RANGE& range);

    inline std::ostream& operator<<(std::ostream& os, const SpatialMappingParameters::MAPPING_RANGE& range) {
        return os << toString(range);
    }

    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SpatialMappingParameters::SPATIAL_MAP_TYPE& map_type);

    inline std::ostream& operator<<(std::ostream& os, const SpatialMappingParameters::SPATIAL_MAP_TYPE& map_type) {
        return os << toString(map_type);
    }
    ///@endcond

}

#endif /* __CAMERA_HPP__ */
