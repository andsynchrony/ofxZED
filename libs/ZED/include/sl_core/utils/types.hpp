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

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>

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
#define SL_DEPRECATED(func) __declspec(deprecated) func
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 
#endif
#include <Windows.h>
#define __CUSTOM__PRETTY__FUNC__ __FUNCSIG__
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#elif __GNUC__
#define SL_DEPRECATED(func) func __attribute__ ((deprecated))
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

    /// \defgroup Camera_group Camera classes
    /// \defgroup Core_group 
    /// \defgroup SpatialMapping_group 
    /// \defgroup PositionalTracking_group 

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
    \enum UNIT
    \ingroup Camera_group
    \brief Lists available unit for measures.
     */
    enum UNIT {
        UNIT_MILLIMETER, /**< International System, 1/1000 METER. */
        UNIT_CENTIMETER, /**< International System, 1/100 METER. */
        UNIT_METER, /**< International System, 1 METER */
        UNIT_INCH, /**< Imperial Unit, 1/12 FOOT */
        UNIT_FOOT, /**< Imperial Unit, 1 FOOT */
        UNIT_LAST
    };

    /**
    \ingroup Camera_group
    \brief Converts the given enum to a readable string
    \return The enum value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const UNIT &unit);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const UNIT &unit) {
        return os << toString(unit);
    }
    ///@endcond

    /**
    \enum COORDINATE_SYSTEM
    \ingroup Camera_group
    \brief Lists available coordinates systems for positional tracking and 3D measures.
     */
    enum COORDINATE_SYSTEM {
        COORDINATE_SYSTEM_IMAGE, /**< Standard coordinates system in computer vision. Used in OpenCV : see here : http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html */
        COORDINATE_SYSTEM_LEFT_HANDED_Y_UP, /**< Left-Handed with Y up and Z forward. Used in Unity with DirectX. */
        COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP, /**< Right-Handed with Y pointing up and Z backward. Used in OpenGL. */
        COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP, /**< Right-Handed with Z pointing up and Y forward. Used in 3DSMax. */
        COORDINATE_SYSTEM_LEFT_HANDED_Z_UP, /**< Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine. */
        COORDINATE_SYSTEM_LAST
    };

    /**
    \ingroup Camera_group
    \brief Converts the given enum to a readable string
    \return The enum value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const COORDINATE_SYSTEM &coord_system);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const COORDINATE_SYSTEM &coord_system) {
        return os << toString(coord_system);
    }
    ///@endcond

    /**
    \enum ERROR_CODE
    \ingroup Camera_group
    \brief Lists error codes in the ZED SDK.
     */
    enum ERROR_CODE {
        SUCCESS, /**< Standard code for successful behavior.*/
        ERROR_CODE_FAILURE, /**< Standard code for unsuccessful behavior.*/
        ERROR_CODE_NO_GPU_COMPATIBLE, /**< No GPU found or CUDA capability of the device is not supported.*/
        ERROR_CODE_NOT_ENOUGH_GPUMEM, /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE).*/
        ERROR_CODE_CAMERA_NOT_DETECTED, /**< The ZED camera is not plugged or detected.*/
        ERROR_CODE_SENSOR_NOT_DETECTED, /**< a ZED-M camera is detected but the sensor (imu) cannot be opened. Only for ZED-M device*/
        ERROR_CODE_INVALID_RESOLUTION, /**< For Nvidia Jetson X1 only, resolution not yet supported (USB3.0 bandwidth).*/
        ERROR_CODE_LOW_USB_BANDWIDTH, /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
        ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE, /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
        ERROR_CODE_INVALID_CALIBRATION_FILE, /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
        ERROR_CODE_INVALID_SVO_FILE, /**< The provided SVO file is not valid.*/
        ERROR_CODE_SVO_RECORDING_ERROR, /**< An recorder related error occurred (not enough free storage, invalid file).*/
        ERROR_CODE_INVALID_COORDINATE_SYSTEM, /**< The requested coordinate system is not available.*/
        ERROR_CODE_INVALID_FIRMWARE, /**< The firmware of the ZED is out of date. Update to the latest version.*/
        ERROR_CODE_INVALID_FUNCTION_PARAMETERS, /**< An invalid parameter has been set for the function. */
        ERROR_CODE_NOT_A_NEW_FRAME, /**< In grab() only, the current call return the same frame as last call. Not a new frame.*/
        ERROR_CODE_CUDA_ERROR, /**< In grab() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.*/
        ERROR_CODE_CAMERA_NOT_INITIALIZED, /**< In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.*/
        ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE, /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
        ERROR_CODE_INVALID_FUNCTION_CALL, /**< The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open. */
        ERROR_CODE_CORRUPTED_SDK_INSTALLATION, /**< The SDK wasn't able to load its dependencies, the installer should be launched. */
        ERROR_CODE_INCOMPATIBLE_SDK_VERSION, /**< The installed SDK is incompatible SDK used to compile the program. */
        ERROR_CODE_INVALID_AREA_FILE, /**< The given area file does not exist, check the path. */
        ERROR_CODE_INCOMPATIBLE_AREA_FILE, /**< The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set. */
        ERROR_CODE_CAMERA_FAILED_TO_SETUP, /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
        ERROR_CODE_CAMERA_DETECTION_ISSUE, /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
        ERROR_CODE_CAMERA_ALREADY_IN_USE, /**< The Camera is already used by another process.*/
        ERROR_CODE_NO_GPU_DETECTED, /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
        ERROR_CODE_PLANE_NOT_FOUND, /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
        ERROR_CODE_LAST
    };

    /**
    \ingroup Camera_group
    \brief Converts the given enum to a readable string
    \return The enum value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const ERROR_CODE &errorCode);

    /**
    \ingroup Camera_group
    \brief Return a text explaining how to fix the given ERROR_CODE.
    \return A string of advice for the user.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toVerbose(const ERROR_CODE &errorCode);


    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const ERROR_CODE &errorCode) {
        return os << toString(errorCode);
    }
    ///@endcond

    /**
    \enum MODEL
    \ingroup Camera_group
    \brief Lists compatible ZED Camera model
     */
    enum MODEL {
        MODEL_ZED, /**< Defines ZED Camera model */
        MODEL_ZED_M, /**<  Defines ZED Mini (ZED-M) Camera model */
        MODEL_LAST
    };

    /**
    \ingroup Camera_group
    \brief Converts the given enum to a readable string
    \return The enum value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const MODEL &model);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const MODEL &model) {
        return os << toString(model);
    }
    ///@endcond

    /**
    \enum CAMERA_STATE
    \ingroup Camera_group
    \brief List of possible camera state
     */
    enum CAMERA_STATE {
        CAMERA_STATE_AVAILABLE, /**< Defines if the camera can be opened by the SDK */
        CAMERA_STATE_NOT_AVAILABLE, /**<  Defines if the camera is already opened and unavailable*/
        CAMERA_STATE_LAST
    };

    /**
    \ingroup Camera_group
    \brief Converts the given enum to a readable string
    \return The enum value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const CAMERA_STATE &camera_state);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const CAMERA_STATE &camera_state) {
        return os << toString(camera_state);
    }
    ///@endcond

    /**
   \struct DeviceProperties
   \ingroup Camera_group
   \brief Properties of a camera

   \note A camera_model MODEL_ZED_M with an id '-1' can be due to an inverted USB-C cable.

   \warning Experimental on Windows.
     */
    struct DeviceProperties {
        /**
        the camera state
         */
        sl::CAMERA_STATE camera_state = sl::CAMERA_STATE_NOT_AVAILABLE;

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
        sl::MODEL camera_model = sl::MODEL_LAST;

        /**
        the camera serial number
        \n Not provided for Windows
         */
        unsigned int serial_number = 0;
    };

    /**
    \ingroup Camera_group
    \brief Converts the given struct to a readable string
    \return The struct informations as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const DeviceProperties &properties);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const DeviceProperties &properties) {
        return os << toString(properties);
    }
    ///@endcond

    /**
    \class InputType
    \ingroup Camera_group
    \brief Defines the input type used in the ZED SDK. Can be used to select a specific camera with ID or serial number, or a svo file

    \note This replaces the previous InitParameters::camera_linux_id and InitParameters::svo_input_filename (now deprecated).
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

    private:

        enum INPUT_TYPE {
            INPUT_TYPE_ID,
            INPUT_TYPE_SERIAL,
            INPUT_TYPE_SVO_FILE,
            INPUT_TYPE_LAST
        };

        INPUT_TYPE input_type = INPUT_TYPE_ID;
        unsigned int serial_number = 0;
        unsigned int id = 0;
        sl::String svo_input_filename;
    };



    ///@{
    ///  @name Enumeration conversion

    /*!
    \ingroup Camera_group
    \brief Converts the given MODEL into a string
    \param model : a specific MODEL
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ model2str(MODEL model)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Camera_group
    \brief Converts the given ERROR_CODE into a string
    \param err : a specific ERROR_CODE
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ errorCode2str(ERROR_CODE errorCode)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Camera_group
    \brief Converts the given UNIT into a string
    \param unit : a specific UNIT
    \return The corresponding string

    \deprecated See \ref toString.
     */
    /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ unit2str(UNIT unit)/*@cond SHOWHIDDEN*/)/*@endcond*/;

    /*!
    \ingroup Camera_group
    \brief Converts the given string into a UNIT
    \param unit : a specific unit string
    \return The corresponding UNIT
     */

    UNIT /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ str2unit(String unit);
    ///@}

    /*!
    \ingroup Core_group
    \brief Tells the program to wait for x ms.
    \param time : the number of ms to wait.
     */
    inline void sleep_ms(int time) {
        std::this_thread::sleep_for(std::chrono::milliseconds(time));
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
    \brief Represents a generic three-dimensional matrix

    It is defined in a row-major order, it means that, in the value buffer, the entire first row is stored first, followed by the entire second row, and so on.
    You can access the data with the 'r' ptr or by element attribute.
    \f[\begin{bmatrix}
    r00 & r01 & r02 \\
    r10 & r11 & r22 \\
    r20 & r21 & r22
    \end{bmatrix} \f]
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
        \param rotation : the Matrix3f to copy.
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
        \return A \ref String containing the components of the current Matix3f.
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
    \brief Represents a generic fourth-dimensional matrix.

    sIt is defined in a row-major order, it means that, in the value buffer, the entire first row is stored first, followed by the entire second row, and so on.
    You can access the data by the 'm' ptr or by the element attribute.
    \f[\begin{bmatrix}
    r00 & r01 & r02 & tx \\
    r10 & r11 & r22 & ty \\
    r20 & r21 & r22 & tz \\
    m30 & m31 & m32 & m33
    \end{bmatrix} \f]
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
        \param rotation : the Matrix4f to copy.
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
        \return SUCCESS if the inverse has been computed, ERROR_CODE_FAILURE is not (det = 0).
         */
        ERROR_CODE inverse();

        /**
        \brief Creates the inverse of a Matrix4f.
        \param rotation : the Matrix4f to compute the inverse from.
        \return The inverse of the given Matrix4f.
         */
        static Matrix4f inverse(const Matrix4f &mat);

        /**
        \brief Sets the Matrix4f to its transpose.
         */
        void transpose();

        /**
        \brief Creates the transpose of a Matrix4f.
        \param rotation : the Matrix4f to compute the transpose from.
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
        \param \ref Matrix3f  : sub matrix to put inside the Matrix4f.
        \param row : index of the row to start the 3x3 block. Must be 0 or 1.
        \param column : index of the column to start the 3x3 block. Must be 0 or 1.
        \return SUCCESS if everything went well, ERROR_CODE_FAILURE otherwise.
         */
        ERROR_CODE setSubMatrix3f(Matrix3f input, int row = 0, int column = 0);

        /**
        \brief Sets a 3x1 Vector inside the Matrix4f at the specified column index.
        \note Can be used to set the Translation/Position matrix when the matrix4f is a pose or an isometry.
        \param \ref Vector3  : sub vector to put inside the Matrix4f.
        \param column : index of the column to start the 3x3 block. By default, it is the last column (translation for a Pose).
        \return SUCCESS if everything went well, ERROR_CODE_FAILURE otherwise.
         */
        ERROR_CODE setSubVector3f(Vector3<float> input, int column = 3);

        /**
        \brief Sets a 4x1 Vector inside the Matrix4f at the specified column index.
        \note Can be used to set the Translation/Position matrix when the matrix4f is a pose or an isometry.
        \param \ref Vector4  : sub vector to put inside the Matrix4f.
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

        _FCT_CPU_GPU_ const T *ptr() const {
            return &this->v[0];
        }

        inline _FCT_CPU_GPU_ Vector3<T> &setValues(const T *b) {
            this->x = b[0];
            this->y = b[1];
            this->z = b[2];
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

#ifndef timeStamp
    typedef uint64_t/*unsigned long long*/ timeStamp;
#endif
    ///@}

    /// @cond

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector2<T> &v2) {
        os << v2.x << " " << v2.y << "\n";
        return os;
    }

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector3<T> &v3) {
        os << v3.x << " " << v3.y << " " << v3.z << "\n";
        return os;
    }

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const Vector4<T> &v4) {
        os << v4.x << " " << v4.y << " " << v4.z << " " << v4.w << "\n";
        return os;
    }

    /**
    \ingroup Core_group
    \brief Returns the current timestamp at the time the function is called. Can be compared to  sl::Camera::getCameraTimestamp for synchronization.

    Use this function to compare the current timestamp and the camera timestamp, since they have the same reference (Computer start time).
    \return The current timestamp in ns.
     */
    static inline timeStamp getCurrentTimeStamp() {
        timeStamp current_ts = 0ULL;
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
