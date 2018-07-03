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

#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include "sl_core/utils/Core.hpp"
#include "sl_core/mapping/Mesh.hpp"
#include "sl_zed/defines.hpp"
#include <cuda.h>

// Stereolabs namespace
namespace sl {

    /**
    \class InitParameters
    \ingroup Video_group
    \brief Parameters that will be fixed for the whole execution life time of the \ref Camera.

    By default, it opens the ZED camera in live mode at RESOLUTION_HD720 and set the depth mode to DEPTH_MODE_PERFORMANCE to get low computation time.
    \n You can customize it to fit your application and then save it to create a preset that can be loaded for further executions.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ InitParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        Define the chosen camera resolution.
        \n default : \ref RESOLUTION_HD720
         */
        RESOLUTION camera_resolution;

        /**
        Requested camera frame rate. If set to 0, the highest FPS of the specified camera_resolution will be used, see \ref RESOLUTION.
        \n default : 0
        \note If the requested camera_fps is unsuported, the closest available FPS will be used.
         */
        int camera_fps;

        /**
        Defines if the image are horizontally flipped.
        \n default : false
         */
        int camera_image_flip;

        /**
        If set to true, it will disable self-calibration and take the optional calibration parameters without optimizing them.
        \n It is advised to leave it as false, so that calibration parameters can be optimized.
        \n default : false
         */
        bool camera_disable_self_calib;

        /**
        Defines if right MEASURE should be computed (needed for MEASURE_<XXX>_RIGHT)
        \n default : false
         */
        bool enable_right_side_measure;

        /**
        ONLY for LINUX : Set the number of buffers in the internal grabbing process.
        \n Decrease this number may reduce latency but can also produce more corrupted frames.
        \n default:  4

        \warning Linux Only, this parameter has no effect on Windows.
         */
        int camera_buffer_count_linux;

        /**
        ONLY for LINUX : in case if you have multiples ZED connected, use this parameter to specify the ZED you want to use.
        \n See \ref Camera::getDeviceList to know how ZED are connected and to have the relationship between id/serial number and system path.
        \n Each ZED will create its own memory (CPU and GPU), therefore the number of ZED available will depend on the configuration of your computer.
        \n default : 0

        \deprecated Use InitParameters.input.setFromCameraID() instead
        \warning Notice that only the camera with id 0 can be used on Windows (other values will lead to an error).
         */
        int camera_linux_id;

        /**
        Path with filename to the recorded SVO file.
        \n default : (empty)
        \deprecated Use InitParameters.input.setFromSVOFile() instead
         */
        String svo_input_filename;

        /**
        When enabled the timestamp is taken as reference to determine the reading framerate.
        \n This mode simulates the live camera and consequently skipped frames if the computation framerate is too slow.
        \n default : false
         */
        bool svo_real_time_mode;

        /**
        Defines the quality of the depth map, affects the level of details and also the computation time.
        \n default : \ref DEPTH_MODE_PERFORMANCE
         */
        DEPTH_MODE depth_mode;

        /**
        Defines if the depth map should be stabilized.
        \n This requires the positional tracking data, it will be enabled automatically if needed.
        \n default : true
         */
        int depth_stabilization;

        /**
        Specify the minimum depth value (from the camera) that will be computed, in the \ref UNIT you define.
        \n In case of limited computation power, consider increasing the value.
        \n default : (-1) corresponding to 700 mm for a ZED and 200 mm for ZED-M.

        \note With a ZED camera you can decrease this value to 300 mm whereas you can set it to 100 mm using a ZED-M. In any case it can not be greater than 3.0000 mm
        \warning The computation time is affected by the value. The smaller it gets the longer the \ref grab will take.
         */
        float depth_minimum_distance;

        /**
        Define the unit for all the metric values ( depth, point cloud, tracking, mesh).
        \n default : \ref UNIT_MILLIMETER
         */
        UNIT coordinate_units;

        /**
        Define the coordinate system of the world frame (and the camera frame as well).
        \n This defines the order and the direction of the axis of the coordinate system. see COORDINATE_SYSTEM for more information.
        \n default : \ref COORDINATE_SYSTEM_IMAGE
         */
        COORDINATE_SYSTEM coordinate_system;

        /**
        Defines the graphics card on which the computation will be done.
        \n default : -1
        \n A non positive value will search for all CUDA capable device and select the most powerful.
         */
        CUdevice sdk_gpu_id;

        /**
        \brief Defines if you want the SDK provides text feedback in the console.
        \n default : false
        \n If set to true, it will output some information about the current status of initialization.
         */
        bool sdk_verbose;

        /**
        Store the program outputs into the log file defined by its filename.
        \n default : (empty)

        \note it will redirect std::cout calls for the program using the ZED SDK (including SDK verbosity outputs) in the log file.
        \note Can be used with Unreal, Unity or any software that doesn't have a standard console output.
         */
        String sdk_verbose_log_file;

        /**
        \brief Set your own CUDA context.
        \n default : (empty)

        If your application as already a CUDA Context you can share it with the ZED SDK by setting this parameter.

        \note When creating you own CUDA context you have to define the device you will use, do not forget to also specify it on \ref sdk_gpu_id.
        \note ONLY FOR JETSON : you have to set the flag CU_CTX_SCHED_YIELD, during CUDA context creation.
         */
        CUcontext sdk_cuda_ctx;

        /**
        Specify the input type (live from id/sn or svo with filename) that will be used. See InputType for more information.
        \n default : (empty)
        */
        InputType input;

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        InitParameters(RESOLUTION camera_resolution_ = RESOLUTION_HD720,
                int camera_fps_ = 0,
                int camera_linux_id_ = 0,
                String svo_input_filename_ = String(),
                bool svo_real_time_mode_ = false,
                DEPTH_MODE depth_mode_ = DEPTH_MODE_PERFORMANCE,
                UNIT coordinate_units_ = UNIT_MILLIMETER,
                COORDINATE_SYSTEM coordinate_system_ = COORDINATE_SYSTEM_IMAGE,
                bool sdk_verbose_ = false,
                int sdk_gpu_id_ = -1,
                float depth_minimum_distance_ = -1.,
                bool camera_disable_self_calib_ = false,
                bool camera_image_flip_ = false,
                bool enable_right_side_measure_ = false,
                int camera_buffer_count_linux_ = 4,
                String sdk_verbose_log_file_ = String(),
                int depth_stabilization_ = 1,
                CUcontext sdk_cuda_ctx_ = CUcontext(),
				InputType input_type = InputType())
        : camera_resolution(camera_resolution_)
        , camera_fps(camera_fps_)
        , camera_linux_id(camera_linux_id_)
        , svo_input_filename(svo_input_filename_)
        , svo_real_time_mode(svo_real_time_mode_)
        , depth_mode(depth_mode_)
        , coordinate_units(coordinate_units_)
        , coordinate_system(coordinate_system_)
        , sdk_verbose(sdk_verbose_)
        , sdk_gpu_id(sdk_gpu_id_)
        , depth_minimum_distance(depth_minimum_distance_)
        , camera_disable_self_calib(camera_disable_self_calib_)
        , camera_image_flip(camera_image_flip_)
        , enable_right_side_measure(enable_right_side_measure_)
        , camera_buffer_count_linux(camera_buffer_count_linux_)
        , sdk_verbose_log_file(sdk_verbose_log_file_)
        , depth_stabilization(depth_stabilization_)
        , sdk_cuda_ctx(sdk_cuda_ctx_)
        , input(input_type) {
        }
        
        /**
        \brief Saves the current set of parameters into a file.
        \param filename : the path to the file in which the parameters will be stored.
        \return True if file was successfully saved, otherwise false.
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
        \n default : \ref SENSING_MODE_STANDARD
         */
        SENSING_MODE sensing_mode;
        /**
        Provides 3D measures (point cloud and normals) in the desired reference frame (default is REFERENCE_FRAME_CAMERA)
        \n default : \ref REFERENCE_FRAME_CAMERA

        \note : replaces previous move_point_cloud_to_world_frame parameter.
         */
        REFERENCE_FRAME measure3D_reference_frame;

        /**
        Defines if the depth map should be computed.
        \n If false, only the images are available.
        \n default : true
         */
        bool enable_depth;

        /**
        Defines if the point cloud should be computed (including XYZRGBA).
        \n default : true

        \deprecated Point cloud is now enabled when depth is.
         */
        bool enable_point_cloud;

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        RuntimeParameters(SENSING_MODE sensing_mode_ = SENSING_MODE_STANDARD,
                bool enable_depth_ = true, bool enable_point_cloud_ = true, REFERENCE_FRAME measure3D_reference_frame_ = REFERENCE_FRAME_CAMERA)
        : sensing_mode(sensing_mode_)
        , enable_depth(enable_depth_)
        , enable_point_cloud(enable_point_cloud_)
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
    \class TrackingParameters
    \ingroup PositionalTracking_group
    \brief Parameters for positional tracking initialization.

    A default constructor is enabled and set to its default parameters.
    \n You can customize it to fit your application and then save it to create a preset that can be loaded for further executions.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ TrackingParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        /**
        Position of the camera in the world frame when camera is started. By default it should be identity.
        \n Use this \ref Transform to place the camera frame in the world frame.
        \n default : Identity matrix

        \note The camera frame (defines the reference frame for the camera) is by default positioned at the world frame when tracking is started.
         */
        Transform initial_world_transform;

        /**
        This mode enables the camera to learn and remember its surroundings. This helps correct positional tracking drift and position different cameras relative to each other in space.
        \n default : true

        \warning : This mode requires few resources to run and greatly improves tracking accuracy. We recommend to leave it on by default.
         */
        bool enable_spatial_memory;


        /**
        This mode enables smooth pose correction for small drift correction.
        \n default : false
         */
        bool enable_pose_smoothing;

        /**
        Area localization file that describes the surroundings (previously saved).
        \n default : (empty)

        \note Loading an area file will start a searching phase during which the camera will try to position itself in the previously learned area.

        \warning : The area file describes a specific location. If you are using an area file describing a different location, the tracking function will continuously search for a position and may not find a correct one.
        \warning The '.area' file can only be used with the same depth mode (\ref MODE) as the one used during area recording.
         */
        String area_file_path;

        /**
        \brief  Default constructor, set all parameters to their default and optimized values.
         */
        TrackingParameters(Transform init_pos = Transform(), bool _enable_memory = true, bool _enable_pose_smoothing = false, String _area_path = String())
        : initial_world_transform(init_pos)
        , enable_spatial_memory(_enable_memory)
        , enable_pose_smoothing(_enable_pose_smoothing)
        , area_file_path(_area_path) {
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

    /**
    \class SpatialMappingParameters
    \ingroup SpatialMapping_group
    \brief Sets the spatial mapping parameters.

    A default constructor is enabled and set to its default parameters.
    \n You can customize it to fit your application and then save it to create a preset that can be loaded for further executions.

    \note Parameters can be user adjusted.
     */
    struct /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ SpatialMappingParameters {
        friend class CameraMemberHandler;
        friend class Camera;
        ZED_SDK_VERSION_ATTRIBUTE
    public:
        typedef std::pair<float, float> interval;

        /**
        \enum MAPPING_RESOLUTION
        \ingroup SpatialMapping_group
        \brief List the spatial mapping resolution presets.
         */
        enum MAPPING_RESOLUTION {
            MAPPING_RESOLUTION_HIGH, /**< Create a detail geometry, requires lots of memory.*/
            MAPPING_RESOLUTION_MEDIUM, /**< Smalls variations in the geometry will disappear, useful for big object*/
            MAPPING_RESOLUTION_LOW /**< Keeps only huge variations of the geometry , useful outdoor.*/
        };

        /**
        \enum MAPPING_RANGE
        \ingroup SpatialMapping_group
        \brief List the spatial mapping depth range presets.
         */
        enum MAPPING_RANGE {
            MAPPING_RANGE_NEAR, /**< Only depth close to the camera will be used during spatial mapping.*/
            MAPPING_RANGE_MEDIUM, /**< Medium depth range.*/
            MAPPING_RANGE_FAR /**< Takes into account objects that are far, useful outdoor.*/
        };

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        SpatialMappingParameters(MAPPING_RESOLUTION resolution = MAPPING_RESOLUTION_HIGH,
                MAPPING_RANGE range = MAPPING_RANGE_MEDIUM,
                int max_memory_usage_ = 2048,
                bool save_texture_ = true,
                bool use_chunk_only_ = false,
                bool reverse_vertex_order_ = false) {
            max_memory_usage = max_memory_usage_;
            save_texture = save_texture_;
            use_chunk_only = use_chunk_only_;
            reverse_vertex_order = reverse_vertex_order_;
            set(resolution);
            set(range);
        }

        /**
        \brief Return the resolution corresponding to the given \ref RESOLUTION preset.
        \param resolution : the desired \ref RESOLUTION. default : \ref RESOLUTION_HIGH.
        \return The resolution in meter.
         */
        static float get(MAPPING_RESOLUTION mapping_resolution = MAPPING_RESOLUTION_HIGH) {
            float resolution_m = 0.5f;
            switch (mapping_resolution) {
                case MAPPING_RESOLUTION_HIGH:
                    resolution_m = 0.02f;
                    break;
                case MAPPING_RESOLUTION_MEDIUM:
                    resolution_m = 0.05f;
                    break;
                case MAPPING_RESOLUTION_LOW:
                    resolution_m = 0.08f;
                    break;
                default:
                    resolution_m = 0.05f;
                    break;
            }
            return resolution_m;
        }

        /**
        \brief Sets the resolution corresponding to the given \ref RESOLUTION preset.
        \param resolution : the desired \ref RESOLUTION.  default :\ref  RESOLUTION_HIGH.
         */
        void set(MAPPING_RESOLUTION mapping_resolution = MAPPING_RESOLUTION_HIGH) {
            resolution_meter = get(mapping_resolution);
        }

        /**
        \brief  Return the maximum value of depth corresponding to the given \ref RANGE presets.
        \param range : the desired \ref RANGE. default : \ref RANGE_MEDIUM.
        \return The maximum value of depth.
         */
        static float get(MAPPING_RANGE mapping_range = MAPPING_RANGE_MEDIUM) {
            float range_max = 5.;
            switch (mapping_range) {
                case MAPPING_RANGE_NEAR:
                    range_max = 3.5f;
                    break;
                case MAPPING_RANGE_MEDIUM:
                    range_max = 5.f;
                    break;
                case MAPPING_RANGE_FAR:
                    range_max = 10.f;
                    break;
                default:
                    range_max = 5.f;
                    break;
            }
            return range_max;
        }

        /**
        \brief Sets the maximum value of depth corresponding to the given \ref RANGE presets.
        \param range : the desired \ref RANGE. default : \ref RANGE_MEDIUM.
         */
        void set(MAPPING_RANGE mapping_range = MAPPING_RANGE_MEDIUM) {
            range_meter = get(mapping_range);
        }

        /**
        \brief Spatial mapping resolution in meters, should fit \ref allowed_resolution.
         */
        float resolution_meter = 0.03f;

        /**
        \brief The resolutions allowed by the spatial mapping.
         */
        const interval allowed_resolution = std::make_pair(0.01f, 0.2f);

        /**
        \brief Depth range in meters.

        Can be different from the value set by \ref setDepthMaxRangeValue.
         */
        float range_meter = 5.f;

        /**
        \brief Range of the maximal depth value allowed by the spatial mapping.
         */
        const interval allowed_range = std::make_pair(2.f, 20.f);

        /**
        \brief Set to true if you want to be able to apply the texture to your mesh after its creation.

        \note This option will take more memory.
         */
        bool save_texture = true;

        /**
        \brief Set to false if you want to keep consistency between the mesh and its inner chunks data.

        \note Updating the Mesh is time consuming, consider using only Chunks data for better performances.
         */
        bool use_chunk_only = false;

        /**
        \brief The maximum CPU memory (in mega bytes) allocated for the meshing process.
         */
        int max_memory_usage = 2048;

        /**
        \brief Specify if the order of the vertices of the triangles needs to be inverted. If your display process does not handle front and back face culling you can use this to set it right.
         */
        bool reverse_vertex_order = false;

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
    class CameraMemberHandler;

    /**
    \class Camera
    \ingroup Video_group
    \brief This class provides low-level access to the camera and modules of the ZED API: Video, Depth, Tracking and Mapping.
     *
     */
    class /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ Camera {
        friend CameraMemberHandler;
        ZED_SDK_VERSION_ATTRIBUTE

    public:
        /**
        \brief Default constructor which creates an empty Camera.
         */
        Camera();

        /**
        \brief Camera destructor.
         */
        ~Camera();

        /**
        \brief Opens the ZED camera in the desired mode (live/SVO), sets all the defined parameters, checks hardware requirements and launch internal self calibration.

        Every other code indicates an error and the program should be stopped.

        \param init_parameters : a structure containing all the individual parameters. default : a preset of InitParameters.
        \return An error code giving information about the internal process, if \ref SUCCESS is returned, the camera is ready to use.
         */
        ERROR_CODE open(InitParameters init_parameters = InitParameters());

        /**
        \brief Tests if the camera is opened and running
        \return true if the ZED is already setup, otherwise false.
         */
        inline bool isOpened() {
            return opened;
        }

        /**
        \brief Closes the camera and frees the memory.

        Disable calls to any Camera functions. \ref open can then be called again to reset the camera if needed.
         */
        void close();


        /**
        \brief Grabs a new image, rectifies it and computes the depth map.

        This function is typically called in the main loop.

        \param rt_parameters : a structure containing all the individual parameters. default : a preset of \ref RuntimeParameters.
        \return An \ref SUCCESS if no problem was encountered.
         */
        ERROR_CODE grab(RuntimeParameters rt_parameters = RuntimeParameters());

        /**
        \brief Returns camera information (calibration parameters, serial number and current firmware version).

        It also returns the ZED Serial Number (as uint) (Live or SVO) and the ZED Firmware version (as uint), 0 if the ZED is not connected.

        \param image_size : You can specify a size different from default image size to get the scaled camera information. default = (0,0) meaning original image size.
        \return CameraInformation containing the calibration parameters of the ZED, as well as serial number and firmware version.
         */
        CameraInformation getCameraInformation(Resolution image_size = Resolution(0, 0));

        /**
        \brief Returns the CUDA context used for GPU calls. Useful if your application is using CUDA.
        \return The CUDA context created by the inner process.
         */
        CUcontext getCUDAContext();

        ///@{
        /// @name Video Functions
        // -----------------------------------------------------------------
        //                         Video :
        // -----------------------------------------------------------------

        /**
        \brief Returns the current image size.
        \return The image resolution.
         */
        Resolution getResolution();

        /**
        \brief Retrieves the desired image. Multiple views are available.

        The retrieve function should be called after the function \ref grab.

        \param mat : [out] the \ref Mat to store the image.
        \param view  : defines the image you want (see \ref VIEW). default : VIEW_LEFT.
        \param type : the type of the memory of provided mat that should be used. default : MEM_CPU.
        \param width : if specified, define the width of the output mat. If set to 0, the width of the ZED resolution will be taken. default : 0.
        \param height : if specified, define the height of the output mat. If set to 0, the height of the ZED resolution will be taken. default : 0.
        \return SUCCESS if the method succeeded, ERROR_CODE_FAILURE if an error occurred.
         */
        ERROR_CODE retrieveImage(Mat &mat, VIEW view = VIEW_LEFT, MEM type = MEM_CPU, int width = 0, int height = 0);

        /**
        \brief Returns the current value of the selected camera settings. \ref CAMERA_SETTINGS (Gain, brightness, hue, exposure...).
        \param setting : enum for the control mode.
        \return The current value for the corresponding control (-1 if something wrong happened).

        \note Works only if the camera is open in live mode.
         */
        int getCameraSettings(CAMERA_SETTINGS settings);

        /**
        \brief Sets the value to the corresponding \ref CAMERA_SETTINGS (Gain, brightness, hue, exposure...).
        \param settings : enum for the control mode.
        \param value : value to set for the corresponding control.
        \param use_default : will set default (or automatic) value if set to true (value (int) will not be taken into account). default : false.

        \warning setting \ref CAMERA_SETTINGS_EXPOSURE or \ref CAMERA_SETTINGS_GAIN to default will automatically sets the other to default.
        \note Works only if the camera is open in live mode.
         */
        void setCameraSettings(CAMERA_SETTINGS settings, int value, bool use_default = false);

        /**
        \brief Returns the current FPS of the camera.
        \return The current FPS (or recorded FPS for SVO). Return -1.f if something goes wrong.
         */
        float getCameraFPS();

        /**
        \brief Sets a new frame rate for the camera, or the closest available frame rate.
        \param desired_fps : the new desired frame rate.
        \deprecated This function is not thread safe and can cause instability
         * 
        \note Works only if the camera is open in live mode.
         */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/void setCameraFPS(int desired_fps)/*@cond SHOWHIDDEN*/)/*@endcond*/;

        /**
        \brief Returns the current FPS of the application/callback.

        It is based on the difference of camera timestamps between two successful grab().

        \return The current FPS of the application (if grab leads the application) or callback (if ZED is called in a thread)
         */
        float getCurrentFPS();


        /**
        \brief Returns the timestamp

        \param reference_time : \n
         * sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE: Returns the timestamp at the time the frame has been extracted from USB stream. (should be called after a grab()).\n
         * sl::TIME_REFERENCE::TIME_REFERENCE_CURRENT: Returns the current timestamp at the time the function is called. Can be compared to the camera getCameraTimestamp for synchronization.
        Use this function to compare the current timestamp and the camera timestamp, since they have the same reference (Computer start time).\n

        \return The timestamp in nanosecond. -1 if not available (SVO file without compression).
         */
        timeStamp getTimestamp(sl::TIME_REFERENCE reference_time);


        /**
        \brief Returns the timestamp at the time the frame has been extracted from USB stream. (should be called after a grab()).
        \return The timestamp of the frame grab in nanosecond. -1 if not available (SVO file without compression).

        \deprecated See \ref Camera::getTimestamp with sl::TIME_REFERENCE_IMAGE.
        \note SVO file from SDK 1.0.0 (with compression) contains the camera timestamp for each frame.
         */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED/*@endcond*/(timeStamp getCameraTimestamp()/*@cond SHOWHIDDEN*/)/*@endcond*/;

        /**
        \brief Returns the current timestamp at the time the function is called. Can be compared to the camera getCameraTimestamp for synchronization.
        Use this function to compare the current timestamp and the camera timestamp, since they have the same reference (Computer start time).
        \return The current timestamp in nanosecond.

        \deprecated See \ref Camera::getTimestamp with sl::TIME_REFERENCE_CURRENT.
         */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED/*@endcond*/(timeStamp getCurrentTimestamp()/*@cond SHOWHIDDEN*/)/*@endcond*/;


        /**
        \brief Returns the number of frame dropped since \ref grab has been called for the first time.

        Based on camera timestamp and FPS comparison.

        \return The number of frame dropped since first \ref grab call.
         */
        unsigned int getFrameDroppedCount();

        /**
        \brief Returns the current position of the SVO file.
        \return The current position in the SVO file as int (-1 if the SDK is not reading an SVO).

        \note Works only if the camera is open in SVO reading mode.
         */
        int getSVOPosition();

        /**
        \brief Sets the position of the SVO file to a desired frame.
        \param frame_number : the number of the desired frame to be decoded.

        \note Works only if the camera is open in SVO playback mode.
         */
        void setSVOPosition(int frame_number);

        /**
        \brief Returns the number of frames in the SVO file.
        \return The total number of frames in the SVO file (-1 if the SDK is not reading a SVO).

        \note Works only if the camera is open in SVO reading mode.
         */
        int getSVONumberOfFrames();

        /**
        \brief Returns the current status of the self-calibration.
        \return A status code giving information about the self calibration status.
         */
        SELF_CALIBRATION_STATE getSelfCalibrationState();

        /**
        \brief Resets the self camera calibration. This function can be called at any time AFTER the \ref open function has been called.

        It will reset and calculate again correction for misalignment, convergence and color mismatch.
        It can be called after changing camera parameters without needing to restart your executable.

        If no problem was encountered, the camera will use new parameters. Otherwise, it will be the old ones.
         */
        void resetSelfCalibration();
        ///@}

        ///@{
        /// @name Depth Sensing Functions
        // -----------------------------------------------------------------
        //                         Depth functions:
        // -----------------------------------------------------------------

        /**
        \brief Retrieves the desired measure (depth, point cloud, normals).

        The retrieve function should be called after the function \ref grab

        \param mat : [out] the \ref Mat to store the measures.
        \param measure : defines the measure you want. (see \ref MEASURE), default : MEASURE_DEPTH
        \param type : the type of the memory of provided mat that should by used. default : MEM_CPU.
        \param width : if specified, define the width of the output mat. If set to 0, the width of the ZED resolution will be taken. default : 0
        \param height : if specified, define the height of the output mat. If set to 0, the height of the ZED resolution will be taken. default : 0
        \return SUCCESS if the method succeeded, ERROR_CODE_FAILURE if an error occurred.
         */
        ERROR_CODE retrieveMeasure(Mat &mat, MEASURE measure = MEASURE_DEPTH, MEM type = MEM_CPU, int width = 0, int height = 0);

        /**
        \brief Returns the current maximum distance of depth estimation.
        \return The current maximum distance that can be computed in the defined \ref UNIT.
         */
        float getDepthMaxRangeValue();

        /**
        \brief Sets the maximum distance of depth estimation (all values after this limit will be reported as \ref TOO_FAR value).
        \param depth_max_range : maximum distance in the defined \ref UNIT.
         */
        void setDepthMaxRangeValue(float depth_max_range);

        /**
        \brief Returns the closest measurable distance by the camera, according to the camera and the depth map parameters.
        \return The minimum distance that can be computed in the defined \ref UNIT.
         */
        float getDepthMinRangeValue();

        /**
        \brief Returns the current confidence threshold value apply to the depth map.
        \return The current threshold value between 0 and 100.
         */
        int getConfidenceThreshold();

        /**
        \brief Sets a threshold for the depth map confidence.

        A lower value means more confidence and precision (but less density), an upper value reduces the filtering (more density, less certainty).

        \param conf_threshold_value : a value in [1,100].
         */
        void setConfidenceThreshold(int conf_threshold_value);
        ///@}

        ///@{
        /// @name Positional Tracking Functions
        // -----------------------------------------------------------------
        //                        Positional Tracking functions:
        // -----------------------------------------------------------------

        /**
        \brief Initializes and starts the positional tracking processes.
        \param tracking_parameters : Structure of \ref TrackingParameters, which defines specific parameters for tracking. default : a preset of \ref TrackingParameters.
        \return \ref ERROR_CODE_FAILURE if the \ref area_file_path file wasn't found, \ref SUCCESS otherwise.
         */
        ERROR_CODE enableTracking(TrackingParameters tracking_parameters = TrackingParameters());

        /**
        \brief Retrieves the estimated position and orientation of the camera in the specified reference frame.
        \param camera_pose [out] : the pose containing the position of the camera (path or position) and other information (timestamp, confidence)
        \param reference_frame : defines the reference from which you want the pose to be expressed.  default : \ref REFERENCE_FRAME_WORLD.
        \return The current state of the tracking process.

        \n Extract Rotation Matrix : camera_pose.getRotation();
        \n Extract Translation Vector: camera_pose.getTranslation();
        \n Convert to Orientation / quaternion : camera_pose.getOrientation();

        \note The camera frame is positioned at the back of the left eye of the ZED.
         */
        TRACKING_STATE getPosition(Pose &camera_pose, REFERENCE_FRAME reference_frame = REFERENCE_FRAME_WORLD);

        /**
        \brief Saves the current area learning file.

        Works only if you have set \ref TrackingParameters::enable_spatial_memory to true (default).
        The file is saved asynchronously. You can use \ref getAreaExportState to get the export state.
        The positional tracking keeps running.

        \param area_file_path : save the spatial memory database in a '.area' file.
        \return \ref ERROR_CODE_FAILURE if the \ref area_file_path file wasn't found, \ref SUCCESS otherwise.
        \note Please note that this function will also flush the area database built / loaded.
         */
        ERROR_CODE saveCurrentArea(String area_file_path);

        /**
        \brief Returns the state of the spatial memory export process.
        \return The current state of the spatial memory export process
         */
        AREA_EXPORT_STATE getAreaExportState();

        /**
        \brief Resets the tracking, re-initializes the path with the transformation matrix given.
        \param path : Position of the camera in the world frame when the camera is started. By default, it is set to identity.
        \return \ref ERROR_CODE_FAILURE if the \ref area_file_path file wasn't found, \ref SUCCESS otherwise.

        \note Please note that this function will also flush the area database built / loaded.
         */
        ERROR_CODE resetTracking(const Transform &path);

        /**
        \brief Disables positional tracking.

        The positional tracking is directly stopped. If a file path is given, the saving part will be asynchronous. See \ref getAreaExportState to get the exportation state.

        \param area_file_path : if set, save the spatial memory database in a '.area' file. default : (empty)
        areaFilePath is the name and path of the database, e.g. : "path/to/file/myArea1.area".
        \note The '.area' database depends on the depth map SENSING_MODE chosen during the recording. The same mode must be used to reload the database.
         */
        void disableTracking(String area_file_path = "");


        // -----------------------------------------------------------------
        //                        Positional Tracking functions, for ZED-M only (using IMU)
        // -----------------------------------------------------------------

        /**
        \brief Retrieves the IMU Data specified by IMUData structure at a specific time reference
        \param imu_data [out] : the IMUData that inherits from sl::Pose, containing the orientation of the IMU (pose in world reference frame) and other information (timestamp, raw imu data)
        \param reference_time : defines the time reference from when you want the pose to be extracted.
        \return SUCCESS if IMUData has been filled

        \n Extract Rotation Matrix : imu_data.getRotation();
        \n Extract Orientation / Quaternion : imu_data.getOrientation();

        \note : Translation is not provided when using the IMU only.
        \note : The quaternion (fused data) is given in the specified COORDINATE_SYSTEM of InitParameters. The camera_imu_transform given in getCameraInformation() is already applied, therefore the 
                quaternions values are not expressed in the IMU frame but the left camera frame.

        \warning In SVO reading mode, the TIME_REFERENCE_CURRENT is currently not available (yielding ERROR_CODE_INVALID_FUNCTION_PARAMETERS),
         * only the quaternion data at TIME_REFERENCE_IMAGE is available, other values will be set to 0.
         */
        ERROR_CODE getIMUData(IMUData &imu_data, TIME_REFERENCE reference_time);

        /**
        \brief Set an optionnal prior to IMU orientation.
        \note This function is only effective if a ZED Mini (ZED-M) is used.
        \param sl::Transform to be ingested in IMU fusion. Note that only the rotation is used.
        \return SUCCESS if the transform has been passed, sl::ERROR_CODE_INVALID_FUNCTION_CALL otherwise (ZED Camera used for example).
         */
        ERROR_CODE setIMUPrior(const sl::Transform& transform);

        //@}

        ///@{
        /// @name Spatial Mapping Functions
        // -----------------------------------------------------------------
        //                         Spatial Mapping functions:
        // -----------------------------------------------------------------

        /**
        \brief Initializes and starts the spatial mapping processes.

        The spatial mapping will create a geometric representation of the scene based on both tracking data and 3D point clouds.
        \n The resulting output is a \ref Mesh and can be obtained by the \ref extractWholeMesh function or with \ref retrieveMeshAsync after calling \ref requestMeshAsync.

        \param spatial_mapping_parameters : the structure containing all the specific parameters for the spatial mapping.
        \n default : a balanced parameters preset between geometric fidelity and output file size. For more information, checkout the \ref SpatialMappingParameters documentation.
        \return \ref SUCCESS if everything went fine, \ref ERROR_CODE_FAILURE otherwise

        \warning The tracking needs to be enabled to create a map.
        \warning The performance greatly depends on the input parameters.
        If the mapping framerate is too slow in live mode, consider using an SVO file, or choose a coarser mesh resolution.

        \note This features is using host memory (RAM) to store the 3D map, the maximum amount of available memory allowed can be tweaked using the SpatialMappingParameters.
         */
        ERROR_CODE enableSpatialMapping(SpatialMappingParameters spatial_mapping_parameters = SpatialMappingParameters());

        /**
        \brief Returns the current spatial mapping state.
        \return The current state of the spatial mapping process
         */
        SPATIAL_MAPPING_STATE getSpatialMappingState();

        // -----------------------------------------------------------------
        // Async functions of mesh generation ( *Async())
        // -----------------------------------------------------------------
        /**
        \brief Starts the mesh generation process in a non blocking thread from the spatial mapping process.

        \note Only one mesh generation can be done at a time, consequently while the previous launch is not done every call will be ignored.
         */
        void requestMeshAsync();

        /**
        \brief Returns the mesh generation status, useful after calling \ref requestMeshAsync to know if you can call \ref retrieveMeshAsync.
        \return \ref SUCCESS if the mesh is ready and not yet retrieved, otherwise \ref ERROR_CODE_FAILURE.
         */
        ERROR_CODE getMeshRequestStatusAsync();

        /**
        \brief Retrieves the current generated mesh.

        This function only updates chunks that need to be updated and add the new ones in order to improve update speed.
        \note You should not modify the mesh between two calls of this function, otherwise it can lead to corrupted mesh.

        \param mesh [out] : The mesh to be filled.
        \return \ref SUCCESS if the mesh is retrieved, otherwise \ref ERROR_CODE_FAILURE.
         */
        ERROR_CODE retrieveMeshAsync(Mesh &mesh);

        // -----------------------------------------------------------------
        // Blocking (synchronous) function of mesh generation
        // -----------------------------------------------------------------
        /**
        \brief Extracts the current mesh from the spatial mapping process.
        \param mesh [out] : The mesh to be filled.
        \return \ref SUCCESS if the mesh is filled and available, otherwise \ref ERROR_CODE_FAILURE.

        \note This function will return when the mesh has been created or updated. This is, therefore, a blocking function. You should either call it in a thread or at the end of the mapping process.
        Calling this function in the grab loop will block the depth and tracking computation and therefore gives bad results.
         */
        ERROR_CODE extractWholeMesh(Mesh &mesh);

        /**
        \brief Switches the pause status of the data integration mechanism for the spatial mapping.
        \param status : if true, the integration is paused. If false, the spatial mapping is resumed.
         */
        void pauseSpatialMapping(bool status);

        /**
        \brief Disables the Spatial Mapping process.
        All the spatial mapping functions are disables, mesh cannot be retrieves after this call.
         */
        void disableSpatialMapping();
        ///@}

        /**
        \brief Detect the plane at the corresponding image coordinate
         * 
        \param coord [in] : The image coordinate. The coordinate must be taken from the full size image
        \param plane [out] : The detected plane if the function succeeded
         \return \ref SUCCESS if a plane is found otherwise \ref ERROR_CODE_PLANE_NOT_FOUND
         \note The reference frame is defined by the sl:RuntimeParameters (measure3D_reference_frame)
         * given to the grab() function.
         */
         ERROR_CODE findPlaneAtHit(sl::uint2 coord, sl::Plane &plane);
        
        /**
        \brief Detect the floor plane of the scene
         * 
        \param floorPlane [out] : The detected floor plane if the function succeeded
        \param resetTrackingFloorFrame [out] : The transform to align the tracking with the floor plane. The initial position will then 
         * be at ground height, with the axis align with the gravity. The positional tracking needs to be reset/enable 
         * with this transform as parameter (TrackingParameters.initial_world_transform)
        \param floor_height_prior [in] : Prior set to locate the floor plane depending on the known camera 
         * distance to the ground, expressed in the same unit as the ZED. If the prior is too far from the detected floor plane, 
         * the function will return ERROR_CODE_PLANE_NOT_FOUND
        \param world_orientation_prior [in] : Prior set to locate the floor plane depending on the known camera 
         * orientation to the ground. If the prior is too far from the detected floor plane, 
         * the function will return ERROR_CODE_PLANE_NOT_FOUND
        \param floor_height_prior_tolerance [in] : Prior height tolerance, absolute value.
         \return \ref SUCCESS if the floor plane is found and match the priors (optional, only if defined), 
         * otherwise \ref ERROR_CODE_PLANE_NOT_FOUND
         \note The reference frame is defined by the sl:RuntimeParameters (measure3D_reference_frame)
         * given to the grab() function. The length unit is defined by sl:InitParameters (coordinate_units)
         * With the ZED the assumption is made that the floor plane is the dominant plane in the scene, The ZED Mini uses the gravity as prior
         */
        ERROR_CODE findFloorPlane(sl::Plane &floorPlane, sl::Transform &resetTrackingFloorFrame,
                float floor_height_prior = INVALID_VALUE, sl::Rotation world_orientation_prior = sl::Matrix3f::zeros(),
                float floor_height_prior_tolerance = INVALID_VALUE);

        ///@{
        /// @name Recording
        // -----------------------------------------------------------------
        //                 		Recording functions
        // -----------------------------------------------------------------      

        /**
        \brief Creates a file to store video frames.
        \param video_filename : can be a *.svo file or a *.avi file (detected by the suffix name provided).
        \param compression_mode : can be one of the \ref SVO_COMPRESSION_MODE enum. default : SVO_COMPRESSION_MODE_LOSSLESS.
        \return an \ref ERROR_CODE that defines if file was successfully created and can be filled with images.

        \warning This function can be called multiple times during ZED lifetime, but if video_filename is already existing, the file will be erased.
         \note SVO_COMPRESSION_MODE::SVO_COMPRESSION_MODE_RAW is deprecated in recording mode
         */
        ERROR_CODE enableRecording(String video_filename, SVO_COMPRESSION_MODE compression_mode = SVO_COMPRESSION_MODE_LOSSLESS);

        /**
        \brief Records the current frame provided by \ref grab into the file.
        \return The recording state structure, for more details see \ref RecordingState.

		\param rfu : Not used for the moment. Reserved for future use.
        \warning \ref grab must be called before \ref record to take the last frame available. Otherwise, it will be the last grabbed frame.
         */
        RecordingState record();

        /**
        \brief Disables the recording and closes the generated file.
         */
        void disableRecording();
        ///@}

        // -----------------------------------------------------------------
        //                         (static)
        // -----------------------------------------------------------------
        /**
        \brief Returns the version of the currently installed ZED SDK.
        \return The ZED SDK version as a string with the following format : MAJOR.MINOR.PATCH
         */
        static String getSDKVersion();

        /**
        \brief Checks if ZED cameras are connected, can be called before instantiating a Camera object.
        \return The number of connected ZED.

        \deprecated See \ref getDeviceList.
         */
        /*@cond SHOWHIDDEN*/SL_DEPRECATED(/*@endcond*/static int isZEDconnected()/*@cond SHOWHIDDEN*/)/*@endcond*/;

        /**
        \brief List all the connected device with their associated information
        \return The device properties for each connected camera

         */
        static std::vector<sl::DeviceProperties> getDeviceList();

        /**
        \brief ONLY FOR NVIDIA JETSON : Sticks the calling thread to a specific CPU core.
        \param cpuCore : int that defines the core the thread must be run on. could be between 0 and 3. (cpu0,cpu1,cpu2,cpu3).
        \return \ref SUCCESS if stick is OK, otherwise status error.

        \warning Function only available for Nvidia Jetson. On other platforms, the result will always be 0 and no operations are performed.
         */
        static ERROR_CODE sticktoCPUCore(int cpu_core);

        // Suppress default copy constructor
        Camera(const Camera&) = delete;
    private:
        CameraMemberHandler *h = 0;
        bool opened = false;
    };

    /**
    \ingroup Depth_group
    \brief Writes the current depth map into a file.
    \param zed : the current camera object.
    \param format : the depth file format you desired.
    \param name : the name (path) in which the depth will be saved.
    \param factor : only for PNG and PGM, apply a gain to the depth value. default : 1.
    The PNG format only stores integers between 0 and 65536, if you're saving a depth map in meters, values will be rounded to the nearest integer. Set factor to 0.01 to reduce this effect by converting to millimeters.
    Do not forget to scale (by 1./factor) the pixel value at reading to get the real depth.
    The occlusions are represented by 0.
    \return False if something wrong happens, else return true.
     */
    /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ bool saveDepthAs(Camera &zed, DEPTH_FORMAT format, String name, float factor = 1.);

    /**
    \ingroup Depth_group
    \brief Writes the current depth map into a file.
    \param depth : the depth map to record (CPU 32F_C1 \ref Mat)
    \param format : the depth file format you desired.
    \param name : the name (path) in which the depth will be saved.
    \param factor : only for PNG and PGM, apply a gain to the depth value. default : 1.
    The PNG format only stores integers between 0 and 65536, if you're saving a depth map in meters, values will be rounded to the nearest integer. Set factor to 0.01 to reduce this effect by converting to millimeters.
    Do not forget to scale (by 1./factor) the pixel value at reading to get the real depth.
    The occlusions are represented by 0.
    \return False if something wrong happens, else return true.
     */
    /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ bool saveDepthAs(Mat &depth, DEPTH_FORMAT format, String name, float factor = 1.);

    /**
    \ingroup Depth_group
    \brief Writes the current point cloud into a file
    \param zed : the current camera object.
    \param format : the point cloud file format you desired.
    \param name : the name (path) in which the point cloud will be saved.
    \param with_color : indicates if the color must be saved. default : false.
    \return False if something wrong happens, else return true.

    \note The color is not saved for XYZ and VTK files.
     */
    /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ bool savePointCloudAs(Camera &zed, POINT_CLOUD_FORMAT format, String name, bool with_color = false);

    /**
    \ingroup Depth_group
    \brief Writes the current point cloud into a file
    \param cloud : the point cloud to record (CPU 32F_C4 \ref Mat)
    \param format : the point cloud file format you desired.
    \param name : the name (path) in which the point cloud will be saved.
    \param with_color : indicates if the color must be saved. default : false.
    \return False if something wrong happens, else return true.

    \note The color is not saved for XYZ and VTK files.
     */
    /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ bool savePointCloudAs(Mat &cloud, POINT_CLOUD_FORMAT format, String name, bool with_color = false);

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SpatialMappingParameters::MAPPING_RESOLUTION &resolution);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const SpatialMappingParameters::MAPPING_RESOLUTION &resolution) {
        return os << toString(resolution);
    }
    ///@endcond

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SDK_EXPORT/*@endcond*/ toString(const SpatialMappingParameters::MAPPING_RANGE &range);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const SpatialMappingParameters::MAPPING_RANGE &range) {
        return os << toString(range);
    }
    ///@endcond
}

#endif /* __CAMERA_HPP__ */
