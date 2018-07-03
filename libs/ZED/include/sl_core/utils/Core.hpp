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

#ifndef __CORE_HPP__
#define __CORE_HPP__

#include <sl_core/utils/types.hpp>

namespace sl {

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
        size_t area() {
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
    \enum MEM
    \ingroup Core_group
    \brief List available memory type
     */
    enum MEM {
        MEM_CPU = 1, /**< CPU Memory (Processor side).*/
        MEM_GPU = 2 /**< GPU Memory (Graphic card side).*/
    };

    /**
    \ingroup Core_group
    \brief Convert the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const MEM &mem);
    ///@cond

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
    enum COPY_TYPE {
        COPY_TYPE_CPU_CPU, /**< copy data from CPU to CPU.*/
        COPY_TYPE_CPU_GPU, /**< copy data from CPU to GPU.*/
        COPY_TYPE_GPU_GPU, /**< copy data from GPU to GPU.*/
        COPY_TYPE_GPU_CPU /**< copy data from GPU to CPU.*/
    };

    /**
    \ingroup Core_group
    \brief Convert the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const COPY_TYPE &cpy);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const COPY_TYPE &cpy) {
        return os << toString(cpy);
    }
    ///@endcond

    /**
    \enum MAT_TYPE
    \ingroup Core_group
    \brief List available Mat formats.
     */
    enum MAT_TYPE {
        MAT_TYPE_32F_C1, /**< float 1 channel.*/
        MAT_TYPE_32F_C2, /**< float 2 channels.*/
        MAT_TYPE_32F_C3, /**< float 3 channels.*/
        MAT_TYPE_32F_C4, /**< float 4 channels.*/
        MAT_TYPE_8U_C1, /**< unsigned char 1 channel.*/
        MAT_TYPE_8U_C2, /**< unsigned char 2 channels.*/
        MAT_TYPE_8U_C3, /**< unsigned char 3 channels.*/
        MAT_TYPE_8U_C4 /**< unsigned char 4 channels.*/
    };

    /**
    \ingroup Core_group
    \brief Convert the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const MAT_TYPE &type);
    ///@cond

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
        MEM mem_type = MEM_CPU;

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
        // Variable used in verbose mode to indicate witch Mat is printing informations.
        String name;

        // Whether the MAT can display informations or not.
        bool verbose = false;

        // Data timestamp
        timeStamp timestamp = 0;

        /**
         * \brief empty Mat default constructor.
         */
        Mat();

        /**
        \brief Mat constructor.

        This function directly allocates the requested memory. It calls \ref alloc.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
        \param memory_type : defines where the buffer will be stored. (\ref MEM_CPU and/or \ref MEM_GPU).
         */
        Mat(size_t width, size_t height, MAT_TYPE mat_type, MEM memory_type = MEM_CPU);

        /**
        \brief Mat constructor from an existing data pointer.

        This function doesn't allocate the memory.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
        \param ptr : pointer to the data array. (CPU or GPU).
        \param step : step of the data array. (the Bytes size of one pixel row).
        \param memory_type : defines where the buffer will be stored. (\ref MEM_CPU and/or \ref MEM_GPU).
         */
        Mat(size_t width, size_t height, MAT_TYPE mat_type, sl::uchar1 *ptr, size_t step, MEM memory_type = MEM_CPU);

        /**
        \brief Mat constructor from two existing data pointers, CPU and GPU.

        This function doesn't allocate the memory.

        \param width : width of the matrix in pixels.
        \param height : height of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
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
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
        \param memory_type : defines where the buffer will be stored (\ref MEM_CPU and/or \ref MEM_GPU).
         */
        Mat(Resolution resolution, MAT_TYPE mat_type, MEM memory_type = MEM_CPU);

        /**
        \brief Mat constructor from an existing data pointer.

        This function doesn't allocate the memory.

        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
        \param ptr : pointer to the data array. (CPU or GPU).
        \param step : step of the data array (the Bytes size of one pixel row).
        \param memory_type : defines where the buffer will be stored. (\ref MEM_CPU and/or \ref MEM_GPU).
         */
        Mat(Resolution resolution, MAT_TYPE mat_type, sl::uchar1 *ptr, size_t step, MEM memory_type = MEM_CPU);

        /**
        \brief Mat constructor from two existing data pointers, CPU and GPU.

        This function doesn't allocate the memory.

        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
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
        \param mat_type : the type of the matrix (\ref MAT_TYPE_32F_C1, \ref MAT_TYPE_8U_C4...).
        \param memory_type : defines where the buffer will be stored. (\ref MEM_CPU and/or \ref MEM_GPU).

        \warning It erases previously allocated memory.
         */
        void alloc(size_t width, size_t height, MAT_TYPE mat_type, MEM memory_type = MEM_CPU);

        /**
        \brief Allocates the Mat memory.
        \param resolution : the size of the matrix in pixels.
        \param mat_type : the type of the matrix (sl::MAT_TYPE_32F_C1,sl::MAT_TYPE_8U_C4...).
        \param memory_type : defines where the buffer will be stored. (sl::MEM_CPU and/or sl::MEM_GPU).

        \warning It erases previously allocated memory.
         */
        void alloc(Resolution resolution, MAT_TYPE mat_type, MEM memory_type = MEM_CPU);

        /**
        \brief Mat destructor. This function calls \ref free to release owned memory.
         */
        ~Mat();

        /**
        \brief Free the owned memory.
        \param memory_type : specify whether you want to free the \ref MEM_CPU and/or \ref MEM_GPU memory.
         */
        void free(MEM memory_type = MEM_CPU | MEM_GPU);

        /**
        \brief Performs a shallow copy.

        This function doesn't copy the data array, it only copies the pointer.

        \param that : the \ref Mat to be copied.
        \return The new \ref Mat object which point to the same data as that.
         */
        Mat &operator=(const Mat &that);

        /**
        \brief Downloads data from DEVICE (GPU) to HOST (CPU), if possible.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note If no CPU or GPU memory are available for this Mat, some are directly allocated.
        \note If verbose sets, you have informations in case of failure.
         */
        ERROR_CODE updateCPUfromGPU();

        /**
        \brief Uploads data from HOST (CPU) to DEVICE (GPU), if possible.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note If no CPU or GPU memory are available for this Mat, some are directly allocated.
        \note If verbose sets, you have informations in case of failure.
         */
        ERROR_CODE updateGPUfromCPU();

        /**
        \brief Copies data an other Mat (deep copy).
        \param dst : the Mat where the data will be copied.
        \param cpyType : specify the memories that will be used for the copy.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note If the destination is not allocated or has a not a compatible \ref MAT_TYPE or \ref Resolution,
        current memory is freed and new memory is directly allocated.
         */
        ERROR_CODE copyTo(Mat &dst, COPY_TYPE cpyType = COPY_TYPE_CPU_CPU) const;

        /**
        \brief Copies data from an other Mat (deep copy).
        \param src : the Mat where the data will be copied from.
        \param cpyType : specify the memories that will be used for the update.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note If the current Mat is not allocated or has a not a compatible \ref MAT_TYPE or \ref Resolution with the source,
        current memory is freed and new memory is directly allocated.
         */
        ERROR_CODE setFrom(const Mat &src, COPY_TYPE cpyType = COPY_TYPE_CPU_CPU);

        /**
        \brief Reads an image from a file (only if \ref MEM_CPU is available on the current \ref Mat).

        Supported input files format are PNG and JPEG.

        \param filePath : file path including the name and extension.
        \return \ref UCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note Supported \ref MAT_TYPE are : \ref MAT_TYPE_8U_C1, \ref MAT_TYPE_8U_C3 and \ref MAT_TYPE_8U_C4.
         */
        ERROR_CODE read(const String &filePath);

        /**
        \brief Writes the \ref Mat (only if \ref MEM_CPU is available) into a file as an image.

        Supported output files format are PNG and JPEG.

        \param filePath : file path including the name and extension.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note Supported \ref MAT_TYPE are : \ref MAT_TYPE_8U_C1, \ref MAT_TYPE_8U_C3 and \ref MAT_TYPE_8U_C4.
         */
        ERROR_CODE write(const String &filePath);

        /**
        \brief Fills the Mat with the given value.

        This function overwrite all the matrix.

        \param value : the value to be copied all over the matrix.
        \param memory_type : defines which buffer to fill, CPU and/or GPU.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.
         */
        template <typename T>
        ERROR_CODE setTo(T value, MEM memory_type = MEM_CPU);

        /**
        \brief Sets a value to a specific point in the matrix.
        \param x : specify the column.
        \param y : specify the row.
        \param value : the value to be set.
        \param memory_type : defines which memory will be updated.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.

        \warning Not efficient for \ref MEM_GPU, use it on sparse data.
         */
        template <typename N>
        ERROR_CODE setValue(size_t x, size_t y, N value, MEM memory_type = MEM_CPU);

        /**
        \brief Returns the value of a specific point in the matrix.
        \param x : specify the column
        \param y : specify the row
        \param memory_type : defines which memory should be read.
        \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.

        \note This function is templated for \ref uchar1, \ref uchar2, \ref uchar3, \ref uchar4, \ref float1, \ref float2, \ref float3, \ref float4.

        \warning Not efficient for \ref MEM_GPU, use it on sparse data.
         */
        template <typename N>
        ERROR_CODE getValue(size_t x, size_t y, N *value, MEM memory_type = MEM_CPU) const;

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
        \brief Returns the height of the matrix.
        \return The height of the matrix in pixels.
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
        \param memory_type : specify whether you want \ref MEM_CPU or \ref MEM_GPU step.
        \return The pointer of the Mat data.
         */
        template <typename N>
        N *getPtr(MEM memory_type = MEM_CPU) const;

        /**
        \brief Returns the memory step in Bytes (the Bytes size of one pixel row).
        \param memory_type : specify whether you want \ref MEM_CPU or \ref MEM_GPU step.
        \return The step in bytes of the specified memory.
         */
        size_t getStepBytes(MEM memory_type = MEM_CPU) const;

        /**
        \brief Returns the memory step in number of elements (the number of values in one pixel row).
        \param memory_type : specify whether you want \ref MEM_CPU or \ref MEM_GPU step.
        \return The step in number of elements.
         */
        template <typename N>
        inline size_t getStep(MEM memory_type = MEM_CPU) const {
            return getStepBytes(memory_type) / sizeof(N);
        }

        /**
        \brief Returns the memory step in number of elements (the number of values in one pixel row).
        \param memory_type : specify whether you want \ref MEM_CPU or \ref MEM_GPU step.
        \return The step in number of elements.
         */
        inline size_t getStep(MEM memory_type = MEM_CPU)const {
            switch(data_type) {
            case MAT_TYPE_32F_C1:
                return getStep<sl::float1>(memory_type);
            case MAT_TYPE_32F_C2:
                return getStep<sl::float2>(memory_type);
            case MAT_TYPE_32F_C3:
                return getStep<sl::float3>(memory_type);
            case MAT_TYPE_32F_C4:
                return getStep<sl::float4>(memory_type);
            case MAT_TYPE_8U_C1:
                return getStep<sl::uchar1>(memory_type);
            case MAT_TYPE_8U_C2:
                return getStep<sl::uchar2>(memory_type);
            case MAT_TYPE_8U_C3:
                return getStep<sl::uchar3>(memory_type);
            case MAT_TYPE_8U_C4:
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
        \return The Euler angles, as a \ref float3 (x=roll, y=pitch, z=yaw)
         */
        float3 getEulerAngles(bool radian = true) const;

        /**
        \brief Sets the Rotation from the Euler angles.
        \param euler_angles : The Euler angles, as a \ref float3 (x=roll, y=pitch, z=yaw)
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
        inline void setRotation(const Rotation &rotation) {
            setRotationMatrix(rotation);
        }

        /**
        \brief Returns the current orientation as a Rotation.
        \return The rotation computed from the orientation data.

        \deprecated See \ref getRotationMatrix
         */
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
        inline void setRotation(const Rotation &rotation) {
            setRotationMatrix(rotation);
        }

        /**
        \brief Returns the Rotation of the current Transform.
        \return The Rotation created from the Transform values.
        \warning The given Rotation contains a copy of the Transform values. Not references.

        \deprecated See \ref getRotationMatrix
         */
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
        \return The Euler angles, as a \ref float3 (x=roll, y=pitch, z=yaw)
         */
        float3 getEulerAngles(bool radian = true) const;

        /**
        \brief Sets the Rotation of the Transform from the Euler angles.
        \param euler_angles : The Euler angles, as a \ref float3 (x=roll, y=pitch, z=yaw)
        \param radian : Define if the angle in is radian or degree
         */
        void setEulerAngles(const float3 &euler_angles, bool radian = true);
    };

    /**
    \struct CameraParameters
    \ingroup Depth_group
    \brief Intrinsic parameters of a camera.

    \note Similar to the CalibrationParameters, those parameters are taken from the settings file (SNXXX.conf) and are modified during the sl::Camera::open call (with or without Self-Calibration).
    Those parameters given after sl::Camera::open call, represent the "new camera matrix" that fits/defines each image taken after rectification ( through retrieveImage).
    \note fx,fy,cx,cy must be the same for Left and Right Camera once sl::Camera::open has been called. Since distortion is corrected during rectification, distortion should not be considered after sl::Camera::open call.
     */
    struct CameraParameters {
        float fx; /**< Focal length in pixels along x axis. */
        float fy; /**< Focal length in pixels along y axis. */
        float cx; /**< Optical center along x axis, defined in pixels (usually close to width/2). */
        float cy; /**< Optical center along y axis, defined in pixels (usually close to height/2). */
        double disto[5]; /**< Distortion factor : [ k1, k2, p1, p2, k3 ]. Radial (k1,k2,k3) and Tangential (p1,p2) distortion.*/
        float v_fov; /**< Vertical field of view after stereo rectification, in degrees. */
        float h_fov; /**< Horizontal field of view after stereo rectification, in degrees.*/
        float d_fov; /**< Diagonal field of view after stereo rectification, in degrees.*/
        Resolution image_size; /** size in pixels of the images given by the camera.*/

        /**
        \brief Setups the parameter of a camera.
        \param focal_x : horizontal focal length.
        \param focal_y : vertical focal length.
        \param focal_x : horizontal optical center.
        \param focal_x : vertical optical center.
         */
        void SetUp(float focal_x, float focal_y, float center_x, float center_y) {
            fx = focal_x;
            fy = focal_y;
            cx = center_x;
            cy = center_y;
        }
    };

    /**
    \struct CalibrationParameters
    \ingroup Depth_group
    \brief Intrinsic parameters of each cameras and extrinsic (translation and rotation).
    \note The calibration/rectification process, called during sl::Camera::open, is using the raw parameters defined in the SNXXX.conf file, where XXX is the ZED Serial Number.
    \n Those values may be adjusted or not by the Self-Calibration to get a proper image alignment. After sl::Camera::open is done (with or without Self-Calibration activated) success, most of the stereo parameters (except Baseline of course) should be 0 or very close to 0.
    \n It means that images after rectification process (given by retrieveImage()) are aligned as if they were taken by a "perfect" stereo camera, defined by the new CalibrationParameters.
     */
    struct CalibrationParameters {
        float3 R; /**< Rotation (using Rodrigues' transformation) between the two sensors. Defined as 'tilt', 'convergence' and 'roll'.*/
        float3 T; /**< Translation between the two sensors. T.x is the distance between the two cameras (baseline) in the sl::UNIT chosen during sl::Camera::open (mm, cm, meters, inches...).*/
        CameraParameters left_cam; /**< Intrinsic parameters of the left camera  */
        CameraParameters right_cam; /**< Intrinsic parameters of the right camera  */
    };

    /**
    \struct CameraInformation
    \ingroup Video_group
    \brief Camera specific parameters
     */
    struct CameraInformation {
        CalibrationParameters calibration_parameters; /**< Intrinsic and Extrinsic stereo parameters for rectified images (default).  */
        CalibrationParameters calibration_parameters_raw; /**< Intrinsic and Extrinsic stereo parameters for original images (unrectified).  */
        sl::Transform camera_imu_transform; /**< Left Camera to IMU transform matrix, that contains rotation and translation between camera frame and IMU frame. Note that this transform is already applied in the fused quaternion provided in getIMUData(). Therefore, this transform will only be filled in LIVE mode with ZED-M */
        unsigned int serial_number = 0; /**< camera dependent serial number.  */
        unsigned int firmware_version = 0; /**< current firmware version of the camera. */
        sl::MODEL camera_model = sl::MODEL_LAST; /**< camera model (ZED or ZED-M) */
    };
    ///@}

    /**
    \class Pose
    \ingroup PositionalTracking_group
    \brief Contains positional tracking data which gives the position and orientation of the ZED in 3D space.

    Different representations of position and orientation can be retrieved, along with timestamp and pose confidence.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ Pose {
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
        \return The Euler angles, as a \ref float3 (x=roll, y=pitch, z=yaw)
         */
        float3 getEulerAngles(bool radian = true);

        /**
        4x4 Matrix which contains the rotation (3x3) and the translation. Orientation is extracted from this transform as well.
         */
        Transform pose_data;

        /**
        Timestamp of the pose. This timestamp should be compared with the camera timestamp for synchronization.
         */
        sl::timeStamp timestamp;

        /**
        Confidence/Quality of the pose estimation for the target frame.
        \n A confidence metric of the tracking [0-100], 0 means that the tracking is lost, 100 means that the tracking can be fully trusted.
         */
        int pose_confidence;

        /**
        boolean that indicates if tracking is activated or not. You should check that first if something wrong.
         */
        bool valid;
    };

    /**
    \class IMUData
    \ingroup PositionalTracking_group
    \brief Contains inertial positional tracking data which gives the orientation of the ZED-M.
    \note This data will not be filled if you are using a ZED camera

    Different representations of orientation can be retrieved, along with timestamp and pose confidence.
    Raw data (linear acceleration and angular velocity) are also given along with the calculated orientation.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ IMUData : Pose {
        friend class CameraMemberHandler;
        friend class Camera;
        //ZED_SDK_VERSION_ATTRIBUTE

    public:

        /**
        \brief Default constructor which creates an empty IMUData (identity).
         */
        IMUData();

        /**
        \brief IMUData constructor with deep copy.
         */
        IMUData(const IMUData &pose);

        /**
        \brief IMUData constructor with deep copy.
         */
        IMUData(const Transform &pose_data, unsigned long long timestamp = 0, int confidence = 0);

        /**
        \brief IMUData destructor.
         */
        ~IMUData();

        /**
        (3x3) 3x3 Covariance matrix for orientation (x,y,z axes)
         */
        sl::Matrix3f orientation_covariance;

        /**
        (3x1) Vector for angular velocity of the IMU, given in deg/s.
        In other words, the current velocity at which the sensor is rotating around the x, y, and z axes.
         */
        sl::float3 angular_velocity;

        /**
        (3x1) Vector for linear acceleration of the IMU, given in m/s^2.
        In other words, the current acceleration of the sensor, along with the x, y, and z axes.
         */
        sl::float3 linear_acceleration;

        /**
        (3x3) 3x3 Covariance matrix for the angular velocity
         */
        sl::Matrix3f angular_velocity_convariance;

        /**
        (3x3) 3x3 Covariance matrix for the linear acceleration
         */
        sl::Matrix3f linear_acceleration_convariance;

		/**
		Reserved to future use.
		*/
		int imu_image_sync_val;

    };


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
    \param floatMat : (in/out) matrix to transform, can be either a \ref MAT_TYPE_32F_C4 (the fourth value will be ignored as it contained the color information) or a \ref MAT_TYPE_32F_C3.
    \param coord_system_src : the current coordinate system of floatMat.
    \param coord_system_dst : the destination coordinate system for floatMat.
    \param mem : define which memory should be transformed from floatMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertCoordinateSystem(Mat &floatMat, COORDINATE_SYSTEM coord_system_src, COORDINATE_SYSTEM coord_system_dst, MEM mem = MEM_CPU);

    /*!
    \brief Change the coordinate system of a transform matrix.
    \param motionMat : (in/out) matrix to transform
    \param coord_system_src : the current coordinate system of motionMat.
    \param coord_system_dst : the destination coordinate system for motionMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.
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
    \param floatMat : (in/out) matrix to transform, can be either a \ref MAT_TYPE_32F_C4 (the fourth value will be ignored as it contained the color information), \ref MAT_TYPE_32F_C3 or a \ref MAT_TYPE_32F_C1.
    \param unit_src : the current unit of floatMat.
    \param unit_dst : the destination unit for floatMat.
    \param mem : define which memory should be transformed from floatMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertUnit(Mat &floatMat, UNIT unit_src, UNIT unit_dst, MEM mem = MEM_CPU);

    /*!
    \brief Change the unit (of the translations) of a transform matrix.
    \param floatMat : (in/out) matrix to transform
    \param unit_src : the current unit of motionMat.
    \param unit_dst : the destination unit for motionMat.
    \return \ref SUCCESS if everything went well, \ref ERROR_CODE_FAILURE otherwise.
     */
    /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ERROR_CODE convertUnit(Transform &motionMat, UNIT unit_src, UNIT unit_dst);

}
#endif /* __CORE_HPP__ */
