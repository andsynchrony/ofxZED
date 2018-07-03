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

#ifndef __MESH_HPP__
#define __MESH_HPP__

#include <vector>

#include <sl_core/utils/Core.hpp>

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
    enum MESH_FILE_FORMAT {
        MESH_FILE_PLY, /**< Contains only vertices and faces.*/
        MESH_FILE_PLY_BIN, /**< Contains only vertices and faces, encoded in binary.*/
        MESH_FILE_OBJ, /**< Contains vertices, normals, faces and textures informations if possible.*/
        MESH_FILE_LAST
    };

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MESH_FILE_FORMAT &mesh_frmt);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const MESH_FILE_FORMAT &mesh_frmt) {
        return os << toString(mesh_frmt);
    }
    ///@endcond

    /**
    \enum MESH_TEXTURE_FORMAT
    \ingroup SpatialMapping_group
    \brief Lists available mesh texture formats.
     */
    enum MESH_TEXTURE_FORMAT {
        MESH_TEXTURE_RGB, /**< The texture has 3 channels.*/
        MESH_TEXTURE_RGBA, /**< The texture has 4 channels.*/
        MESH_TEXTURE_LAST
    };

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MESH_TEXTURE_FORMAT &text_frmt);
    ///@cond

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
        enum MESH_FILTER {
            MESH_FILTER_LOW, /**< Clean the mesh by closing small holes and removing isolated faces.*/
            MESH_FILTER_MEDIUM, /**< Soft decimation and smoothing.*/
            MESH_FILTER_HIGH, /**< Decimate the number of triangles and apply a soft smooth.*/
            MESH_FILTER_LAST
        };

        /**
        \brief Default constructor, set all parameters to their default and optimized values.
         */
        MeshFilterParameters(MESH_FILTER mesh_filtering = MESH_FILTER_LOW) {
            set(mesh_filtering);
        }

        /**
        \brief Sets the filtering intensity
        \param filtering_ : the desired \ref FILTER.
         */
        void set(MESH_FILTER mesh_filtering = MESH_FILTER_LOW) {
            filtering = mesh_filtering;
        }

        MESH_FILTER filtering = MESH_FILTER_LOW;

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

    /**
    \ingroup SpatialMapping_group
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const MeshFilterParameters::MESH_FILTER &mesh_filter);
    ///@cond

    inline std::ostream &operator<<(std::ostream &os, const MeshFilterParameters::MESH_FILTER &mesh_filter) {
        return os << toString(mesh_filter);
    }
    ///@endcond

    /**
    \class Texture
    \ingroup SpatialMapping_group
    \brief Contains information about texture image associated to a \ref Mesh.
     */
    class /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ Texture {
    public:
        /**
        \brief Default constructor which creates an empty \ref Texture.
         */
        Texture();

        /**
        \brief \ref Texture destructor.
         */
        ~Texture();

        /** \ref Mat that contains the data of the texture.*/
        Mat data;

        /** Useful for OpenGL binding reference (value not set by the SDK).*/
        unsigned int indice_gl;

        /**  The name of the file in which the texture is saved.*/
        String name;

        /**
        \brief Clears data.
         */
        void clear();
    };

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
        friend class CameraMemberHandler;
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
        \n In order to display a textured mesh you need to bind the Texture and then draw each triangles by picking its uv values.

        \note Contains data only if your mesh have textures (by loading it or calling \ref applyTexture).
         */
        std::vector<float2> uv;

        /**
        Texture of the Mesh.

        \note Contains data only if your mesh have textures (by loading it or calling \ref applyTexture).
         */
        Texture texture;

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
        void updateMeshFromChunkList(chunkList IDs = chunkList(0));

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
        There is only one texture for the mesh, the uv of each chunks are expressed for it in its globality.
        Vectors of vertices/normals and uv have now the same size.

        \param texture_format : define the number of channels desired for the computed texture. default : MESH_TEXTURE_RGB.
        \return True if the texturing was successful, false otherwise.

        \note This function can be called as long as you do not start a new spatial mapping process, due to shared memory.
        \note This function can require a lot of computation time depending on the number of triangles in the mesh. Its recommended to call it once a the end of your spatial mapping process.

        \warning The save_texture parameter in SpatialMappingParameters must be set as true when enabling the spatial mapping to be able to apply the textures.
        \warning The mesh should be filtered before calling this function since \ref filter will erase the textures, the texturing is also significantly slower on non-filtered meshes.
         */
        bool applyTexture(MESH_TEXTURE_FORMAT texture_format = MESH_TEXTURE_RGB);

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
        \param IDs : (by default empty) Specify a set of chunks to be saved, if none provided alls chunks are saved. default : (empty).
        \return True if the file was successfully saved, false otherwise.

        \note Only \ref MESH_FILE_OBJ support textures data.
        \note This function operate on the Mesh not on the chunks. This way you can save different parts of your Mesh (update your Mesh with \ref updateMeshFromChunkList).
         */
        bool save(String filename, MESH_FILE_FORMAT type = MESH_FILE_OBJ, chunkList IDs = chunkList(0));

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
    enum PLANE_TYPE {
        PLANE_TYPE_HORIZONTAL,
        PLANE_TYPE_VERTICAL,
        PLANE_TYPE_UNKNOWN,
        PLANE_TYPE_LAST
    };

    /**
    \brief Converts the given enumerated value into readable text.
    \return The enumerated value as a string.
     */
    String /*@cond SHOWHIDDEN*/SL_SCANNING_EXPORT/*@endcond*/ toString(const PLANE_TYPE &type);
    ///@cond

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
        friend class PlaneDetector;
        friend class PlaneDetectorAtHit;
        friend class Camera;
        ///@endcond
    public:

        Plane();
        ~Plane();

        /**
         * The plane type define the plane orientation : vertical or horizontal.
         \note It is deduced from the gravity vector and is therefore only available with the ZED-M.
          The ZED will give PLANE_TYPE_UNKNOWN for every planes.
         */
        PLANE_TYPE type = PLANE_TYPE_UNKNOWN;


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
        std::vector<sl::uint2> bounds2D_RcCi; // image plane
        std::vector<sl::float3> boundingRect_RuCu;
        sl::float2 sizeRect;
        sl::Transform planePose_RuCu;

        sl::float4 planeEquation_RuCu;
        sl::float4 centroid_RuCu;
        sl::Mesh mesh_RuCu; // not computed by default

        float fx_, fy_, cx, cy, unit_factor /* unit_user = unit_factor * cm */;
        sl::Transform transform_RcCi2RuCu;
        sl::Transform transform_Ci2Cu;
        sl::Transform planePose_axis_aligned; // in RuCu

        bool validity, isFloor;

        std::vector<sl::float3> computeBoundingRect();

    };

}

#endif /* MESH_HPP_ */