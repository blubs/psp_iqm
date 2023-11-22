#include <math.h>
#include "logging.h"
// #include "math.hpp"


#define IQM_MAGIC "INTERQUAKEMODEL"
#define IQM_VERSION_1 1
#define IQM_VERSION_2 2
#define IQM_MAX_BONES 256


typedef struct vec3_s {
    float x,y,z;
} vec3_t;


typedef struct vec2_s {
    float x,y;
} vec2_t;

typedef struct quat_s {
    float x,y,z;
    float w;
} quat_t;


// 
// Dot-product between two vectors
// 
float dot_vec3(vec3_t a, vec3_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}


// 
// Add two vec3_t structs
//
vec3_t add_vec3(vec3_t a, vec3_t b) {
    vec3_t result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// 
// Multiply a float by a vec3 
// 
vec3_t mul_float_vec3(float a, vec3_t b) {
    vec3_t result;
    result.x = a * b.x;
    result.y = a * b.y;
    result.z = a * b.z;
    return result;
}



// 
// Dot-product between two quaternions
// 
float dot_quat(quat_t a, quat_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}


// 
// Linearly interpolate between two floats
// 
float lerp_float(float a, float b, float lerpfrac) {
    return a + lerpfrac * (b - a);
}


// 
// Linear interpolation between two vectors
//
vec3_t lerp_vec3(vec3_t a, vec3_t b, float lerpfrac) {
    vec3_t result;
    result.x = lerp_float(a.x, b.x, lerpfrac);
    result.y = lerp_float(a.y, b.y, lerpfrac);
    result.z = lerp_float(a.z, b.z, lerpfrac);
    return result;
}


// 
// Linear interpolation between two quaternions
//
quat_t lerp_quat(const quat_t a, const quat_t b, float lerpfrac) {
    // If the quaternions are the same, return either one
    if(a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w) {
        return a;
    }
    quat_t result;
    result.x = lerp_float(a.x, b.x, lerpfrac);
    result.y = lerp_float(a.y, b.y, lerpfrac);
    result.z = lerp_float(a.z, b.z, lerpfrac);
    result.w = lerp_float(a.w, b.w, lerpfrac);
    return result;
}


// 
// Spherical linear interpolation between two quaternions
//
quat_t slerp_quat(const quat_t a, const quat_t b, float lerpfrac) {
    // If the quaternions are the same, return either one
    if(a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w) {
        return a;
    }
    quat_t result;
    float dot_product = dot_quat(a,b);
    // Avoid div by zero
    if(dot_product > 0.999f) {
        // Revert to linear interpolation
        return lerp_quat(a,b,lerpfrac);
    }

    // Interpolate the short way around
    quat_t c;
    if(dot_product < 0) {
        dot_product = -dot_product;
        c.x = -b.x;
        c.y = -b.y;
        c.z = -b.z;
        c.w = -b.w;
    }
    else {
        c.x = b.x;
        c.y = b.y;
        c.z = b.z;
        c.w = b.w;
    }

    float theta = acosf(dot_product);
    float c1 = sinf(theta * (1 - lerpfrac));
    float c2 = sinf(theta * lerpfrac);
    float c3 = 1.0 / sinf(theta);

    result.x = (a.x * c1 + c.x * c2) * c3;
    result.y = (a.y * c1 + c.y * c2) * c3;
    result.z = (a.z * c1 + c.z * c2) * c3;
    result.w = (a.w * c1 + c.w * c2) * c3;
    // Normalize
    float quat_nor = 1.0 / sqrtf(dot_quat(result,result));
    result.x *= quat_nor;
    result.y *= quat_nor;
    result.z *= quat_nor;
    result.w *= quat_nor;
    return result;
}



// 
// 3 row x 4 column matrix struct used to hold affine transforms
// 
// The general form of a 3D transform normally requires a 4x4 matrix, but by 
// enforcing transforms to be affine, we can get away with a 3x4 matrix.
//
// Think of this matrix as a 4x4 with the following form:
// 
//  [   M   p ]
//  [ 0 0 0 1 ]
//  
//  Where M is a 3x3 matrix containing the affine rotation / scaling component.
//  p is a 3x1 column vector containing the translation of the transform.
// 
// Given that these 3x4 matrices are treated as 4x4s, they are invertable.
// 
// These matrices are column-major, with the indices of `m` laid out as:
// 
// [  0  3  6  9 ]
// [  1  4  7 10 ]
// [  2  5  8 11 ] 
//
typedef struct mat3x4_s {
    float m[12];
} mat3x4_t;


// 
// 3 row x 3 column matrix struct used to hold the rotation / scale component
// of an affine 3D transform.
// 
// These matrices are column major, with the indices of `m` laid out as:
// 
// [  0  3  6 ]
// [  1  4  7 ]
// [  2  5  8 ] 
typedef struct mat3x3_s {
    float m[9];
} mat3x3_t;



// 
// Builds and returns a 3x4 transform matrix filled with the top 3 rows of a 4x4 identify matrix
// 
mat3x4_t identity_mat3x4() {
    mat3x4_t result;
    result.m[0] = 1.0f; result.m[3] = 0.0f; result.m[6] = 0.0f; result.m[9]  = 0.0f; 
    result.m[1] = 0.0f; result.m[4] = 1.0f; result.m[7] = 0.0f; result.m[10] = 0.0f; 
    result.m[2] = 0.0f; result.m[5] = 0.0f; result.m[8] = 1.0f; result.m[11] = 0.0f; 
    return result;
}




// 
// Builds and returns a 3x4 translation transform matrix
//
mat3x4_t translation_mat3x4(vec3_t pos) {
    mat3x4_t result = identity_mat3x4();
    result.m[9]  = pos.x;
    result.m[10] = pos.y;
    result.m[11] = pos.z;
    return result;
}


// 
// Builds and returns a 3x4 scale transform matrix
//
mat3x4_t scale_mat3x4(vec3_t scale) {
    mat3x4_t result = identity_mat3x4();
    result.m[0]  = scale.x;
    result.m[4] = scale.y;
    result.m[8] = scale.z;
    return result;
}


// 
// Builds and returns a 3x4 rotation transform matrix
//
mat3x4_t rotation_mat3x4(quat_t rot) {
    mat3x4_t result = identity_mat3x4();
    // Precomputing floating point multiplications
    float xx2 = 2.0f * rot.x * rot.x;
    float yy2 = 2.0f * rot.y * rot.y;
    float zz2 = 2.0f * rot.z * rot.z;
    float xz2 = 2.0f * rot.x * rot.z;
    float xy2 = 2.0f * rot.x * rot.y;
    float yz2 = 2.0f * rot.y * rot.z;
    float wx2 = 2.0f * rot.w * rot.x;
    float wy2 = 2.0f * rot.w * rot.y;
    float wz2 = 2.0f * rot.w * rot.z;
    result.m[0] = 1.0f - yy2 - zz2;
    result.m[1] = xy2 + wz2;
    result.m[2] = xz2 - wy2;
    result.m[3] = xy2 - wz2;
    result.m[4] = 1.0f - xx2 - zz2;
    result.m[5] = yz2 + wx2;
    result.m[6] = xz2 + wy2;
    result.m[7] = yz2 - wx2;
    result.m[8] = 1.0f - xx2 - yy2;
    return result;
}


// 
// Multiplies two 3x4 matrices together 
// (Made possible by treating them both as 4x4 matrices with the bottom row: [0 0 0 1])
//
mat3x4_t matmul_mat3x4_mat3x4(mat3x4_t a, mat3x4_t b) {
    mat3x4_t result = identity_mat3x4();
    result.m[0]  = a.m[0] * b.m[0]  + a.m[3] * b.m[1]  + a.m[6]  * b.m[2];
    result.m[1]  = a.m[1] * b.m[0]  + a.m[4] * b.m[1]  + a.m[7]  * b.m[2]; 
    result.m[2]  = a.m[2] * b.m[0]  + a.m[5] * b.m[1]  + a.m[8]  * b.m[2];

    result.m[3]  = a.m[0] * b.m[3]  + a.m[3] * b.m[4]  + a.m[6]  * b.m[5];
    result.m[4]  = a.m[1] * b.m[3]  + a.m[4] * b.m[4]  + a.m[7]  * b.m[5];
    result.m[5]  = a.m[2] * b.m[3]  + a.m[5] * b.m[4]  + a.m[8]  * b.m[5];

    result.m[6]  = a.m[0] * b.m[6]  + a.m[3] * b.m[7]  + a.m[6]  * b.m[8];
    result.m[7]  = a.m[1] * b.m[6]  + a.m[4] * b.m[7]  + a.m[7]  * b.m[8];
    result.m[8]  = a.m[2] * b.m[6]  + a.m[5] * b.m[7]  + a.m[8]  * b.m[8];

    result.m[9]  = a.m[0] * b.m[9]  + a.m[3] * b.m[10] + a.m[6]  * b.m[11] + a.m[9];
    result.m[10] = a.m[1] * b.m[9]  + a.m[4] * b.m[10] + a.m[7]  * b.m[11] + a.m[10];
    result.m[11] = a.m[2] * b.m[9]  + a.m[5] * b.m[10] + a.m[8]  * b.m[11] + a.m[11];
    return result;
}

//
// Builds and returns a 3x4 transform matrix from the corresponding translation vector, rotation quaternion, and scale vector
//
mat3x4_t translate_rotate_scale_mat3x4(vec3_t translation, quat_t rotation, vec3_t scale ) {
    mat3x4_t translation_mat = translation_mat3x4(translation);
    mat3x4_t rotation_mat = rotation_mat3x4(rotation);
    mat3x4_t scale_mat = scale_mat3x4(scale);

    // First scale, then rotate, then translate
    mat3x4_t result = matmul_mat3x4_mat3x4(translation_mat, matmul_mat3x4_mat3x4(rotation_mat, scale_mat));
    return result;
}



// 
// Gets the top-left 3x3 matrix from a 3x4 matrix
// 
mat3x3_t get_mat3x4_mat3x3(mat3x4_t mat) {
    mat3x3_t result;
    result.m[0] = mat.m[0]; result.m[3] = mat.m[3]; result.m[6] = mat.m[6]; 
    result.m[1] = mat.m[1]; result.m[4] = mat.m[4]; result.m[7] = mat.m[7]; 
    result.m[2] = mat.m[2]; result.m[5] = mat.m[5]; result.m[8] = mat.m[8]; 
    return result;
}


// 
// Inverts a 3x3 matrix
// https://stackoverflow.com/a/18504573
// 
mat3x3_t invert_mat3x3(mat3x3_t mat) {
    float det = mat.m[0] * (mat.m[4] * mat.m[8] - mat.m[7] * mat.m[5]) -
                mat.m[3] * (mat.m[1] * mat.m[8] - mat.m[7] * mat.m[2]) +
                mat.m[6] * (mat.m[1] * mat.m[5] - mat.m[4] * mat.m[2]);
    mat3x3_t result;
    // If determinant is close to 0, return 0 matrix:
    if(fabs(det) < 1e-5) {
        return result;
    }
    float inv_det = 1.0f / det;
    result.m[0] = (mat.m[4] * mat.m[8] - mat.m[5] * mat.m[7]) * inv_det;
    result.m[1] = (mat.m[2] * mat.m[7] - mat.m[1] * mat.m[8]) * inv_det;
    result.m[2] = (mat.m[1] * mat.m[5] - mat.m[2] * mat.m[4]) * inv_det;
    result.m[3] = (mat.m[5] * mat.m[6] - mat.m[3] * mat.m[8]) * inv_det;
    result.m[4] = (mat.m[0] * mat.m[8] - mat.m[2] * mat.m[6]) * inv_det;
    result.m[5] = (mat.m[2] * mat.m[3] - mat.m[0] * mat.m[5]) * inv_det;
    result.m[6] = (mat.m[3] * mat.m[7] - mat.m[4] * mat.m[6]) * inv_det;
    result.m[7] = (mat.m[1] * mat.m[6] - mat.m[0] * mat.m[7]) * inv_det;
    result.m[8] = (mat.m[0] * mat.m[4] - mat.m[1] * mat.m[3]) * inv_det;
    return result;
}

// 
// Transposes a 3x3 matrix
//
mat3x3_t transpose_mat3x3(mat3x3_t mat) {
    mat3x3_t result;
    result.m[0] = mat.m[0];
    result.m[1] = mat.m[3];
    result.m[2] = mat.m[6];
    result.m[3] = mat.m[1];
    result.m[4] = mat.m[4];
    result.m[5] = mat.m[7];
    result.m[6] = mat.m[2];
    result.m[7] = mat.m[5];
    result.m[8] = mat.m[8];
    return result;
}


// 
// Multiplies 3x3 matrix * 3x1 column vector vec
// Returns a 3x1 column vector
// 
vec3_t mul_mat3x3_vec3(mat3x3_t mat, vec3_t vec) {
    vec3_t result;
    result.x = mat.m[0] * vec.x + mat.m[3] * vec.y + mat.m[6] * vec.z;
    result.y = mat.m[1] * vec.x + mat.m[4] * vec.y + mat.m[7] * vec.z;
    result.z = mat.m[2] * vec.x + mat.m[5] * vec.y + mat.m[8] * vec.z;
    return result;
}


// 
// Inverts a 3x4 matrix by treating it as a 4x4 affine transform matrix.
// https://stackoverflow.com/a/2625420
// 
mat3x4_t invert_mat3x4(mat3x4_t mat) {
    // Get the top-left 3x3 matrix from the 3x4 matrix, and invert it
    mat3x3_t inv_m3x3 = invert_mat3x3(get_mat3x4_mat3x3(mat));

    // Get the translation vector from the mat3x4 matrix
    vec3_t translation;
    translation.x = mat.m[9];
    translation.y = mat.m[10];
    translation.z = mat.m[11];

    // Multiply: (-1.0 * inv_m3x3 * translation)
    vec3_t inv_translation = mul_float_vec3(-1.0f, mul_mat3x3_vec3(inv_m3x3, translation));

    mat3x4_t result;
    // Copy in inv_m3x3 as the upper-left 3x3
    result.m[0] = inv_m3x3.m[0]; result.m[3] = inv_m3x3.m[3]; result.m[6] = inv_m3x3.m[6];
    result.m[1] = inv_m3x3.m[1]; result.m[4] = inv_m3x3.m[4]; result.m[7] = inv_m3x3.m[7];
    result.m[2] = inv_m3x3.m[2]; result.m[5] = inv_m3x3.m[5]; result.m[8] = inv_m3x3.m[8];
    // Copy in inv_translation as the right-most column
    result.m[9]  = inv_translation.x;
    result.m[10] = inv_translation.y; 
    result.m[11] = inv_translation.z; 
    return result;
}




// 
// Multiplies 3x4 matrix * 3x1 column vector vec.
//
// This function treats the matrix like a 4x4 with bottom row: [0 0 0 1]
// This function treats the vector as a 4x1 column vector: [vec.x vec.y vec.z 1]
//
// Returns a 3x1 column vector
// 
vec3_t mul_mat3x4_vec3(mat3x4_t mat, vec3_t vec) {
    vec3_t result;
    result.x = mat.m[0] * vec.x + mat.m[3] * vec.y + mat.m[6] * vec.z + mat.m[9];
    result.y = mat.m[1] * vec.x + mat.m[4] * vec.y + mat.m[7] * vec.z + mat.m[10];
    result.z = mat.m[2] * vec.x + mat.m[5] * vec.y + mat.m[8] * vec.z + mat.m[11];
    return result;
}


typedef struct iqm_header_s {
    char magic[16];
    uint32_t version;
    uint32_t filesize;
    uint32_t flags;
    uint32_t n_text, ofs_text;
    uint32_t n_meshes, ofs_meshes;
    uint32_t n_vert_arrays, n_verts, ofs_vert_arrays;
    uint32_t n_tris, ofs_tris, ofs_adjacency;
    uint32_t n_joints, ofs_joints;
    uint32_t n_poses, ofs_poses;
    uint32_t n_anims, ofs_anims;
    uint32_t n_frames, n_frame_channels, ofs_frames, ofs_bounds;
    uint32_t n_comments, ofs_comments;
    uint32_t n_extensions, ofs_extensions;
} iqm_header_t;

typedef struct iqm_mesh_s {
    uint32_t name;
    uint32_t material;
    uint32_t first_vert_idx, n_verts;
    uint32_t first_tri, n_tris;
} iqm_mesh_t;

enum class iqm_vert_array_type : uint32_t {
    IQM_VERT_POS            = 0,
    IQM_VERT_UV             = 1,
    IQM_VERT_NOR            = 2,
    IQM_VERT_TAN            = 3,
    IQM_VERT_BONE_IDXS      = 4,
    IQM_VERT_BONE_WEIGHTS   = 5,
    IQM_VERT_COLOR          = 6,
    IQM_VERT_CUSTOM         = 0X10
};

enum class iqm_dtype : uint32_t {
    IQM_DTYPE_BYTE          = 0,
    IQM_DTYPE_UBYTE         = 1,
    IQM_DTYPE_SHORT         = 2,
    IQM_DTYPE_USHORT        = 3,
    IQM_DTYPE_INT           = 4,
    IQM_DTYPE_UINT          = 5,
    IQM_DTYPE_HALF          = 6,
    IQM_DTYPE_FLOAT         = 7,
    IQM_DTYPE_DOUBLE        = 8,
};


typedef struct iqm_tri_s {
    uint32_t vert_idxs[3];
} iqm_tri_t;


typedef struct iqm_joint_euler_s {
    uint32_t name;
    int32_t parent_joint_idx;
    float translate[3], rotate[3], scale[3];
} iqm_joint_euler_t;


typedef struct iqm_joint_quaternion_s {
    uint32_t name;
    int32_t parent_joint_idx;
    float translate[3], rotate[4], scale[3];
} iqm_joint_quaternion_t;


typedef struct iqm_pose_euler_s {
    int32_t parent_idx; // Parent POSE idx? parent bone idx?
    uint32_t mask;
    float channel_ofs[9];
    float channel_scale[9];
} iqm_pose_euler_t;


typedef struct iqm_pose_quaternion_s {
    int32_t parent_idx; // Parent POSE idx? parent bone idx?
    uint32_t mask;
    float channel_ofs[10];
    float channel_scale[10];
} iqm_pose_quaternion_t;

typedef struct iqm_anim_s {
    uint32_t name;
    uint32_t first_frame, n_frames;
    float framerate;
    uint32_t flags;
} iqm_anim_t;


enum class iqm_anim_flag : uint32_t {
    IQM_ANIM_FLAG_LOOP = 1<<0
};


typedef struct iqm_vert_array_s {
    uint32_t type; // TODO - iqm_vert_array_type?
    uint32_t flags;
    uint32_t format;// TODO - iqm_dtype
    uint32_t size;
    uint32_t ofs;
} iqm_vert_array_t;


typedef struct iqm_bounds_s {
    float mins[3], maxs[3];
    float xyradius;
    float radius;
} iqm_bounds_t;


typedef struct iqm_extension_s {
    uint32_t name;
    uint32_t n_data, ofs_data;
    uint32_t ofs_extensions; // Linked list pointer to next extension. (As byte offset from start of IQM file)
} iqm_extension_t;


typedef struct iqm_ext_fte_mesh_s {
    uint32_t contents;      // default: CONTENTS_BODY (0x02000000)
    uint32_t surfaceflags;  // Propagates to trace_surfaceflags
    uint32_t surfaceid;     // Body reported to QC via trace_surface
    uint32_t geomset;
    uint32_t geomid;
    float min_dist;
    float max_dist;
} iqm_ext_fte_mesh_t;


typedef struct iqm_ext_fte_event_s {
    uint32_t anim;
    float timestamp;
    uint32_t event_code;
    uint32_t event_data_str;        // Stringtable
} iqm_ext_fte_event_t;


typedef struct iqm_ext_fte_skin_s {
    uint32_t n_skinframes;
    uint32_t n_meshskins;
} iqm_ext_fte_skin_t;


typedef struct iqm_ext_fte_skin_skinframe_s {
    uint32_t material_idx;
    uint32_t shadertext_idx;
} iqm_ext_fte_skin_skinframe_t;


typedef struct iqm_ext_fte_skin_meshskin_s {
    uint32_t first_frame;
    uint32_t n_frames;
    float interval;
} iqm_ext_fte_skin_meshskin_t;


typedef struct vertex_s {
    float u, v;
    uint32_t color;
    float nor_x, nor_y, nor_z;
    float x,y,z;
} vertex_t;



typedef struct skeletal_mesh_s {
    // Number of vertices in mesh
    uint32_t n_verts;
    // Number of triangles in mesh
    uint32_t n_tris;
    uint32_t n_tri_verts;
    uint32_t first_vert;
    // Contains vertex indices
    uint16_t *tri_verts = nullptr;

    // TODO - Geomset identifiers...
    // TODO - Material identifiers...
    // TODO - Other vertex attributes (bone_idxs, bone_weights, normals, tangents, colors)
    // TODO - An array with pre-allocated memory for cur_posed_vertices after a skeleton pose is applied
} skeletal_mesh_t;



typedef struct skeletal_model_s {
    // List of all mesh vertices
    uint32_t n_verts;
    vec3_t *vert_rest_positions; // Contains the rest position of all vertices
    vec3_t *vert_rest_normals; // Contains the rest normals of all vertices
    // Contains the drawn vertex struct.
    // The vertex positions in this array are updated whenever a skeleton pose is applied via `apply_skeleton_pose`
    vertex_t *verts = nullptr;
    float *vert_bone_weights; // 4 bone weights per vertex
    uint8_t *vert_bone_idxs; // 4 bone indices per vertex


    // Model mins / maxs across all animation frames
    vec3_t mins;
    vec3_t maxs;


    // List of meshes
    uint32_t n_meshes;
    skeletal_mesh_t *meshes;


    // List of bones
    uint32_t n_bones;
    char **bone_name;
    int16_t *bone_parent_idx;

    vec3_t *bone_rest_pos;
    quat_t *bone_rest_rot;
    vec3_t *bone_rest_scale;
    // The per-bone transform that takes us from rest-pose model-space to bone local space
    // These are static, so compute them once
    mat3x4_t *bone_rest_transforms;
    mat3x4_t *inv_bone_rest_transforms;
    


    // Animation frames data
    uint16_t n_frames;
    // TODO - IQM has a parent index for each pose, do we need it? is it always the same as the parent's bone parent idx?
    vec3_t *frames_bone_pos;
    quat_t *frames_bone_rot;
    vec3_t *frames_bone_scale;


    // Animation framegroup data
    uint16_t n_framegroups;
    char **framegroup_name;
    uint32_t *framegroup_start_frame;
    uint32_t *framegroup_n_frames;
    float *framegroup_fps;
    bool *framegroup_loop;


    // Animation framegroup FTE events data
    uint16_t *framegroup_n_events;
    float **framegroup_event_time;
    uint32_t **framegroup_event_code;
    char ***framegroup_event_data_str;


    // TODO - Bone data (rest pose matrices, inverse rest pose matrices)
    // -----------------------------------

} skeletal_model_t;



typedef struct skeletal_skeleton_s {
    const skeletal_model_t *model; // Animation / skeleton data is pulled from this model
    // Holds last built bone transforms
    mat3x4_t *bone_transforms;
    // Holds the 3x3 inverse-transpose of the above transform
    // Used to transform normals from rest pose to current pose
    mat3x3_t *bone_normal_transforms;

    // anim_bone_idx[i] contains the ith skeleton bone's index in the animation model bone list last used to build the skeleton
    // If anim_bone_idx[i] == -1, the skeleton's i-th bone was not found in the model's animation. This bone will not be animated.
    int *anim_bone_idx;
    // Contains a pointer to the last skeletal model we used to build the skeleton.
    skeletal_model_t *prev_anim_model;

} skeletal_skeleton_t;






// 
// Returns true if `bone_idx` is in list `bone_list`
// 
bool bone_in_list(uint8_t bone_idx, const uint8_t *bone_list, const int n_bones) {
    for(int i = 0; i < n_bones; i++) {
        if(bone_idx == bone_list[i]) {
            return true;
        }
    }
    return false;
}

//
// Returns true if list `bone_list_a` is a subset of `bone_list_b`
//
bool bone_list_is_subset(const uint8_t *bone_list_a, const int bone_list_a_n_bones, const uint8_t *bone_list_b, const int bone_list_b_n_bones) {
    for(int i = 0; i < bone_list_a_n_bones; i++) {
        bool bone_in_list = false;
        for(int j = 0; j < bone_list_b_n_bones; j++) {
            if(bone_list_a[i] == bone_list_b[j]) {
                bone_in_list = true;
                break;
            }
        }
        // If any bone was missing from list b, not a subset
        if(!bone_in_list) {
            return false;
        }
    }
    // All `a` bones were found in `b`, `a` is a subset of `b`.
    return true;
}

//
// Returns the number of bones that are in `bone_list_a` but not in `bone_list_b`
//  i.e. len(set(bone_list_a) - set(bone_list_b))
//
int bone_list_set_difference(const uint8_t *bone_list_a, const int bone_list_a_n_bones, const uint8_t *bone_list_b, const int bone_list_b_n_bones) {
    int n_missing_bones = 0;

    for(int i = 0; i < bone_list_a_n_bones; i++) {
        bool bone_in_list = false;
        for(int j = 0; j < bone_list_b_n_bones; j++) {
            if(bone_list_a[i] == bone_list_b[j]) {
                bone_in_list = true;
                break;
            }
        }
        // If bone was missing from list b, count it
        if(!bone_in_list) {
            n_missing_bones += 1;
        }
    }
    return n_missing_bones;
}

//
// Performs the set union of bones in both `bone_list_a` and `bone_list_b`
// Writes the union into `bone_list_a`
//  i.e. len(set(bone_list_a) + set(bone_list_b))
//
// WARNING: Assumes `bone_list_a` has the capacity for the union of both lists.
//
int bone_list_set_union(uint8_t *bone_list_a, int bone_list_a_n_bones, const uint8_t *bone_list_b, const int bone_list_b_n_bones) {
    for(int i = 0; i < bone_list_b_n_bones; i++) {
        bool bone_already_in_a = false;

        for(int j = 0; j < bone_list_a_n_bones; j++) {
            if(bone_list_a[j] == bone_list_b[i]) {
                bone_already_in_a = true;
                break;
            }
        }
        if(bone_already_in_a) {
            continue;
        }

        bone_list_a[bone_list_a_n_bones] = bone_list_b[i];
        bone_list_a_n_bones += 1;
    }
    return bone_list_a_n_bones;
}



// 
// Splits each mesh in `skel_model` into submeshes that reference no more than 8 bones.
//
void submesh_skeletal_model(skeletal_model_t *skel_model) {
    const int VERT_BONES = 4;
    const int TRI_VERTS = 3;
    const int SUBMESH_BONES = 8;
    log_printf("=========== Submesh started =============\n");


    // For each mesh, break it up into submeshes
    for(int i = 0; i < skel_model->n_meshes; i++) {

        // log_printf("Mesh triangles: %d\n",skel_model->meshes[i].n_tris);
        // if(skel_model->meshes[i].n_tris < 50) {
        //     log_printf("///////////// Debugging mesh triangles:\n");

        //     for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
        //         log_printf("\tTri %d vert indices: ", tri_idx);
        //         for(int tri_vert_idx = 0; tri_vert_idx < TRI_VERTS; tri_vert_idx++ ) {
        //             log_printf("%d, ", skel_model->meshes[i].tri_verts[(tri_idx * TRI_VERTS) + tri_vert_idx]);
        //         }
        //         log_printf("\n");
        //     }
        // }
        // continue;

        // --------------------------------------------------------------------
        // Build the set of bones referenced by each triangle
        // --------------------------------------------------------------------
        uint8_t *tri_n_bones = (uint8_t*) malloc(sizeof(uint8_t) * skel_model->meshes[i].n_tris); // Contains the number of bones that the i-th triangle references
        uint8_t *tri_bones = (uint8_t*) malloc(sizeof(uint8_t) * TRI_VERTS * VERT_BONES * skel_model->meshes[i].n_tris); // Contains the list of bones referenced by the i-th triangle


        for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
            // Initialize this triangle's bone list to 0s
            for(int tri_bone_idx = 0; tri_bone_idx < TRI_VERTS * VERT_BONES; tri_bone_idx++) {
                tri_bones[(tri_idx * TRI_VERTS * VERT_BONES) + tri_bone_idx] = 0;
            }
            tri_n_bones[tri_idx] = 0;

            for(int tri_vert_idx = 0; tri_vert_idx < TRI_VERTS; tri_vert_idx++ ) {
                int vert_idx = skel_model->meshes[i].tri_verts[(tri_idx * TRI_VERTS) + tri_vert_idx] + skel_model->meshes[i].first_vert;
                // Loop through the vertex's referenced bones
                for(int vert_bone_idx = 0; vert_bone_idx < VERT_BONES; vert_bone_idx++) {
                    uint8_t bone_idx = skel_model->vert_bone_idxs[vert_idx * VERT_BONES + vert_bone_idx];
                    float bone_weight = skel_model->vert_bone_weights[vert_idx * VERT_BONES + vert_bone_idx];

                    if(bone_weight > 0) {
                        // Verify the bone is not already in this triangle's bone list
                        if(!bone_in_list(bone_idx, &tri_bones[tri_idx * TRI_VERTS * VERT_BONES], tri_n_bones[tri_idx])) {
                            tri_bones[(tri_idx * TRI_VERTS * VERT_BONES) + tri_n_bones[tri_idx]] = bone_idx;
                            tri_n_bones[tri_idx] += 1;
                        }
                    }
                }
            }
        }
        // --------------------------------------------------------------------


        // // Debug print a few triangle bone lists:
        // for(int j = 0; j < 10; j++) {
        //     log_printf("Mesh: %d tri: %d bones (%d): ", i, j, tri_n_bones[j]);
        //     for(int k = 0; k < tri_n_bones[j]; k++) {
        //         log_printf("%d, ", tri_bones[j * TRI_VERTS * VERT_BONES + k]);
        //     }
        //     log_printf("\n");
        // }
        // break;

        // --------------------------------------------------------------------
        // Assign each triangle in the mesh to a submesh idx
        // --------------------------------------------------------------------
        int8_t *tri_submesh_idx = (int8_t*) malloc(sizeof(int8_t) * skel_model->meshes[i].n_tris); // Contains the set the i-th triangle belongs to
        const int SET_DISCARD = -2; // Discarded triangles
        const int SET_UNASSIGNED = -1; // Denotes unassigned triangles
        for(int j = 0; j < skel_model->meshes[i].n_tris; j++) {
            tri_submesh_idx[j] = SET_UNASSIGNED;
        }

        int cur_submesh_idx = -1;

        while(true) {
            // Find the unassigned triangle that uses the most bones:
            int cur_tri = -1;
            for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                // If this triangle isn't `UNASSIGNED`, skip it
                if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                    continue;
                }
                // If we haven't found one yet, set it
                if(cur_tri == -1) {
                    cur_tri = tri_idx;
                    continue;
                }
                // If this triangle references more bones, update cur_tri
                if(tri_n_bones[tri_idx] > tri_n_bones[cur_tri]) {
                    cur_tri = tri_idx;
                }
            }

            // If we didn't find any triangles, stop submesh algorithm. We're done.
            if(cur_tri == -1) {
                break;
            }

            cur_submesh_idx += 1;
            int cur_submesh_n_bones = 0;
            uint8_t *cur_submesh_bones = (uint8_t*) malloc(sizeof(uint8_t) * SUBMESH_BONES);
            log_printf("Creating submesh %d for mesh %d\n", cur_submesh_idx, i);

            // Verify the triangle doesn't have more than the max bones allowed:
            if(tri_n_bones[cur_tri] > SUBMESH_BONES) {
                log_printf("Warning: Mesh %d Triangle %d references %d bones, which is more than the maximum allowed for any mesh (%d). Skipping triangle...\n", i, cur_tri, tri_n_bones[cur_tri], SUBMESH_BONES);
                // Discard it
                tri_submesh_idx[cur_tri] = SET_DISCARD;
                continue;
            }

            // Add the triangle to the current submesh:
            tri_submesh_idx[cur_tri] = cur_submesh_idx;

            // Add the triangle's bones to the current submesh:
            for(int submesh_bone_idx = 0; submesh_bone_idx < tri_n_bones[cur_tri]; submesh_bone_idx++) {
                cur_submesh_bones[submesh_bone_idx] = tri_bones[(cur_tri * TRI_VERTS * VERT_BONES) + submesh_bone_idx];
                cur_submesh_n_bones += 1;
            }

            log_printf("\tstart submesh bones (%d): [", cur_submesh_n_bones);
            for(int submesh_bone_idx = 0; submesh_bone_idx < cur_submesh_n_bones; submesh_bone_idx++) {
                log_printf("%d, ", cur_submesh_bones[submesh_bone_idx]);
            }
            log_printf("]\n");

            // Add all unassigned triangles from the main mesh that references bones in this submesh's bone list
            for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                // If this triangle isn't `UNASSIGNED`, skip it
                if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                    continue;
                }

                // if(i == 0) {
                //     log_printf("Mesh %d submesh %d checking tri %d\n",i,cur_submesh_idx,tri_idx);
                //     log_printf("\tTri bones: (%d), ", tri_n_bones[tri_idx]);
                //     for(int tri_bone_idx = 0; tri_bone_idx < tri_n_bones[tri_idx]; tri_bone_idx++) {
                //         log_printf("%d,", tri_bones[(tri_idx * TRI_VERTS * VERT_BONES) + tri_bone_idx]);
                //     }
                //     log_printf("\n");
                // }

                // If this triangle's bones is not a subset of the current submesh bone list, skip it
                if(!bone_list_is_subset(&tri_bones[tri_idx * TRI_VERTS * VERT_BONES], tri_n_bones[tri_idx], cur_submesh_bones, cur_submesh_n_bones)) {
                    continue;
                }

                // Otherwise, it is a subset, add it to the current submesh
                tri_submesh_idx[tri_idx] = cur_submesh_idx;
            }


            // Print how many triangles belong to the current submesh
            int cur_submesh_n_tris = 0;
            int n_assigned_tris = 0;
            for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                    n_assigned_tris++;
                }
                if(tri_submesh_idx[tri_idx] == cur_submesh_idx) {
                    cur_submesh_n_tris++;
                }
            }
            log_printf("\tcur submesh (%d) n_tris: %d/%d, remaining unassigned: %d/%d\n", cur_submesh_idx, cur_submesh_n_tris, skel_model->meshes[i].n_tris, n_assigned_tris, skel_model->meshes[i].n_tris);


            // Repeat until there are no unassigned triangles remaining:
            while(true) {
                // Get triangle with the minimum number of bones not in the current submesh bone list
                cur_tri = -1;
                int cur_tri_n_missing_bones = 0;
                for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                    // If this triangle isn't `UNASSIGNED`, skip it
                    if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                        continue;
                    }
                    // Count the number of bones referenced by this triangle that are not in the current submesh bone list
                    int n_missing_bones = bone_list_set_difference(&tri_bones[tri_idx * TRI_VERTS * VERT_BONES], tri_n_bones[tri_idx], cur_submesh_bones, cur_submesh_n_bones);
                    if(cur_tri == -1 || n_missing_bones < cur_tri_n_missing_bones) {
                        cur_tri = tri_idx;
                        cur_tri_n_missing_bones = n_missing_bones;
                    }
                }

                // If no triangle found, stop. We're done.
                if(cur_tri == -1) {
                    log_printf("\tNo more unassigned triangles. Done with mesh.\n");
                    break;
                }

                // If this triangle pushes us past the submesh-bone limit, we are done with this submesh. Move onto the next one.
                if(cur_submesh_n_bones + cur_tri_n_missing_bones > SUBMESH_BONES) {
                    log_printf("\tReached max number of bones allowed. Done with submesh.\n");
                    break;
                }

                log_printf("\tNext loop using triangle: %d, missing bones: %d\n", cur_tri, cur_tri_n_missing_bones);


                // Assign the triangle to the current submesh
                tri_submesh_idx[cur_tri] = cur_submesh_idx;

                // Add this triangle's bones to the current submesh list of bones
                cur_submesh_n_bones = bone_list_set_union( cur_submesh_bones, cur_submesh_n_bones, &tri_bones[cur_tri * TRI_VERTS * VERT_BONES], tri_n_bones[cur_tri]);


                log_printf("\tcur submesh bones (%d): [", cur_submesh_n_bones);
                for(int submesh_bone_idx = 0; submesh_bone_idx < cur_submesh_n_bones; submesh_bone_idx++) {
                    log_printf("%d, ", cur_submesh_bones[submesh_bone_idx]);
                }
                log_printf("]\n");


                // Add all unassigned triangles from the main mesh that reference bones in this submesh's bone list
                for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                    // If this triangle isn't `UNASSIGNED`, skip it
                    if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                        continue;
                    }

                    // if(i == 0) {
                    //     log_printf("Mesh %d submesh %d checking tri %d\n",i,cur_submesh_idx,tri_idx);
                    //     log_printf("\tTri bones: (%d), ", tri_n_bones[tri_idx]);
                    //     for(int tri_bone_idx = 0; tri_bone_idx < tri_n_bones[tri_idx]; tri_bone_idx++) {
                    //         log_printf("%d,", tri_bones[(tri_idx * TRI_VERTS * VERT_BONES) + tri_bone_idx]);
                    //     }
                    //     log_printf("\n");
                    // }

                    // If this triangle's bones is not a subset of the current submesh bone list, skip it
                    if(!bone_list_is_subset(&tri_bones[tri_idx * TRI_VERTS * VERT_BONES], tri_n_bones[tri_idx], cur_submesh_bones, cur_submesh_n_bones)) {
                        continue;
                    }

                    // Otherwise, it is a subset, add it to the current submesh
                    tri_submesh_idx[tri_idx] = cur_submesh_idx;
                }

                // Print how many triangles belong to the current submesh
                cur_submesh_n_tris = 0;
                n_assigned_tris = 0;
                for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                    if(tri_submesh_idx[tri_idx] != SET_UNASSIGNED) {
                        n_assigned_tris++;
                    }
                    if(tri_submesh_idx[tri_idx] == cur_submesh_idx) {
                        cur_submesh_n_tris++;
                    }
                }
                log_printf("\tDone adding new tris for cur triangle");
                log_printf("\tcur submesh (%d) n_tris: %d/%d, total assigned: %d/%d\n", cur_submesh_idx, cur_submesh_n_tris, skel_model->meshes[i].n_tris, n_assigned_tris, skel_model->meshes[i].n_tris);
            }

            free(cur_submesh_bones);
        }


        int n_submeshes = cur_submesh_idx + 1;

        for(int submesh_idx = 0; submesh_idx < n_submeshes; submesh_idx++) {
            log_printf("Reconstructing submesh %d for mesh %d\n", submesh_idx, i);
            // Count the number of triangles that have been assigned to this submesh
            int submesh_tri_count = 0;
            for(int tri_idx = 0; tri_idx < skel_model->meshes[i].n_tris; tri_idx++) {
                if(tri_submesh_idx[tri_idx] == submesh_idx) {
                    submesh_tri_count++;
                }
            }

            // Allocate enough memory to fit submesh mesh triangle indices
            uint16_t *submesh_mesh_tri_idxs = (uint16_t*) malloc(sizeof(uint16_t) * submesh_tri_count); // Indices into mesh list of triangles
             // FIXME - Do we ever actually read this ^^^?

            // Allocate enough memory to fit theoretical max amount of unique vertes this model can reference (given we know its triangle count)
            uint16_t *submesh_mesh_vert_idxs = (uint16_t*) malloc(sizeof(uint16_t) * TRI_VERTS * submesh_tri_count); // Indices into mesh list of vertices
            // Allocate enough memoery to fit 3 vertex indices per triangle
            uint16_t *submesh_tri_verts = (uint16_t*) malloc(sizeof(uint16_t) * TRI_VERTS * submesh_tri_count);
            int submesh_n_tris = 0;
            int submesh_n_verts = 0;

            // ----------------------------------------------------------------
            // Build this submesh's list of triangle indices, vertex list, and
            // triangle vertex indices
            // ----------------------------------------------------------------
            for(int mesh_tri_idx = 0; mesh_tri_idx < skel_model->meshes[i].n_tris; mesh_tri_idx++) {
                // Skip triangles that don't belong to this submesh
                if(tri_submesh_idx[mesh_tri_idx] != submesh_idx) {
                    continue;
                }

                // Add the triangle to our submesh's list of triangles
                int submesh_tri_idx = submesh_n_tris;
                submesh_mesh_tri_idxs[submesh_tri_idx] = mesh_tri_idx; // FIXME - Do we ever actually read this?
                submesh_n_tris += 1;

                // Add each of the triangle's verts to the submesh list of verts
                // If that vertex is already in the submesh, use that index instead
                for(int tri_vert_idx = 0; tri_vert_idx < TRI_VERTS; tri_vert_idx++) {
                    int mesh_vert_idx = skel_model->meshes[i].tri_verts[(mesh_tri_idx * TRI_VERTS) + tri_vert_idx] + skel_model->meshes[i].first_vert;
                    // FIXME - This is a pointer into the full model vert indices list... Do we instead want the index into the mesh's vertex list?

                    // Check if this vertex is already in the submesh
                    int submesh_vert_idx = -1;
                    for(int j = 0; j < submesh_n_verts; j++) {
                        if(submesh_mesh_vert_idxs[j] == mesh_vert_idx) {
                            submesh_vert_idx = j;
                            break;
                        }
                    }
                    // If we didn't find the vertex in the submesh vertex list, add it
                    if(submesh_vert_idx == -1) {
                        submesh_vert_idx = submesh_n_verts;
                        submesh_mesh_vert_idxs[submesh_n_verts] = mesh_vert_idx;
                        submesh_n_verts += 1;
                    }

                    // Store the submesh vert idx for this triangle
                    submesh_tri_verts[(submesh_tri_idx * TRI_VERTS) + tri_vert_idx] = submesh_vert_idx;
                }
            }
            // ----------------------------------------------------------------


            free(submesh_mesh_tri_idxs);
            free(submesh_mesh_vert_idxs);
            free(submesh_tri_verts);

        }



        // TODO: - Free these:
        free(tri_n_bones);
        free(tri_bones);
        free(tri_submesh_idx);
        // --------------------------------------------------------------------

        // -------------------------------------------------------
    }


    




    for(int i = 0; i < skel_model->n_meshes; i++) {
        // TODO - Will need to dynamically allocate enough submeshes to fit.
    }

    // skel_model->meshes[i].tri_verts[j*3 + 0] = vert_a;
}



//
// Creates a `Skeletal_Skeleton` object that uses this model's skeleton / animation skeleton pose information.
//
skeletal_skeleton_t *create_skeleton(skeletal_model_t *model) {
    skeletal_skeleton_t *skeleton = (skeletal_skeleton_t*) malloc(sizeof(skeletal_skeleton_t));
    skeleton->model = model;
    skeleton->bone_transforms = (mat3x4_t*) malloc(sizeof(mat3x4_t) * skeleton->model->n_bones);
    skeleton->bone_normal_transforms = (mat3x3_t*) malloc(sizeof(mat3x3_t) * skeleton->model->n_bones);
    skeleton->anim_bone_idx = (int*) malloc(sizeof(int) * skeleton->model->n_bones);
    skeleton->prev_anim_model = nullptr;
    return skeleton;
}


// 
// Processes an animation's elapsed framegroup events.
//
void process_anim_events(skeletal_model_t *model, int framegroup_idx, float start_frametime, float end_frametime) {
    // TODO - Loop through the list of events (already sorted by frametime) and execute the callback for each event within the window.
    // TODO - Decide if the window domain should be closed on either side.
    // TODO - Add event callback somehow to this function
}


//
// Sets a skeleton's current pose matrices using the animation data from `source_model`
//
//  skeleton -- Skeleton object to set current pose for
//  source_model -- IQM model from which to pull bone animation data. This need not be the same as skeleton's.
//  framegroup_idx -- Framegroup index for framegroups in `source_model`
//  frametime -- Time into the framegroup animation in `source_model`
//
void build_skeleton(skeletal_skeleton_t *skeleton, skeletal_model_t *source_model, int framegroup_idx, float frametime) {

    if(skeleton->model != source_model) {
        // If we already found the mapping for this model, skip it
        if(skeleton->prev_anim_model != source_model) {
            skeleton->prev_anim_model = source_model;

            for(uint32_t i = 0; i < skeleton->model->n_bones; i++) {
                // Default to -1, indicating that no bones have been found yet.
                skeleton->anim_bone_idx[i] = -1;
                for(uint32_t j = 0; j < source_model->n_bones; j++) {
                    if(!strcmp( skeleton->model->bone_name[i], source_model->bone_name[j])) {
                        skeleton->anim_bone_idx[i] = j;
                        break;
                    }
                }
            }
        }
    }
    else {
        // If we already found the mapping for this model, skip it
        if(skeleton->prev_anim_model != source_model) {
            skeleton->prev_anim_model = source_model;
            for(uint32_t i = 0; i < skeleton->model->n_bones; i++) {
                skeleton->anim_bone_idx[i] = i;
            }
        }
    }


    if(framegroup_idx < 0 || framegroup_idx >= source_model->n_framegroups) {
        return;
    }

    // Find the two nearest frames to interpolate between
    int frame1_idx = (int) floor(frametime * source_model->framegroup_fps[framegroup_idx]);
    int frame2_idx = frame1_idx + 1;
    // float delta_frametime = 1.0f / source_model->framegroup_fps[framegroup_idx];
    // float lerpfrac = fmod(frametime, delta_frametime) / delta_frametime;

    if(source_model->framegroup_loop[framegroup_idx]) {
        frame1_idx = frame1_idx % source_model->framegroup_n_frames[framegroup_idx];
        frame2_idx = frame2_idx % source_model->framegroup_n_frames[framegroup_idx];
    }
    else {
        frame1_idx = std::min(std::max(0, frame1_idx), (int) source_model->framegroup_n_frames[framegroup_idx] - 1);
        frame2_idx = std::min(std::max(0, frame2_idx), (int) source_model->framegroup_n_frames[framegroup_idx] - 1);
    }

    float lerpfrac;
    if(frame1_idx == frame2_idx) {
        lerpfrac = 0.0f;
    }
    else {
        // FIXME - I suspect there's a simpler way to calculate this... that still relies on frame1_time
        float delta_frametime = 1.0f / source_model->framegroup_fps[framegroup_idx];
        float anim_duration = delta_frametime * source_model->framegroup_n_frames[framegroup_idx];
        float frame1_time = frame1_idx * delta_frametime;
        lerpfrac = (fmod(frametime,anim_duration) - frame1_time) / delta_frametime;
        // Floating point math can make it go slightly out of bounds:
        lerpfrac = std::min(std::max(0.0f, lerpfrac), 1.0f);
    }
    frame1_idx = source_model->framegroup_start_frame[framegroup_idx] + frame1_idx;
    frame2_idx = source_model->framegroup_start_frame[framegroup_idx] + frame2_idx;
    // TODO - If lerpfrac is 0.0 or 1.0, don't interpolate, just copy the bone poses

    log_printf("build_skeleton. frametime: %f, (%d, %d, %f)\n", frametime, frame1_idx, frame2_idx, lerpfrac);



    // Build the transform that takes us from model-space to each bone's bone-space for the current pose.
    for(uint32_t i = 0; i < skeleton->model->n_bones; i++) {
        const int anim_bone_idx = skeleton->anim_bone_idx[i];
        // If this bone is not present in the model's animation data, skip it.
        if(anim_bone_idx == -1) {
            skeleton->bone_transforms[i] = skeleton->model->bone_rest_transforms[i];
            continue;
        }

        vec3_t frame1_pos = source_model->frames_bone_pos[source_model->n_bones * frame1_idx + anim_bone_idx];
        vec3_t frame2_pos = source_model->frames_bone_pos[source_model->n_bones * frame2_idx + anim_bone_idx];
        quat_t frame1_rot = source_model->frames_bone_rot[source_model->n_bones * frame1_idx + anim_bone_idx];
        quat_t frame2_rot = source_model->frames_bone_rot[source_model->n_bones * frame2_idx + anim_bone_idx];
        vec3_t frame1_scale = source_model->frames_bone_scale[source_model->n_bones * frame1_idx + anim_bone_idx];
        vec3_t frame2_scale = source_model->frames_bone_scale[source_model->n_bones * frame2_idx + anim_bone_idx];

        // Get local bone transforms (relative to parent space)
        vec3_t bone_local_pos = lerp_vec3(frame1_pos, frame2_pos, lerpfrac);
        quat_t bone_local_rot = slerp_quat(frame1_rot, frame2_rot, lerpfrac);
        vec3_t bone_local_scale = lerp_vec3(frame1_scale, frame2_scale, lerpfrac);
        // Current pose bone-space transform (relative to parent)
        skeleton->bone_transforms[i] = translate_rotate_scale_mat3x4(bone_local_pos, bone_local_rot, bone_local_scale);

        // If we have a parent, concat parent transform to get model-space transform
        int parent_bone_idx = skeleton->model->bone_parent_idx[i];
        if(parent_bone_idx >= 0) {
            skeleton->bone_transforms[i] = matmul_mat3x4_mat3x4( skeleton->bone_transforms[parent_bone_idx], skeleton->bone_transforms[i] );
        }
        // If we don't have a parent, the bone-space transform _is_ the model-space transform

    }

    // Now that all bone transforms have been computed for the current pose, multiply in the inverse rest pose transforms
    // These transforms will take the vertices from model-space to each bone's local-space:
    for(uint32_t i = 0; i < source_model->n_bones; i++) {
        skeleton->bone_transforms[i] = matmul_mat3x4_mat3x4(skeleton->bone_transforms[i], skeleton->model->inv_bone_rest_transforms[i]);
        // Invert-transpose the upper-left 3x3 matrix to get the transform that should be applied to vertex normals
        skeleton->bone_normal_transforms[i] = transpose_mat3x3(invert_mat3x3(get_mat3x4_mat3x3(skeleton->bone_transforms[i])));
    }
}

// 
// Applies a `skeletal_skeleton_t` object's current built pose to the model. 
// Populates the mesh's `verts` array with the final model-space vertex locations.
// 
void apply_skeleton_pose(skeletal_skeleton_t *skeleton, skeletal_model_t *model) {

    if(skeleton->model != model) {
        // FIXME - Is this a valid condition?
        return;
    }


    // Apply skeleton pose to all vertices
    for(uint32_t i = 0; i < model->n_verts; i++) {
        vec3_t vert_rest_pos = model->vert_rest_positions[i];
        vec3_t vert_rest_nor = model->vert_rest_normals[i];
        // Accumulate final vertex position here to calculate weighted sum
        vec3_t vert_pos = { 0.0f, 0.0f, 0.0f};
        vec3_t vert_nor = { 0.0f, 0.0f, 0.0f};
        // Accumulate weighted sum total here
        float total_weight = 0.0f;

        // Loop through the vert's 4 bone indices
        for(int j = 0; j < 4; j++) {
            int vert_bone_idx = model->vert_bone_idxs[4*i + j];
            float vert_bone_weight = model->vert_bone_weights[4*i + j];
            // TODO - Measure impact of this speed? It skips over half othe vert-bone mapping matrix
            if(vert_bone_idx >= 0 && vert_bone_weight > 0.01) {
                // float vert_bone_weight = model->vert_bone_weights[4*i + j];
                vec3_t vert_bone_pos = mul_mat3x4_vec3(skeleton->bone_transforms[vert_bone_idx], vert_rest_pos);
                vec3_t vert_bone_nor = mul_mat3x3_vec3(skeleton->bone_normal_transforms[vert_bone_idx], vert_rest_nor);
                vert_pos = add_vec3( vert_pos, mul_float_vec3( vert_bone_weight, vert_bone_pos));
                vert_nor = add_vec3( vert_nor, mul_float_vec3( vert_bone_weight, vert_bone_nor));
                total_weight += vert_bone_weight;
            }
        }

        model->verts[i].x = vert_pos.x / total_weight;
        model->verts[i].y = vert_pos.y / total_weight;
        model->verts[i].z = vert_pos.z / total_weight;
        model->verts[i].nor_x = vert_nor.x / total_weight;
        model->verts[i].nor_y = vert_nor.y / total_weight;
        model->verts[i].nor_z = vert_nor.z / total_weight;
        // TODO - Write vertex tangents?
    }


    // TODO - I should error out if the skeleton's model is not `model`
    // TODO   Should I just get rid of 


    // TODO - Should a model be stateful? Or should it be a shared reused asset across all instances?
    // Depends, in dquake, where are the interpolated vertices stored? Are they stored in the model?
    // If the vertices are not stored in the model, I'd need another struct to hold them... Skip for now.


    // TODO - A Skeleton may have been built on top of a different model. 
    // TODO   If built on same model, use bone indices directly
    // TODO   If built on different model, need to somehow match up 
    // TODO   this model's bones to the skeleton model's bones by bone name
    // TODO - Will likely need to create some sort of lookup table for 
    // TODO   translating bone indices from one model to the other.
    // TODO - The core idea is that shared bones are used, all others ignored.


}




//
// Call sequence for internal animations looks like this:
//
// skeletal_model_t *zombie_model = load_iqm("something.iqm");
// skeletal_skeleton_t *zombie_skeleton = create_skeleton(zombie_model); // Skeleton is built from the model struct
// build_skeleton( zombie_skeleton, zombie_model, framegroup_idx, frametime);
// process_anim_events(zombie_anim_walk1, framegroup_idx, prev_frametime, cur_frametime, event_callback);
// apply_skeleton_pose(zombie_skeleton, zombie_model);

//
// Call sequence for external animations looks like this:
//
// skeletal_model_t *zombie_model = load_iqm("something.iqm");
// skeletal_model_t *zombie_anim_walk1 = load_iqm("something_else.iqm");
// skeletal_skeleton_t *zombie_skeleton = create_skeleton(zombie_model); // Skeleton is built from the model struct
// build_skeleton( zombie_skeleton, zombie_anim_walk1, framegroup_idx, frametime);
// process_anim_events(zombie_anim_walk1, framegroup_idx, prev_frametime, cur_frametime, event_callback);
// apply_skeleton_pose(zombie_skeleton, zombie_model);




// apply_skeleton_pose() // Populates posed_vertices with transforms from bones...



//
// Parses an IQM Vertex array and converts all values to floats
//
void iqm_parse_float_array(const uint8_t *iqm_data, const iqm_vert_array_t *vert_array, float *out, size_t n_elements, size_t element_len, float *default_value) {
    iqm_dtype dtype = (iqm_dtype) vert_array->format;
    size_t iqm_values_to_read = (size_t) vert_array->size;
    if(vert_array->ofs == 0) {
        iqm_values_to_read = 0;
        dtype = iqm_dtype::IQM_DTYPE_FLOAT;
    }
    const uint8_t *iqm_array_data = iqm_data + vert_array->ofs;


    // Special cases:
    if(dtype == iqm_dtype::IQM_DTYPE_FLOAT && element_len == iqm_values_to_read) {
        memcpy(out, (const float*) iqm_array_data, sizeof(float) * element_len * n_elements);
        return;
    }
    if(dtype == iqm_dtype::IQM_DTYPE_HALF) {
        iqm_values_to_read = 0;
    }

    // For all other dtype cases, parse each value from IQM:
    for(size_t i = 0; i < n_elements; i++) {
        // Read the first `iqm_values_to_read` values for vector `i`
        for(size_t j = 0; j < element_len && j < iqm_values_to_read; j++) {
            switch(dtype) {
                default:
                    iqm_values_to_read = 0;
                    break;
                case iqm_dtype::IQM_DTYPE_BYTE:
                    out[i * element_len + j] = ((const int8_t*)iqm_array_data)[i * iqm_values_to_read + j] * (1.0f/127);
                    break;
                case iqm_dtype::IQM_DTYPE_UBYTE:
                    out[i * element_len + j] = ((const uint8_t*)iqm_array_data)[i * iqm_values_to_read + j] * (1.0f/255);
                    break;
                case iqm_dtype::IQM_DTYPE_SHORT:
                    out[i * element_len + j] = ((const int16_t*)iqm_array_data)[i * iqm_values_to_read + j] * (1.0f/32767);
                    break;
                case iqm_dtype::IQM_DTYPE_USHORT:
                    out[i * element_len + j] = ((const uint16_t*)iqm_array_data)[i * iqm_values_to_read + j] * (1.0f/65535);
                    break;
                case iqm_dtype::IQM_DTYPE_INT:
                    out[i * element_len + j] = ((const int32_t*)iqm_array_data)[i * iqm_values_to_read + j]  / ((float)0x7fffffff);
                    break;
                case iqm_dtype::IQM_DTYPE_UINT:
                    out[i * element_len + j] = ((const uint32_t*)iqm_array_data)[i * iqm_values_to_read + j] / ((float)0xffffffffu);
                    break;
                case iqm_dtype::IQM_DTYPE_FLOAT:
                    out[i * element_len + j] = ((const float*)iqm_array_data)[i * iqm_values_to_read + j];
                    break;
                case iqm_dtype::IQM_DTYPE_DOUBLE:
                    out[i * element_len + j] = ((const double*)iqm_array_data)[i * iqm_values_to_read + j];
                    break;
            }
        }
        // Pad the remaining `element_len - iqm_values_to_read` values for vector `i`
        for(size_t j = iqm_values_to_read; j < element_len; j++) {
            out[i * element_len + j] = default_value[j];
        }
    }
}


//
// Parses an IQM Vertex array and converts all values to uint8_t
//
void iqm_parse_uint8_array(const uint8_t *iqm_data, const iqm_vert_array_t *vert_array, uint8_t *out, size_t n_elements, size_t element_len, uint8_t max_value) {
    iqm_dtype dtype = (iqm_dtype) vert_array->format;
    size_t iqm_values_to_read = (size_t) vert_array->size;
    if(vert_array->ofs == 0) {
        iqm_values_to_read = 0;
        dtype = iqm_dtype::IQM_DTYPE_UBYTE;
    }
    const uint8_t *iqm_array_data = iqm_data + vert_array->ofs;


    // Special cases:
    if(dtype == iqm_dtype::IQM_DTYPE_FLOAT && element_len == iqm_values_to_read) {
        memcpy(out, (const float*) iqm_array_data, sizeof(float) * element_len * n_elements);
        return;
    }
    if(dtype == iqm_dtype::IQM_DTYPE_HALF) {
        iqm_values_to_read = 0;
    }

    // For all other dtype cases, parse each value from IQM:
    for(size_t i = 0; i < n_elements; i++) {
        // Read the first `iqm_values_to_read` values for vector `i`
        for(size_t j = 0; j < element_len && j < iqm_values_to_read; j++) {

            uint8_t in_val;
            switch(dtype) {
                case iqm_dtype::IQM_DTYPE_FLOAT:    // Skip, these values don't make sense
                case iqm_dtype::IQM_DTYPE_DOUBLE:   // Skip, these values don't make sense
                default:
                    in_val = 0;
                    iqm_values_to_read = 0;
                    break;
                case iqm_dtype::IQM_DTYPE_BYTE:     // Interpret as signed
                case iqm_dtype::IQM_DTYPE_UBYTE:
                    in_val = ((const uint8_t*)iqm_array_data)[i * iqm_values_to_read + j];
                    break;
                case iqm_dtype::IQM_DTYPE_SHORT:    // Interpret as signed
                case iqm_dtype::IQM_DTYPE_USHORT:
                    in_val = (uint8_t) ((const uint16_t*)iqm_array_data)[i * iqm_values_to_read + j];
                    break;
                case iqm_dtype::IQM_DTYPE_INT:      // Interpret as signed
                case iqm_dtype::IQM_DTYPE_UINT:
                    in_val = (uint8_t) ((const uint32_t*)iqm_array_data)[i * iqm_values_to_read + j];
                    break;
            }

            if(in_val >= max_value) {
                // TODO - Mark invalid, return that array had invalid values
                in_val = 0;
            }
            out[i * element_len + j] = in_val;
        }
        // Pad the remaining `element_len - iqm_values_to_read` values for vector `i`
        for(size_t j = iqm_values_to_read; j < element_len; j++) {
            out[i * element_len + j] = 0;
        }
    }
}

static const void *iqm_find_extension(const uint8_t *iqm_data, size_t iqm_data_size, const char *extension_name, size_t *extension_size) {
    const iqm_header_t *iqm_header = (const iqm_header_t*) iqm_data;
    const iqm_extension_t *iqm_extension;

    iqm_extension = (const iqm_extension_t *) (iqm_data + iqm_header->ofs_extensions);

    for(uint16_t i = 0; i < iqm_header->n_extensions; i++) {
        // If past end of file, stop.
        if((const uint8_t *)iqm_extension > (iqm_data + iqm_data_size)) {
            break;
        }
        // If extension name is invalid, stop.
        if(iqm_extension->name > iqm_header->n_text) {
            break;
        }
        // If extension data pointer is past end of file, stop.
        if(iqm_extension->ofs_data + iqm_extension->n_data > iqm_data_size) {
            break;
        }
        // If name matches what we're looking for, return this extension
        const char *cur_extension_name = (const char*) ((iqm_data + iqm_header->ofs_text) + iqm_extension->name);
        if(strcmp(cur_extension_name, extension_name) == 0) {
            *extension_size = iqm_extension->n_data;
            return iqm_data + iqm_extension->ofs_data;
        }
        
        // Advance to next entry in linked list
        iqm_extension = (const iqm_extension_t *) (iqm_data + iqm_extension->ofs_extensions);
    }

    *extension_size = 0;
    return nullptr;
}


skeletal_model_t *load_iqm_file(const char*file_path) {
    FILE *f = fopen(file_path, "rb");
    fseek(f, 0, SEEK_END);
    size_t file_len = ftell(f);
    rewind(f);
    uint8_t *iqm_data = (uint8_t*) malloc(sizeof(uint8_t) * file_len);
    fread(iqm_data, file_len, 1, f);
    fclose(f);

    const iqm_header_t *iqm_header = (const iqm_header_t*) iqm_data;
    // TODO - Let's get quick-n-dirty to load up the vertices

    if(memcmp(iqm_header->magic, IQM_MAGIC, sizeof(iqm_header->magic))) {
        free(iqm_data);
        return nullptr; 
    }

    if(iqm_header->version != IQM_VERSION_2) {
        // TODO - also allow version1?
        free(iqm_data);
        return nullptr;
    }

    if(iqm_header->filesize != file_len) {
        free(iqm_data);
        return nullptr;
    }


    const iqm_vert_array_t *iqm_verts_pos = nullptr;
    const iqm_vert_array_t *iqm_verts_uv = nullptr;
    const iqm_vert_array_t *iqm_verts_nor = nullptr;
    const iqm_vert_array_t *iqm_verts_tan = nullptr;
    const iqm_vert_array_t *iqm_verts_color = nullptr;
    const iqm_vert_array_t *iqm_verts_bone_idxs = nullptr;
    const iqm_vert_array_t *iqm_verts_bone_weights = nullptr;


    const iqm_vert_array_t *vert_arrays = (const iqm_vert_array_t*)(iqm_data + iqm_header->ofs_vert_arrays);
    for(unsigned int i = 0; i < iqm_header->n_vert_arrays; i++) {
        if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_POS) {
            iqm_verts_pos = &vert_arrays[i];
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_UV) {
            iqm_verts_uv = &vert_arrays[i];
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_NOR) {
            iqm_verts_nor = &vert_arrays[i];
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_TAN) {
            // Only use tangents if float and if each tangent is a 4D vector:
            if((iqm_dtype) vert_arrays[i].format == iqm_dtype::IQM_DTYPE_FLOAT && vert_arrays[i].size == 4) {
                iqm_verts_tan = &vert_arrays[i];
            }
            else {
                log_printf("Warning: IQM vertex normals array (idx: %d, type: %d, fmt: %d, size: %d) is not 4D float array.\n", i, vert_arrays[i].type, vert_arrays[i].format, vert_arrays[i].size);
            }
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_COLOR) {
            iqm_verts_color = &vert_arrays[i];
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_BONE_IDXS) {
            iqm_verts_bone_idxs = &vert_arrays[i];
        }
        else if((iqm_vert_array_type) vert_arrays[i].type == iqm_vert_array_type::IQM_VERT_BONE_WEIGHTS) {
            iqm_verts_bone_weights = &vert_arrays[i];
        }
        else {
            log_printf("Warning: Unrecognized IQM vertex array type (idx: %d, type: %d, fmt: %d, size: %d)\n", i, vert_arrays[i].type, vert_arrays[i].format, vert_arrays[i].size);
        }
    }




    skeletal_model_t *skel_model = (skeletal_model_t*) malloc(sizeof(skeletal_model_t));
    skel_model->n_verts = iqm_header->n_verts;
    skel_model->verts = (vertex_t*) malloc(sizeof(vertex_t) * skel_model->n_verts);
    skel_model->vert_rest_positions = (vec3_t*) malloc(sizeof(vec3_t) * skel_model->n_verts);
    skel_model->vert_rest_normals = (vec3_t*) malloc(sizeof(vec3_t) * skel_model->n_verts);
    skel_model->n_meshes = iqm_header->n_meshes;
    skel_model->meshes = (skeletal_mesh_t*) malloc(sizeof(skeletal_mesh_t) * skel_model->n_meshes);

    vec2_t *verts_uv = (vec2_t*) malloc(sizeof(vec2_t) * skel_model->n_verts);

    // ------------------------------------------------------------------------
    // Convert verts_pos / verts_uv datatypes to floats 
    // ------------------------------------------------------------------------
    vec3_t default_vert = {0,0,0};
    vec2_t default_uv = {0,0};
    vec3_t default_nor = {0,0,1.0f};

    iqm_parse_float_array(iqm_data, iqm_verts_pos, (float*) skel_model->vert_rest_positions, skel_model->n_verts, 3, (float*) &default_vert);
    iqm_parse_float_array(iqm_data, iqm_verts_uv, (float*) verts_uv, skel_model->n_verts, 2, (float*) &default_uv);
    iqm_parse_float_array(iqm_data, iqm_verts_nor, (float*) skel_model->vert_rest_normals, skel_model->n_verts, 3, (float*) &default_nor);


    skel_model->vert_bone_weights = (float*) malloc(sizeof(float) * 4 * skel_model->n_verts);
    skel_model->vert_bone_idxs = (uint8_t*) malloc(sizeof(uint8_t) * 4 * skel_model->n_verts);
    float default_bone_weights[] = {0.0f, 0.0f, 0.0f, 0.0f};
    iqm_parse_float_array(iqm_data, iqm_verts_bone_weights, skel_model->vert_bone_weights,  skel_model->n_verts, 4, (float*) &default_bone_weights);
    iqm_parse_uint8_array(iqm_data, iqm_verts_bone_idxs,    skel_model->vert_bone_idxs,     skel_model->n_verts, 4, (uint8_t) std::min( (int) iqm_header->n_joints, (int) IQM_MAX_BONES));



    // Populate verts array:
    for(uint32_t i = 0; i < skel_model->n_verts; i++) {
        // NOTE: Initialize the vertex positions to the rest position.
        // NOTE: The vertex xyz coords will be updated whenever we apply a new skeletal pose to this model.
        skel_model->verts[i].x = skel_model->vert_rest_positions[i].x; 
        skel_model->verts[i].y = skel_model->vert_rest_positions[i].y;
        skel_model->verts[i].z = skel_model->vert_rest_positions[i].z;
        skel_model->verts[i].u = verts_uv[i].x;
        skel_model->verts[i].v = verts_uv[i].y;
        // skel_model->verts[i].color = 0xffffffff;
        skel_model->verts[i].color = 0x00000000;
        skel_model->verts[i].nor_x = skel_model->vert_rest_normals[i].x;
        skel_model->verts[i].nor_y = skel_model->vert_rest_normals[i].y;
        skel_model->verts[i].nor_z = skel_model->vert_rest_normals[i].z;
        // TODO - Parse other potentially optional vertex attribute arrays

    free(verts_uv);
    // ------------------------------------------------------------------------



    const iqm_mesh_t *iqm_meshes = (const iqm_mesh_t*)(iqm_data + iqm_header->ofs_meshes);

    for(uint32_t i = 0; i < iqm_header->n_meshes; i++) {
        const char *material_name = (const char*) ((iqm_data + iqm_header->ofs_text) + iqm_meshes[i].material);
        // log_printf("Mesh[%d]: \"%s\"\n", i, material_name);

        uint32_t first_vert = iqm_meshes[i].first_vert_idx;
        uint32_t first_tri = iqm_meshes[i].first_tri;
        skel_model->meshes[i].first_vert = first_vert;
        skel_model->meshes[i].n_verts = iqm_meshes[i].n_verts;
        skel_model->meshes[i].n_tris = iqm_meshes[i].n_tris;
        skel_model->meshes[i].n_tri_verts = skel_model->meshes[i].n_tris * 3;
        skel_model->meshes[i].tri_verts = (uint16_t*) malloc(sizeof(uint16_t) * skel_model->meshes[i].n_tri_verts);


        for(uint32_t j = 0; j < skel_model->meshes[i].n_tris; j++) {
            uint16_t vert_a = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[0] - first_vert;
            uint16_t vert_b = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[1] - first_vert;
            uint16_t vert_c = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[2] - first_vert;
            skel_model->meshes[i].tri_verts[j*3 + 0] = vert_a;
            skel_model->meshes[i].tri_verts[j*3 + 1] = vert_b;
            skel_model->meshes[i].tri_verts[j*3 + 2] = vert_c;
        }
    }

    // --------------------------------------------------
    // Parse bones
    // --------------------------------------------------
    log_printf("Parsing joints...\n");
    skel_model->n_bones = iqm_header->n_joints ? iqm_header->n_joints : iqm_header->n_poses;
    skel_model->bone_name = (char**) malloc(sizeof(char*) * skel_model->n_bones);
    skel_model->bone_parent_idx = (int16_t*) malloc(sizeof(int16_t) * skel_model->n_bones);
    skel_model->bone_rest_pos = (vec3_t*) malloc(sizeof(vec3_t) * skel_model->n_bones);
    skel_model->bone_rest_rot = (quat_t*) malloc(sizeof(quat_t) * skel_model->n_bones);
    skel_model->bone_rest_scale = (vec3_t*) malloc(sizeof(vec3_t) * skel_model->n_bones);

    const iqm_joint_quaternion_t *iqm_joints = (const iqm_joint_quaternion_t*) (iqm_data + iqm_header->ofs_joints);
    for(uint32_t i = 0; i < iqm_header->n_joints; i++) {
        const char *joint_name = (const char*) ((iqm_data + iqm_header->ofs_text) + iqm_joints[i].name);
        skel_model->bone_name[i] = (char*) malloc(sizeof(char) * (strlen(joint_name) + 1));
        strcpy(skel_model->bone_name[i], joint_name);
        skel_model->bone_parent_idx[i] = iqm_joints[i].parent_joint_idx;
        skel_model->bone_rest_pos[i].x = iqm_joints[i].translate[0];
        skel_model->bone_rest_pos[i].y = iqm_joints[i].translate[1];
        skel_model->bone_rest_pos[i].z = iqm_joints[i].translate[2];
        skel_model->bone_rest_rot[i].x = iqm_joints[i].rotate[0];
        skel_model->bone_rest_rot[i].y = iqm_joints[i].rotate[1];
        skel_model->bone_rest_rot[i].z = iqm_joints[i].rotate[2];
        skel_model->bone_rest_rot[i].w = iqm_joints[i].rotate[3];
        skel_model->bone_rest_scale[i].x = iqm_joints[i].scale[0];
        skel_model->bone_rest_scale[i].y = iqm_joints[i].scale[1];
        skel_model->bone_rest_scale[i].z = iqm_joints[i].scale[2];
    }

    // By default, IQM exporter will have bones sorted such that a bone is 
    // at a lower index in the bone array than any of its children.
    // We'll rely on this to make computing bone transforms simpler.
    // However, the IQM exporter allows arbitrary bone orderings to be specified
    // Verify that the above bone-ordering assumption still holds.
    
    for(uint32_t i = 0; i < skel_model->n_bones; i++) {
        // i-th bone's parent index must be less than i
        if((int) i <= skel_model->bone_parent_idx[i]) {
            log_printf("Error: IQM file bones are sorted incorrectly. Bone %d is located before its parent bone %d.\n", i, skel_model->bone_parent_idx[i]);
            // TODO - Deallocate all allocated memory
            return nullptr;
        }
    }


    // Build the transform that takes us from model-space to each bone's bone-space for the rest pose.
    // This is static, so compute once and cache
    // First build the bone-space --> model-space transform for each bone (We will invert it later)
    skel_model->bone_rest_transforms = (mat3x4_t*) malloc(sizeof(mat3x4_t) * skel_model->n_bones);
    skel_model->inv_bone_rest_transforms = (mat3x4_t*) malloc(sizeof(mat3x4_t) * skel_model->n_bones);
    for(uint32_t i = 0; i < skel_model->n_bones; i++) {
        // Rest bone-space transform (relative to parent)
        skel_model->bone_rest_transforms[i] = translate_rotate_scale_mat3x4(
            skel_model->bone_rest_pos[i],
            skel_model->bone_rest_rot[i],
            skel_model->bone_rest_scale[i]
        );

        // If we have a parent, concat parent transform to get model-space transform
        int parent_bone_idx = skel_model->bone_parent_idx[i];
        if(parent_bone_idx >= 0) {
            skel_model->bone_rest_transforms[i] = matmul_mat3x4_mat3x4( skel_model->bone_rest_transforms[parent_bone_idx], skel_model->bone_rest_transforms[i]);
        }
        // If we don't have a parent, the bone-space transform _is_ the model-space transform
    }
    // Next, invert the transforms to get the model-space --> bone-space transform
    for(uint32_t i = 0; i < skel_model->n_bones; i++) {
        skel_model->inv_bone_rest_transforms[i] = invert_mat3x4(skel_model->bone_rest_transforms[i]);
    }
    // --------------------------------------------------


    // ========================================================================
    // TEMP - Splitting each mesh into 8-bone meshes
    // ========================================================================
    submesh_skeletal_model(skel_model);
    // ========================================================================




    // --------------------------------------------------
    // Parse all frames (poses)
    // --------------------------------------------------
    skel_model->n_frames = iqm_header->n_frames;
    log_printf("\tBones: %d\n",skel_model->n_bones);
    log_printf("\tFrames: %d\n",iqm_header->n_frames);
    log_printf("\tPoses: %d\n",iqm_header->n_poses);
    skel_model->frames_bone_pos = (vec3_t *) malloc(sizeof(vec3_t) * skel_model->n_bones * skel_model->n_frames);
    skel_model->frames_bone_rot = (quat_t *) malloc(sizeof(quat_t) * skel_model->n_bones * skel_model->n_frames);
    skel_model->frames_bone_scale = (vec3_t *) malloc(sizeof(vec3_t) * skel_model->n_bones * skel_model->n_frames);

    const uint16_t *frames_data = (const uint16_t*)(iqm_data + iqm_header->ofs_frames);
    int frames_data_ofs = 0;
    const iqm_pose_quaternion_t *iqm_poses = (const iqm_pose_quaternion_t*) (iqm_data + iqm_header->ofs_poses);

    // Iterate over actual frames in IQM file:
    for(uint32_t i = 0; i < iqm_header->n_frames; i++) {
        // Iterate over pose (a pose is a bone orientation, one pose per bone)
        for(uint32_t j = 0; j < iqm_header->n_poses; j++) {
            // Read data for all 10 channels
            float pose_data[10] = {0};
            for(uint32_t k = 0; k < 10; k++) {
                pose_data[k] = iqm_poses[j].channel_ofs[k];
                if(iqm_poses[j].mask & (1 << k)) {
                    pose_data[k] += frames_data[frames_data_ofs++] * iqm_poses[j].channel_scale[k];
                }
            }
            int frame_bone_idx = i * skel_model->n_bones + j;
            skel_model->frames_bone_pos[frame_bone_idx].x = pose_data[0];
            skel_model->frames_bone_pos[frame_bone_idx].y = pose_data[1];
            skel_model->frames_bone_pos[frame_bone_idx].z = pose_data[2];
            skel_model->frames_bone_rot[frame_bone_idx].x = pose_data[3];
            skel_model->frames_bone_rot[frame_bone_idx].y = pose_data[4];
            skel_model->frames_bone_rot[frame_bone_idx].z = pose_data[5];
            skel_model->frames_bone_rot[frame_bone_idx].w = pose_data[6];
            skel_model->frames_bone_scale[frame_bone_idx].x = pose_data[7];
            skel_model->frames_bone_scale[frame_bone_idx].y = pose_data[8];
            skel_model->frames_bone_scale[frame_bone_idx].z = pose_data[9];

            // TODO - Compute matrix from these values?
        }
    }
    // --------------------------------------------------

    // --------------------------------------------------
    // Parse animations (framegroups)
    // --------------------------------------------------
    skel_model->n_framegroups = iqm_header->n_anims;
    skel_model->framegroup_name = (char**) malloc(sizeof(char*) * skel_model->n_framegroups);
    skel_model->framegroup_start_frame = (uint32_t*) malloc(sizeof(uint32_t) * skel_model->n_framegroups);
    skel_model->framegroup_n_frames = (uint32_t*) malloc(sizeof(uint32_t) * skel_model->n_framegroups);
    skel_model->framegroup_fps = (float*) malloc(sizeof(float) * skel_model->n_framegroups);
    skel_model->framegroup_loop = (bool*) malloc(sizeof(bool) * skel_model->n_framegroups);

    if(iqm_header->n_anims > 0) {
        const iqm_anim_t *iqm_framegroups = (const iqm_anim_t*)(iqm_data + iqm_header->ofs_anims);
        for(uint32_t i = 0; i < iqm_header->n_anims; i++) {
            const char* framegroup_name = (const char*) (iqm_data + iqm_header->ofs_text + iqm_framegroups[i].name);
            skel_model->framegroup_name[i] = (char*) malloc(sizeof(char) * (strlen(framegroup_name) + 1));
            strcpy(skel_model->framegroup_name[i], framegroup_name);
            skel_model->framegroup_start_frame[i] = iqm_framegroups[i].first_frame;
            skel_model->framegroup_n_frames[i] = iqm_framegroups[i].n_frames;
            skel_model->framegroup_fps[i] = iqm_framegroups[i].framerate;
            skel_model->framegroup_loop[i] = iqm_framegroups[i].flags & (uint32_t) iqm_anim_flag::IQM_ANIM_FLAG_LOOP;
        }
    }
    log_printf("\tParsed %d framegroups.\n", skel_model->n_framegroups);
    // --------------------------------------------------


    // --------------------------------------------------
    // Parse IQM per-frame mins/maxs to compute overall mins/maxs
    // --------------------------------------------------
    const iqm_bounds_t *iqm_frame_bounds = (const iqm_bounds_t *)(iqm_data + iqm_header->ofs_bounds);
    if(iqm_header->ofs_bounds != 0) {
        // Compute overall model mins / maxes by finding the most extreme points for all frames:
        for(uint16_t i = 0; i < iqm_header->n_frames; i++) {
            skel_model->mins.x = fmin(skel_model->mins.x, iqm_frame_bounds[i].mins[0]);
            skel_model->mins.y = fmin(skel_model->mins.y, iqm_frame_bounds[i].mins[1]);
            skel_model->mins.z = fmin(skel_model->mins.z, iqm_frame_bounds[i].mins[2]);
            skel_model->maxs.x = fmax(skel_model->maxs.x, iqm_frame_bounds[i].maxs[0]);
            skel_model->maxs.y = fmax(skel_model->maxs.y, iqm_frame_bounds[i].maxs[1]);
            skel_model->maxs.z = fmax(skel_model->maxs.z, iqm_frame_bounds[i].maxs[2]);
        }
    }
    // --------------------------------------------------

    // --------------------------------------------------
    // Parse FTE_MESH IQM extension
    // --------------------------------------------------
    size_t iqm_fte_ext_mesh_size;
    const iqm_ext_fte_mesh_t *iqm_fte_ext_mesh = (const iqm_ext_fte_mesh_t *) iqm_find_extension(iqm_data, file_len, "FTE_MESH", &iqm_fte_ext_mesh_size);

    // TODO - Do something with the above extensions data
    if(iqm_fte_ext_mesh != nullptr) {
        for(uint16_t i = 0; i < iqm_header->n_meshes; i++) {
            log_printf("IQM FTE Extension \"FTE_MESH \" mesh %d\n", i);
            log_printf("\tcontents %d\n", iqm_fte_ext_mesh[i].contents);
            log_printf("\tsurfaceflags %d\n", iqm_fte_ext_mesh[i].surfaceflags);
            log_printf("\tsurfaceid %d\n", iqm_fte_ext_mesh[i].surfaceid);
            log_printf("\tgeomset %d\n", iqm_fte_ext_mesh[i].geomset);
            log_printf("\tgeomid %d\n", iqm_fte_ext_mesh[i].geomid);
            log_printf("\tmin_dist %d\n", iqm_fte_ext_mesh[i].min_dist); // LOD min distance
            log_printf("\tmax_dist %d\n", iqm_fte_ext_mesh[i].max_dist); // LOD max distance
        }
    }
    // --------------------------------------------------


    // --------------------------------------------------
    // Parse FTE_EVENT IQM extension
    // --------------------------------------------------
    size_t iqm_fte_ext_event_size;
    const iqm_ext_fte_event_t *iqm_fte_ext_events = (const iqm_ext_fte_event_t*) iqm_find_extension(iqm_data, file_len, "FTE_EVENT", &iqm_fte_ext_event_size);
    uint16_t iqm_fte_ext_event_n_events = iqm_fte_ext_event_size / sizeof(iqm_ext_fte_event_t);
    log_printf("FTE_EVENTS parsed size %d\n", iqm_fte_ext_event_size);
    log_printf("num FTE_EVENTS %d\n", iqm_fte_ext_event_n_events);
    log_printf("FTE event struct %d\n", sizeof(iqm_ext_fte_event_t));

    if(iqm_fte_ext_events != nullptr) {
        // Count the number of events for each framegroup:
        skel_model->framegroup_n_events = (uint16_t*) malloc(sizeof(uint16_t) * skel_model->n_framegroups);
        for(uint16_t i = 0; i < skel_model->n_framegroups; i++) {
            skel_model->framegroup_n_events[i] = 0;
        }
        for(uint16_t i = 0; i < iqm_fte_ext_event_n_events; i++) {
            int framegroup_idx = iqm_fte_ext_events[i].anim;
            skel_model->framegroup_n_events[framegroup_idx]++;
        }

        // Allocate memory for all framegroup arrays
        skel_model->framegroup_event_time = (float**) malloc(sizeof(float*) * skel_model->n_framegroups);
        skel_model->framegroup_event_data_str = (char***) malloc(sizeof(char**) * skel_model->n_framegroups);
        skel_model->framegroup_event_code = (uint32_t**) malloc(sizeof(uint32_t*) * skel_model->n_framegroups);

        // Allocate enough memory in each framegroup array to hold events in a list:
        for(uint16_t i = 0; i < skel_model->n_framegroups; i++) {
            if(skel_model->framegroup_n_events[i] == 0) {
                skel_model->framegroup_event_time[i] = nullptr;
                skel_model->framegroup_event_data_str[i] = nullptr;
                skel_model->framegroup_event_code[i] = nullptr;
            }
            else {
                skel_model->framegroup_event_time[i] = (float*) malloc(sizeof(float) * skel_model->framegroup_n_events[i]);
                skel_model->framegroup_event_data_str[i] = (char**) malloc(sizeof(char*) * skel_model->framegroup_n_events[i]);
                skel_model->framegroup_event_code[i] = (uint32_t*) malloc(sizeof(uint32_t) * skel_model->framegroup_n_events[i]);
                // Set each char array pointer to nullptr, we'll use this to identify empty indices
                for(int j = 0; j < skel_model->framegroup_n_events[i]; j++) {
                    skel_model->framegroup_event_data_str[i][j] = nullptr;
                }
            }
        }

        for(uint16_t i = 0; i < iqm_fte_ext_event_n_events; i++) {
            const char *iqm_event_data_str = (const char*) (iqm_data + iqm_header->ofs_text + iqm_fte_ext_events[i].event_data_str);

            int event_framegroup_idx = iqm_fte_ext_events[i].anim;
            float event_time = iqm_fte_ext_events[i].timestamp;
            uint32_t event_code = iqm_fte_ext_events[i].event_code;
            // Allocate data for this string and copy the char array
            char *event_data_str = (char*) malloc(sizeof(char) * (strlen(iqm_event_data_str) + 1));
            strcpy(event_data_str, iqm_event_data_str);

            // If invalid framegroup idx, skip the event
            if(event_framegroup_idx < 0 || event_framegroup_idx >= skel_model->n_framegroups) {
                log_printf("WARNING: Unable to load IQM event. Framegroup idx %d invalid for a model with %d framegroups.\n", event_framegroup_idx, skel_model->n_framegroups);
                continue;
            }

            // Find the correct index to insert the event into in this framegroup's event list
            int event_idx = 0;
            for(; event_idx < skel_model->framegroup_n_events[event_framegroup_idx]; event_idx++) {
                // If this index is empty, insert here
                if(skel_model->framegroup_event_data_str[event_framegroup_idx][event_idx] == nullptr) {
                    break;
                }
                // If the event at this index has an event time > the current event, insert here
                else if(skel_model->framegroup_event_time[event_framegroup_idx][event_idx] > event_time) {
                    break;
                }
            }
            // log_printf("Event %d inserting at framegroup %d index %d\n", i, event_framegroup_idx, event_idx);

            // If event_idx exceeds the length of the array, warn and skip (This should never happen)
            if(event_idx >= skel_model->framegroup_n_events[event_framegroup_idx]) {
                log_printf("WARNING: Unable to load IQM event for time %f for framegroup %d.\n", i, event_framegroup_idx);
                continue;
            }

            // Shift up the events in the array.
            // Starting from the end of the array, move up all events after the insertion `event_idx`
            for(int j = skel_model->framegroup_n_events[event_framegroup_idx] - 2; j >= event_idx; j--) {
                if(skel_model->framegroup_event_data_str[event_framegroup_idx][j] == nullptr) {
                    continue;
                }
                // log_printf("\tframegroup %d moving event at idx %d to idx %d\n", event_framegroup_idx, j, j+1);
                skel_model->framegroup_event_time[event_framegroup_idx][j+1] = skel_model->framegroup_event_time[event_framegroup_idx][j];
                skel_model->framegroup_event_data_str[event_framegroup_idx][j+1] = skel_model->framegroup_event_data_str[event_framegroup_idx][j];
                skel_model->framegroup_event_code[event_framegroup_idx][j+1] = skel_model->framegroup_event_code[event_framegroup_idx][j];
            }

            // Insert the new event at the chosen index:
            skel_model->framegroup_event_time[event_framegroup_idx][event_idx] = event_time;
            skel_model->framegroup_event_data_str[event_framegroup_idx][event_idx] = event_data_str;
            skel_model->framegroup_event_code[event_framegroup_idx][event_idx] = event_code;
        }
    }
    // --------------------------------------------------

    // --------------------------------------------------
    // Parse FTE_SKIN IQM extension
    // --------------------------------------------------
    // size_t iqm_fte_ext_skin_size;
    // const uint32_t *iqm_fte_ext_skin_n_skins = (const uint32_t*) iqm_find_extension(iqm_data, file_len, "FTE_SKINS", &iqm_fte_ext_skin_size);

    // // const iqm_ext_fte_skin_t *iqm_fte_ext_skins
    // if(iqm_fte_ext_skin_n_skins != nullptr) {

    //     if(iqm_fte_ext_skin_size >= (sizeof(uint32_t) * 2 * iqm_header->n_meshes)) {
    //         const uint32_t n_skin_skinframes = iqm_fte_ext_skin_n_skins[0];
    //         const uint32_t n_skin_meshskins = iqm_fte_ext_skin_n_skins[1];
    //         size_t expected_skin_size = sizeof(iqm_ext_fte_skin_t) + sizeof(uint32_t) * iqm_header->n_meshes + sizeof(iqm_ext_fte_skin_t) * n_skin_skinframes + sizeof(iqm_ext_fte_skin_meshskin_t) * n_skin_meshskins;
    //         if(iqm_fte_ext_skin_size == expected_skin_size) {
    //             const iqm_ext_fte_skin_skinframe_t *skinframes = (const iqm_ext_fte_skin_skinframe_t *) (iqm_fte_ext_skin_n_skins + 2 + iqm_header->n_meshes);
    //             const iqm_ext_fte_skin_meshskin_t *meshskins = (const iqm_ext_fte_skin_meshskin_t *) (skinframes + n_skin_skinframes);

    //             // TODO - Print these? Not sure if we'll even want them... or if we should support them?
    //         }
    //     }
    // }
    // --------------------------------------------------


    free(iqm_data);
    return skel_model;
}

