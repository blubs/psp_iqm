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


float dot_vec3(vec3_t a, vec3_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}


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
    float dot_product = (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
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
    float quat_nor = 1.0 / sqrtf((result.x * result.x) + (result.y * result.y) + (result.z * result.z) + (result.w * result.w));
    result.x *= quat_nor;
    result.y *= quat_nor;
    result.z *= quat_nor;
    result.w *= quat_nor;
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
    uint16_t *bone_parent_idx;

    vec3_t *bone_rest_pos;
    quat_t *bone_rest_rot;
    vec3_t *bone_rest_scale;


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
    // TODO - Fields for the current interpolated skeletal bone pose matrices
} skeletal_skeleton_t;



//
// Creates a `Skeletal_Skeleton` object that uses this model's skeleton / animation skeleton pose information.
//
skeletal_skeleton_t *create_skeleton(skeletal_model_t *model) {
    skeletal_skeleton_t *skeleton = (skeletal_skeleton_t*) malloc(sizeof(skeletal_skeleton_t));
    skeleton->model = model;
    // TODO - allocate memory for other fields in the skeleton to store bone current pose matrices
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
void build_skeleton(skeletal_skeleton_t *skeleton, skeletal_model_t *source_model, int framegroup_idx, float frametime) {

    if(skeleton->model != source_model) {
        // FIXME - If this happens, need to build model pose data from model animation data
        // FIXME - Then apply model pose to skeleton pose by bone name
        return;
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


    // TODO - Loop through bones, building up model-space bone transforms
    // FIXME - Do we need to use the bone rest positions for anything? or do they just get replaced?

    for(int i = 0; i < source_model->n_bones; i++) {
        vec3_t frame1_pos = source_model->frames_bone_pos[source_model->n_bones * frame1_idx + i];
        vec3_t frame2_pos = source_model->frames_bone_pos[source_model->n_bones * frame2_idx + i];
        quat_t frame1_rot = source_model->frames_bone_rot[source_model->n_bones * frame1_idx + i];
        quat_t frame2_rot = source_model->frames_bone_rot[source_model->n_bones * frame2_idx + i];
        vec3_t frame1_scale = source_model->frames_bone_scale[source_model->n_bones * frame1_idx + i];
        vec3_t frame2_scale = source_model->frames_bone_scale[source_model->n_bones * frame2_idx + i];

        vec3_t bone_pos = lerp_vec3(frame1_pos, frame2_pos, lerpfrac);
        quat_t bone_rot = slerp_quat(frame1_rot, frame2_rot, lerpfrac);
        vec3_t bone_scale = lerp_vec3(frame1_scale, frame2_scale, lerpfrac);



        // ------------–------------–------------–------------–------------–---
        // Calculate the inverse rest pose transform for this bone:
        // TODO - Calculate this once at load and stash it
        // ------------–------------–------------–------------–------------–---
        vec3_t bone_rest_pos = source_model->bone_rest_pos[i];
        quat_t bone_rest_rot = source_model->bone_rest_rot[i];
        vec3_t bone_rest_scale = source_model->bone_rest_scale[i];
        // ------------–------------–------------–------------–------------–---




    }



}

// 
// Applies a `skeeltal_skeleton_t` object's current built pose to the model. 
// Populates the mesh's `verts` array with the final model-space vertex locations.
// 
void apply_skeleton_pose(skeletal_skeleton_t *skeleton, skeletal_model_t *model) {

    if(skeleton->model != model) {
        // FIXME - Is this a valid condition?
        return;
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


    // TODO - Transform the normals and set them as well.


    // TODO - Compute and cache 3x4 rest pose inverse...
    // TODO - Does this need to be a 4x4? What shape does the inverse take?
    // TODO - When do we need the parent'bone's inverse rest pose?
    // Final pose =
    //  weight_a * (bone_a * inv_rest_pose_a * vert)
    //  + weight_b * (bone_b * inv_rest_pose_b * vert)
    //  + weight_c * (bone_c * inv_rest_pose_c * vert)
    // 
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
    skel_model->n_meshes = iqm_header->n_meshes;
    skel_model->meshes = (skeletal_mesh_t*) malloc(sizeof(skeletal_mesh_t) * skel_model->n_meshes);

    vec2_t *verts_uv = (vec2_t*) malloc(sizeof(vec2_t) * skel_model->n_verts);

    // ------------------------------------------------------------------------
    // Convert verts_pos / verts_uv datatypes to floats 
    // ------------------------------------------------------------------------
    vec3_t default_vert = {0,0,0};
    vec2_t default_uv = {0,0};

    iqm_parse_float_array(iqm_data, iqm_verts_pos, (float*) skel_model->vert_rest_positions, skel_model->n_verts, 3, (float*) &default_vert);
    iqm_parse_float_array(iqm_data, iqm_verts_uv, (float*) verts_uv, skel_model->n_verts, 2, (float*) &default_uv);


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
        // TODO - Parse other potentially optional vertex attribute arrays
    }
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
    skel_model->n_bones = iqm_header->n_joints;
    skel_model->bone_name = (char**) malloc(sizeof(char*) * skel_model->n_bones);
    skel_model->bone_parent_idx = (uint16_t*) malloc(sizeof(uint16_t) * skel_model->n_bones);
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
    
    for(int i = 0; i < skel_model->n_bones; i++) {
        // i-th bone's parent index must be less than i
        if(i <= skel_model->bone_parent_idx[i]) {
            log_printf("Error: IQM file bones are sorted incorrectly. Bone %d is located before its parent bone %d.\n", i, skel_model->bone_parent_idx[i]);
            // TODO - Deallocate all allocated memory
            return nullptr;
        }
    }



    // --------------------------------------------------
    // --------------------------------------------------
    // Parse all frames (poses)
    // --------------------------------------------------

    skel_model->n_frames = iqm_header->n_frames;
    skel_model->frames_bone_pos = (vec3_t *) malloc(sizeof(vec3_t) * skel_model->n_bones * skel_model->n_frames);
    skel_model->frames_bone_rot = (quat_t *) malloc(sizeof(quat_t) * skel_model->n_bones * skel_model->n_frames);
    skel_model->frames_bone_scale = (vec3_t *) malloc(sizeof(vec3_t) * skel_model->n_bones * skel_model->n_frames);

    const uint16_t *frames_data = (const uint16_t*)(iqm_data + iqm_header->ofs_frames);
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
                    pose_data[k] += frames_data[i*iqm_header->n_poses + j] * iqm_poses[j].channel_scale[k];
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

