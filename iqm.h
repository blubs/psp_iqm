#include "logging.h"
#include "math.h"


#define IQM_MAGIC "INTERQUAKEMODEL"
#define IQM_VERSION_1 1
#define IQM_VERSION_2 2


typedef struct vec3_s {
    float pos[3];
} vec3_t;


typedef struct vec2_s {
    float pos[2];
} vec2_t;



// TODO - Define needed structs
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



class Skeletal_Mesh {
public:
    // Number of vertices in mesh
    uint32_t n_verts;
    // Number of triangles in mesh
    uint32_t n_tris;
    // Contains vertex indices
    uint16_t *tri_verts = nullptr;
    uint32_t n_tri_verts;
    uint32_t first_vert;

    // TODO - Geomset identifiers...
    // TODO - Material identifiers...
    // TODO - Other vertex attributes (bone_idxs, bone_weights, normals, tangents, colors)
    // TODO - An array with pre-allocated memory for cur_posed_vertices after a skeleton pose is applied
};



class Skeletal_Model {
public:
    uint32_t n_verts;
    vertex_t *verts = nullptr;
    uint32_t n_meshes;
    // List of submeshes
    Skeletal_Mesh *meshes;


    // TODO - Bone data (parent indices, rest pose trans/rot/scale, rest pose matrices, inverse rest pose matrices)
    // -----------------------------------
    
    // TODO - Animation data 
    // TODO         (per-framegroup per-frame per-bone trans/rot/scale)
    // TODO         (per-framegroup animation FPS)
    // TODO         (per-framegroup animation events)
    // TODO         (per-framegroup animation stop type) (loop or stop)
    // const char *name; // List of bone names
    // uint16_t parent_bone_idx; // List of bone parent indices, -1: no parent
    // // Contains rest-pose translation rotation and scale relative to parent bone
    // // float translate[3], rotate[4], scale[3];



    //
    // Creates a `Skeletal_Skeleton` object that uses this model's skeleton / animation skeleton pose information.
    //
    Skeletal_Skeleton *create_skeleton() {
        // TODO
    }


    // 
    // Processes an animation's elapsed framegroup events.
    //
    void process_anim_events(int framegroup_idx, float start_frametime, float end_frametime) {
        // TODO - Add event callback somehow to this function
    }

    // 
    // Applies a `Skeletal_Skeleton` object's current built pose to the model. 
    // Populates each meshe's `cur_posed_vertices` with final model-space vertex locations.
    // 
    void apply_skeleton_pose(const Skeletal_Skeleton *skeleton) {

        // TODO - A Skeleton may have been built on top of a different model. 
        // TODO   If built on same model, use bone indices directly
        // TODO   If built on different model, need to somehow match up 
        // TODO   this model's bones to the skeleton model's bones by bone name
        // TODO - Will likely need to create some sort of lookup table for 
        // TODO   translating bone indices from one model to the other.
        // TODO - The core idea is that shared bones are used, all others ignored.



        // TODO - Compute and cache 3x4 rest pose inverse...
        // TODO - Does this need to be a 4x4? What shape does the inverse take?
        // TODO - When do we need the parent'bone's inverse rest pose?
        // Final pose =
        //  weight_a * (bone_a * inv_rest_pose_a * vert)
        //  + weight_b * (bone_b * inv_rest_pose_b * vert)
        //  + weight_c * (bone_c * inv_rest_pose_c * vert)
        // 
    }
};



class Skeletal_Skeleton {
public:
    const Skeletal_Model *model; // Animation / skeleton data is pulled from this model

    // TODO - Structs to hold current built skeleton pose

    void build(int framegroup_idx, float frametime) {
        // TODO - Populate current pose matrices
    }
};







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


    // const int8_t *in = (const int8_t*) iqm_array_data;

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


model_t *load_iqm_file(const char*file_path) {
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
        // FIXME
        return NULL; 
    }

    if(iqm_header->version != IQM_VERSION_2) {
        // TODO - also allow version1?
        return NULL;
    }

    if(iqm_header->filesize != file_len) {
        return NULL;
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
        // unsigned int vert_array_type = vert_arrays[i]->type;
        // unsigned int vert_array_format = vert_arrays[i]->format;
        // unsigned int vert_array_size = vert_arrays[i]->size;
        // unsigned int vert_array_type = vert_arrays[i]->ofs;

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



    model_t *model = (model_t*) malloc(sizeof(model_t));
    model->n_verts = iqm_header->n_verts;
    model->verts = (vertex_t*) malloc(sizeof(vertex_t) * model->n_verts);
    model->n_meshes = iqm_header->n_meshes;
    model->meshes = (mesh_t*) malloc(sizeof(mesh_t) * model->n_meshes);

    vec3_t *verts_pos = (vec3_t*) malloc(sizeof(vec3_t) * model->n_verts);
    vec2_t *verts_uv = (vec2_t*) malloc(sizeof(vec2_t) * model->n_verts);

    // ------------------------------------------------------------------------
    // Convert verts_pos / verts_uv datatypes to floats 
    // ------------------------------------------------------------------------
    vec3_t default_vert = {0,0,0};
    vec2_t default_uv = {0,0};

    iqm_parse_float_array(iqm_data, iqm_verts_pos, (float*) verts_pos, model->n_verts, 3, (float*) &default_vert);
    iqm_parse_float_array(iqm_data, iqm_verts_uv, (float*) verts_uv, model->n_verts, 2, (float*) &default_uv);

    // Populate verts array:
    for(uint32_t i = 0; i < model->n_verts; i++) {
        model->verts[i].x = verts_pos[i].pos[0]; 
        model->verts[i].y = verts_pos[i].pos[1];
        model->verts[i].z = verts_pos[i].pos[2];
        model->verts[i].u = verts_uv[i].pos[0];
        model->verts[i].v = verts_uv[i].pos[1];
        // model->verts[i].color = 0xffffffff;
        model->verts[i].color = 0x00000000;
    }
    // ------------------------------------------------------------------------



    const iqm_mesh_t *iqm_meshes = (const iqm_mesh_t*)(iqm_data + iqm_header->ofs_meshes);

    for(uint32_t i = 0; i < iqm_header->n_meshes; i++) {
        const char *material_name = (const char*) ((iqm_data + iqm_header->ofs_text) + iqm_meshes[i].material);
        log_printf("Mesh[%d]: \"%s\"\n", i, material_name);


        uint32_t first_vert = iqm_meshes[i].first_vert_idx;
        uint32_t n_verts = iqm_meshes[i].n_verts;
        uint32_t first_tri = iqm_meshes[i].first_tri;
        uint32_t n_tris = iqm_meshes[i].n_tris;
        model->meshes[i].n_tri_verts = n_tris * 3;
        model->meshes[i].tri_verts = (uint16_t*) malloc(sizeof(uint16_t) * n_tris * 3);
        model->meshes[i].first_vert = first_vert;


        for(uint32_t j = 0; j < n_tris; j++) {
            uint16_t vert_a = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[0] - first_vert;
            uint16_t vert_b = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[1] - first_vert;
            uint16_t vert_c = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[2] - first_vert;
            // uint16_t vert_a = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[0];
            // uint16_t vert_b = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[1];
            // uint16_t vert_c = ((iqm_tri_t*)(iqm_data + iqm_header->ofs_tris))[first_tri + j].vert_idxs[2];
            model->meshes[i].tri_verts[j*3 + 0] = vert_a;
            model->meshes[i].tri_verts[j*3 + 1] = vert_b;
            model->meshes[i].tri_verts[j*3 + 2] = vert_c;
        }
        model->meshes[i].n_verts = n_verts;
        model->meshes[i].n_tris = n_tris;
    }

    // --------------------------------------------------
    // Parse bones
    // --------------------------------------------------
    log_printf("Parsing joints...\n");

    const iqm_joint_quaternion_t *iqm_joints = (const iqm_joint_quaternion_t*) (iqm_data + iqm_header->ofs_joints);
    for(uint32_t i = 0; i < iqm_header->n_joints; i++) {
        const char *joint_name = (const char*) ((iqm_data + iqm_header->ofs_text) + iqm_joints[i].name);
        log_printf("Joint[%d]: \"%s\"\n", i, joint_name);
        log_printf("\tParent bone: %d\n", iqm_joints[i].parent_joint_idx);
        log_printf("\tPos: (%f, %f, %f)\n", iqm_joints[i].translate[0], iqm_joints[i].translate[1], iqm_joints[i].translate[2]);
        log_printf("\tRot: (%f, %f, %f, %f)\n", iqm_joints[i].rotate[0], iqm_joints[i].rotate[1], iqm_joints[i].rotate[2]);
        log_printf("\tScale: (%f, %f, %f)\n", iqm_joints[i].scale[0], iqm_joints[i].scale[1], iqm_joints[i].scale[2]);
    }
    // --------------------------------------------------
    // --------------------------------------------------
    // Parse all frames (poses)
    // --------------------------------------------------
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
            float pos_x = pose_data[0];
            float pos_y = pose_data[1];
            float pos_z = pose_data[2];
            float quat_x = pose_data[3];
            float quat_y = pose_data[4];
            float quat_z = pose_data[5];
            float quat_w = pose_data[6];
            float scale_x = pose_data[7];
            float scale_y = pose_data[8];
            float scale_z = pose_data[9];

            log_printf("Frame: %d, Pose: %d \n", i, j);
            log_printf("\tPos: (%f, %f, %f)\n", pos_x, pos_y, pos_z);
            log_printf("\tRot: (%f, %f, %f, %f)\n", quat_x, quat_y, quat_z, quat_w);
            log_printf("\tScale: (%f, %f, %f)\n", scale_x, scale_y, scale_z);

            // TODO - Store these values?
            // TODO - Compute matrix from these values?
        }

    }
    // --------------------------------------------------

    // --------------------------------------------------
    // Parse animations (framegroups)
    // --------------------------------------------------
    if(iqm_header->n_anims > 0) {
        const iqm_anim_t *iqm_framegroups = (const iqm_anim_t*)(iqm_data + iqm_header->ofs_anims);
        for(uint32_t i = 0; i < iqm_header->n_anims; i++) {
            const char* framegroup_name = (const char*) (iqm_data + iqm_header->ofs_text + iqm_framegroups[i].name);
            log_printf("Framegroup: %d, \"%s\"\n", i, framegroup_name);
            log_printf("\tStart Frame: %d\n", iqm_framegroups[i].first_frame);
            log_printf("\tFrames: %d\n", iqm_framegroups[i].n_frames);
            log_printf("\tFramerate: %f\n", iqm_framegroups[i].framerate);
            log_printf("\tFlags: %d\n", iqm_framegroups[i].flags);
            log_printf("\t\tLoop?: %d\n", iqm_framegroups[i].flags & (uint32_t) iqm_anim_flag::IQM_ANIM_FLAG_LOOP);

        }
    }
    // --------------------------------------------------


    // --------------------------------------------------
    // Parse IQM per-frame mins/maxs to compute overall mins/maxs
    // --------------------------------------------------
    const iqm_bounds_t *iqm_frame_bounds = (const iqm_bounds_t *)(iqm_data + iqm_header->ofs_bounds);
    if(iqm_header->ofs_bounds != 0) {
        // TODO - Compute overall model mins / maxes by finding the most extreme points for all frames:
        for(uint16_t i = 0; i < iqm_header->n_frames; i++) {
            // vec3_t frame_mins;
            // frame_mins.pos[0] = iqm_frame_bounds[i].mins[0];
            // frame_mins.pos[1] = iqm_frame_bounds[i].mins[1];
            // frame_mins.pos[2] = iqm_frame_bounds[i].mins[2];
            // vec3_t frame_maxs;
            // frame_maxs.pos[0] = iqm_frame_bounds[i].maxs[0];
            // frame_maxs.pos[1] = iqm_frame_bounds[i].maxs[1];
            // frame_maxs.pos[2] = iqm_frame_bounds[i].maxs[2];
            log_printf("Frame [%d] bounds: (%f,%f,%f) (%f,%f,%f)\n", i, 
                iqm_frame_bounds[i].mins[0], iqm_frame_bounds[i].mins[1], iqm_frame_bounds[i].mins[2],
                iqm_frame_bounds[i].maxs[0],iqm_frame_bounds[i].maxs[1], iqm_frame_bounds[i].maxs[2]
            );
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

        // TODO - Linked list...
        for(uint16_t i = 0; i < iqm_fte_ext_event_n_events; i++) {
            log_printf("IQM FTE Extension \"FTE_EVENT \" event %d\n", i);
            log_printf("\tanim: %d\n", iqm_fte_ext_events[i].anim);
            log_printf("\ttimestamp: %f\n", iqm_fte_ext_events[i].timestamp);
            log_printf("\tevent_code: %d\n", iqm_fte_ext_events[i].event_code);
            log_printf("\tevent_data_str idx: %d\n", iqm_fte_ext_events[i].event_data_str);
            // TODO - if(iqm_fte_ext_events[i].event_data_str == 0)
            // TODO   then this event has no data.
            // TODO   Parsing it is okay tho', as it'll return an empty string.
            // TODO   Will need to decide how we should handle the no-data event case.
            const char* event_data = (const char*) (iqm_data + iqm_header->ofs_text + iqm_fte_ext_events[i].event_data_str);
            log_printf("\tevent_data_str: \"%s\"\n", event_data);
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


    // TODO - Find and parse FTE_SKINS extension 

    free(iqm_data);
    return model;


    // TODO - Free iqm_data
}

