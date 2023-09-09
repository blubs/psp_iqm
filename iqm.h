#include "logging.h"


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
    unsigned int version;
    unsigned int filesize;
    unsigned int flags;
    unsigned int n_text, ofs_text;
    unsigned int n_meshes, ofs_meshes;
    unsigned int n_vert_arrays, n_verts, ofs_vert_arrays;
    unsigned int n_tris, ofs_tris, ofs_adjacency;
    unsigned int n_joints, ofs_joints;
    unsigned int n_poses, ofs_poses;
    unsigned int n_anims, ofs_anims;
    unsigned int n_frames, n_frame_channels, ofs_frames, ofs_bounds;
    unsigned int n_comments, ofs_comments;
    unsigned int n_extensions, ofs_extensions;
} iqm_header_t;

typedef struct iqm_mesh_s {
    unsigned int name;
    unsigned int material;
    unsigned int first_vert_idx, n_verts;
    unsigned int first_tri, n_tris;
} iqm_mesh_t;

enum class iqm_vert_array_type : unsigned int {
    IQM_VERT_POS            = 0,
    IQM_VERT_UV             = 1,
    IQM_VERT_NOR            = 2,
    IQM_VERT_TAN            = 3,
    IQM_VERT_BONE_IDXS      = 4,
    IQM_VERT_BONE_WEIGHTS   = 5,
    IQM_VERT_COLOR          = 6,
    IQM_VERT_CUSTOM         = 0X10
};

enum class iqm_dtype : unsigned int {
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
    unsigned int vert_idxs[3];
} iqm_tri_t;


typedef struct iqm_joint_euler_s {
    unsigned int name;
    int parent_joint_idx;
    float translate[3], rotate[3], scale[3];
} iqm_joint_euler_t;


typedef struct iqm_joint_quaternion_s {
    unsigned int name;
    int parent_joint_idx;
    float translate[3], rotate[4], scale[3];
} iqm_joint_quaternion_t;


typedef struct iqm_pose_euler_s {
    int parent_idx; // Parent POSE idx? parent bone idx?
    unsigned int mask;
    float channel_ofs[9];
    float channel_scale[9];
} iqm_pose_euler_t;


typedef struct iqm_pose_quaternion_s {
    int parent_idx; // Parent POSE idx? parent bone idx?
    unsigned int mask;
    float channel_ofs[10];
    float channel_scale[10];
} iqm_pose_quaternion_t;

typedef struct iqm_anim_s {
    unsigned int name;
    unsigned int first_frame, n_frames;
    float framerate;
    unsigned int flags;
} iqm_anim_t;


enum class iqm_anim_flag {
    IQM_ANIM_FLAG_LOOP = 1<<0
};


typedef struct iqm_vert_array_s {
    unsigned int type; // TODO - iqm_vert_array_type?
    unsigned int flags;
    unsigned int format;// TODO - iqm_dtype
    unsigned int size;
    unsigned int ofs;
} iqm_vert_array_t;


typedef struct iqm_bounds_s {
    float mins[3], maxs[3];
    float xyradius;
    float radius;
} iqm_bounds_t;


typedef struct iqm_extension_s {
    unsigned int name;
    unsigned int n_data, ofs_data;
    unsigned int ofs_extensions; // Pointer to next extension?
} iqm_extension_t;


typedef struct iqm_ext_fte_mesh_s {
    unsigned int contents;      // default: CONTENTS_BODY
    unsigned int surfaceflags;  // Propagates to trace_surfaceflags
    unsigned int surfaceid;     // Body reported to QC via trace_surface
    unsigned int geomset;
    unsigned int geomid;
    float min_dist;
    float max_dist;
} iqm_ext_fte_mesh_t;


typedef struct iqm_ext_fte_event_s {
    unsigned int anim;
    float timestamp;
    unsigned int event_code;
    unsigned int event_data_str;        // Stringtable
} iqm_ext_fte_event_t;


typedef struct iqm_ext_fte_skin_s {
    unsigned int n_skinframes;
    unsigned int n_meshskins;
} iqm_ext_fte_skin_t;


typedef struct iqm_ext_fte_skin_skinframe_s {
    unsigned int material_idx;
    unsigned int shadertext_idx;
} iqm_ext_fte_skin_skinframe_t;


typedef struct iqm_ext_fte_skin_meshskin_s {
    unsigned int first_frame;
    unsigned int n_frames;
    float interval;
} iqm_ext_fte_skin_meshskin_t;




typedef struct vertex_s {
    float u, v;
    unsigned int color;
    float x,y,z;
} vertex_t;


typedef struct mesh_s {
    // Number of vertices in submesh (FIXME - what is this exactly?)
    unsigned int n_verts;
    // Number of triangles in submesh
    unsigned int n_tris;
    // Contains vertex indices
    // TODO - Use whatever data struct sceGU expects as vert indices for triangles:
    // unsigned short 
    uint16_t *tri_verts;
    unsigned int n_tri_verts;
    unsigned int first_vert;
} mesh_t;


// Temp container:
typedef struct model_s {
    unsigned int n_verts;
    vertex_t *verts;
    unsigned int n_meshes;
    mesh_t *meshes;
} model_t;



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




model_t *load_iqm_file(const char*file_path) {
    FILE *f = fopen(file_path, "rb");
    fseek(f, 0, SEEK_END);
    size_t file_len = ftell(f);
    rewind(f);
    uint8_t *iqm_data = (uint8_t*) malloc(sizeof(uint8_t) * file_len);
    fread(iqm_data, file_len, 1, f);
    fclose(f);

    iqm_header_t *iqm_header = (iqm_header_t*) iqm_data;
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
    for(unsigned int i = 0; i < model->n_verts; i++) {
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

    for(unsigned int i = 0; i < iqm_header->n_meshes; i++) {

        unsigned int first_vert = iqm_meshes[i].first_vert_idx;
        unsigned int n_verts = iqm_meshes[i].n_verts;
        unsigned int first_tri = iqm_meshes[i].first_tri;
        unsigned int n_tris = iqm_meshes[i].n_tris;
        model->meshes[i].n_tri_verts = n_tris * 3;
        model->meshes[i].tri_verts = (uint16_t*) malloc(sizeof(uint16_t) * n_tris * 3);
        model->meshes[i].first_vert = first_vert;


        for(unsigned int j = 0; j < n_tris; j++) {
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


    return model;
    // FIXME - massive memory leaks happening here



    // TODO - Free iqm_data
}

