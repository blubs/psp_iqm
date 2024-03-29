#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


#define F_clock_gettime


#include <pspkernel.h>
#include <pspdisplay.h>
#include <pspdebug.h>
#include <pspgu.h>
#include <pspgum.h>
#include <pspctrl.h>


#include "callbacks.h"
#include "vram.h"



#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_ONLY_TGA
#include "stb_image.h"


#include "iqm.h"


PSP_MODULE_INFO("IQM Test Project", 0, 1, 1);
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER | THREAD_ATTR_VFPU);

#define printf pspDebugScreenPrintf



#define BUF_WIDTH (512)
#define SCR_WIDTH (480)
#define SCR_HEIGHT (272)
// static unsigned int __attribute__((aligned(16))) display_list[262144];
// static unsigned int __attribute__((aligned(16))) display_list[524288];
static unsigned int __attribute__((aligned(16))) display_list[1048576];


void *draw_buffer;
void *display_buffer;
void *depth_buffer;




void init_gu() {
    sceGuInit();
    sceGuStart(GU_DIRECT, display_list);

    // Setup buffers
    draw_buffer = getStaticVramBuffer(BUF_WIDTH, SCR_HEIGHT, GU_PSM_8888);
    display_buffer = getStaticVramBuffer(BUF_WIDTH, SCR_HEIGHT, GU_PSM_8888);
    depth_buffer = getStaticVramBuffer(BUF_WIDTH, SCR_HEIGHT, GU_PSM_4444);
    sceGuDrawBuffer(GU_PSM_8888, draw_buffer, BUF_WIDTH);
    sceGuDispBuffer(SCR_WIDTH, SCR_HEIGHT, display_buffer, BUF_WIDTH);
    sceGuDepthBuffer(depth_buffer, BUF_WIDTH);

    // Center the region where drawing is performed in the virtual coord space
    unsigned int virtual_coordinate_space_width = 4096;
    unsigned int virtual_coordinate_space_height = 4096;
    // X-coord of top-left corner
    unsigned int psp_screen_tl_x = (virtual_coordinate_space_width/2) - (SCR_WIDTH/2);
    // Y-coord of top-left corner 
    unsigned int psp_screen_tl_y = (virtual_coordinate_space_height/2) - (SCR_HEIGHT/2);
    sceGuOffset(psp_screen_tl_x, psp_screen_tl_y);

    // Center window-size viewport  in virtual coord space
    unsigned int center_x = virtual_coordinate_space_width/2;
    unsigned int center_y = virtual_coordinate_space_height/2;
    sceGuViewport(center_x, center_y, SCR_WIDTH, SCR_HEIGHT);

    // Set near and far range values for depth buffer
    sceGuDepthRange((1<<16) - 1, 0);
    // Depth buffer is inverted (a closer than b --> a > b)
    sceGuDepthFunc(GU_GEQUAL);
    sceGuEnable(GU_DEPTH_TEST);
    // Specify region to scissor
    sceGuScissor(0,0,SCR_WIDTH,SCR_HEIGHT);
    sceGuEnable(GU_SCISSOR_TEST);
    sceGuFrontFace(GU_CW);
    sceGuShadeModel(GU_SMOOTH);
    sceGuEnable(GU_CULL_FACE);
    sceGuEnable(GU_TEXTURE_2D);
    sceGuEnable(GU_CLIP_PLANES);
    sceGuFinish();
    // Wait for draw commands to finish processing
    sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
    // Wait for vertical blank start
    sceDisplayWaitVblankStart();
    // Turn display on
    sceGuDisplay(GU_TRUE);
}



typedef struct texture_s {
    int width;
    int height;
    uint8_t *data; // Pointer to RGBA 8888 texture data (4 * width * height) bytes
} texture_t;


int load_png_file(const char *file, texture_t *tex) {
    int bytes_per_pixel;
    int img_width;
    int img_height;
    FILE *f = fopen(file, "rb");
    uint8_t *data = stbi_load_from_file(f, &img_width, &img_height, &bytes_per_pixel, 4);
    fclose(f);
    if(data) {
        tex->width = img_width;
        tex->height = img_height;
        size_t n_bytes = 4 * img_width * img_height;
        tex->data = (uint8_t*) malloc(n_bytes);
        memcpy(tex->data, data, n_bytes);
        free(data);
        return 0;
    }
    return -1;
}


#include "logging.h"

double get_epoch_time() {
    clockid_t clk_id;
    struct timespec tspec;
    clock_gettime(clk_id, &tspec);

    // Time since epoch
    // tspec.tv_sec;
    // tspec.tv_nsec;
    double epoch_time = tspec.tv_sec + (tspec.tv_nsec / 1e9);
    return epoch_time;
}


// ----------------------------------------------------------------------------
// Drawing functions
// ----------------------------------------------------------------------------
void draw_skeletal_model_indexed_swskinning_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->verts);
    }
}


void draw_skeletal_model_indexed_swskinning_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);

        sceGumPushMatrix();
        // sceGumLoadIdentity();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert16s);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_indexed_swskinning_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);

        sceGumPushMatrix();
        // sceGumLoadIdentity();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert8s);
        sceGumPopMatrix();
    }
}


void bind_submesh_bones_float32(skeletal_mesh_t *submesh, skeletal_skeleton_t *skeleton) {
    for(int submesh_bone_idx = 0; submesh_bone_idx < submesh->n_skinning_bones; submesh_bone_idx++) {
        // Get the index into the skeleton list of bones:
        int bone_idx = submesh->skinning_bone_idxs[submesh_bone_idx];
        mat3x4_t *bone_mat3x4 = &(skeleton->bone_transforms[bone_idx]);

        // Translate the mat3x4_t bone transform matrix to ScePspFMatrix4
        ScePspFMatrix4 bone_mat;
        bone_mat.x.x = bone_mat3x4->m[0];   bone_mat.y.x = bone_mat3x4->m[3];   bone_mat.z.x = bone_mat3x4->m[6];   bone_mat.w.x = bone_mat3x4->m[9];
        bone_mat.x.y = bone_mat3x4->m[1];   bone_mat.y.y = bone_mat3x4->m[4];   bone_mat.z.y = bone_mat3x4->m[7];   bone_mat.w.y = bone_mat3x4->m[10];
        bone_mat.x.z = bone_mat3x4->m[2];   bone_mat.y.z = bone_mat3x4->m[5];   bone_mat.z.z = bone_mat3x4->m[8];   bone_mat.w.z = bone_mat3x4->m[11];
        bone_mat.x.w = 0.0f;                bone_mat.y.w = 0.0f;                bone_mat.z.w = 0.0f;                bone_mat.w.w = 1.0f;

        sceGuBoneMatrix(submesh_bone_idx, &bone_mat);
    }
}

void bind_submesh_bones_int8_int16(skeletal_mesh_t *submesh, skeletal_skeleton_t *skeleton) {

    // Transform matrix that undoes int16 quantization scale + ofs
    ScePspFMatrix4 undo_quantization;
    gumLoadIdentity(&undo_quantization);
    ScePspFVector3 undo_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
    ScePspFVector3 undo_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
    gumTranslate(&undo_quantization, &undo_ofs);
    gumScale(&undo_quantization, &undo_scale);
    
    // Transform matrix that reapplies int16 quantization scale + ofs
    ScePspFMatrix4 redo_quantization;
    gumLoadIdentity(&redo_quantization);
    ScePspFVector3 redo_ofs = {-submesh->hw_verts_ofs.x,-submesh->hw_verts_ofs.y,-submesh->hw_verts_ofs.z};
    ScePspFVector3 redo_scale = {1.0f/submesh->hw_verts_scale.x,1.0f/submesh->hw_verts_scale.y,1.0f/submesh->hw_verts_scale.z};
    gumScale(&redo_quantization, &redo_scale);
    gumTranslate(&redo_quantization, &redo_ofs);

    for(int submesh_bone_idx = 0; submesh_bone_idx < submesh->n_skinning_bones; submesh_bone_idx++) {
        // Get the index into the skeleton list of bones:
        int bone_idx = submesh->skinning_bone_idxs[submesh_bone_idx];
        mat3x4_t *bone_mat3x4 = &(skeleton->bone_transforms[bone_idx]);
        // float apply_vert_scale = submesh->verts_unit_scale;
        // float undo_vert_scale = 1.0f / submesh->verts_unit_scale;

        // Translate the mat3x4_t bone transform matrix to ScePspFMatrix4
        ScePspFMatrix4 bone_mat;
        bone_mat.x.x = bone_mat3x4->m[0];   bone_mat.y.x = bone_mat3x4->m[3];   bone_mat.z.x = bone_mat3x4->m[6];   bone_mat.w.x = bone_mat3x4->m[9];
        bone_mat.x.y = bone_mat3x4->m[1];   bone_mat.y.y = bone_mat3x4->m[4];   bone_mat.z.y = bone_mat3x4->m[7];   bone_mat.w.y = bone_mat3x4->m[10];
        bone_mat.x.z = bone_mat3x4->m[2];   bone_mat.y.z = bone_mat3x4->m[5];   bone_mat.z.z = bone_mat3x4->m[8];   bone_mat.w.z = bone_mat3x4->m[11];
        bone_mat.x.w = 0.0f;                bone_mat.y.w = 0.0f;                bone_mat.z.w = 0.0f;                bone_mat.w.w = 1.0f;

        // bone_mat = redo_quantization * bone_mat * undo_quantization
        gumMultMatrix(&bone_mat, &bone_mat, &undo_quantization);
        gumMultMatrix(&bone_mat, &redo_quantization, &bone_mat);
        sceGuBoneMatrix(submesh_bone_idx, &bone_mat);
    }
}



void draw_skeletal_model_indexed_hwskinning_float32_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                submesh->skinning_verts
            );
        }
    }
}


void draw_skeletal_model_indexed_hwskinning_float32_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                // submesh->skinning_verts
                submesh->skinning_verts32_w16
            );
        }
    }
}

void draw_skeletal_model_indexed_hwskinning_float32_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                // submesh->skinning_verts
                submesh->skinning_verts32_w8
            );
        }
    }
}



void draw_skeletal_model_indexed_hwskinning_int16_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);

            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                // submesh->skinning_verts
                submesh->skinning_vert16s_w32
            );

            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_indexed_hwskinning_int8_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);

            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                // submesh->skinning_verts
                submesh->skinning_vert8s_w32
            );

            sceGumPopMatrix();
        }
    }
}


void draw_skeletal_model_indexed_hwskinning_int16_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);

            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                submesh->skinning_vert16s_w16
            );
            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_indexed_hwskinning_int8_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);

            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                submesh->skinning_vert8s_w16
            );
            sceGumPopMatrix();
        }
    }
}



void draw_skeletal_model_indexed_hwskinning_int16_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                submesh->skinning_vert16s_w8
            );
            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_indexed_hwskinning_int8_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                submesh->tri_verts, 
                submesh->skinning_vert8s_w8
            );
            sceGumPopMatrix();
        }
    }
}


void draw_skeletal_model_indexed_swblending_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            mesh->verts[j].x = 0.5 * (mesh->verts[j].x + mesh->verts[j].x);
            mesh->verts[j].y = 0.5 * (mesh->verts[j].y + mesh->verts[j].y);
            mesh->verts[j].z = 0.5 * (mesh->verts[j].z + mesh->verts[j].z);
        }
        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->verts);
    }
}


void draw_skeletal_model_indexed_hwblending_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        vertex_t * const verts_data = static_cast<vertex_t*>(sceGuGetMemory(sizeof(vertex_t) * mesh->n_verts * 2));
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->verts[j].u;
            verts_data[j*2 + 0].v = mesh->verts[j].v;
            verts_data[j*2 + 1].u = mesh->verts[j].u;
            verts_data[j*2 + 1].v = mesh->verts[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->verts[j].x;
            verts_data[j*2 + 0].y       = mesh->verts[j].y;
            verts_data[j*2 + 0].z       = mesh->verts[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->verts[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->verts[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->verts[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->verts[j].x;
            verts_data[j*2 + 1].y       = mesh->verts[j].y;
            verts_data[j*2 + 1].z       = mesh->verts[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->verts[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->verts[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->verts[j].nor_z;
        }

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, mesh->tri_verts, verts_data);
    }
}


void draw_skeletal_model_indexed_swblending_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            mesh->vert16s[j].x = (mesh->vert16s[j].x + mesh->vert16s[j].x) / 2;
            mesh->vert16s[j].y = (mesh->vert16s[j].y + mesh->vert16s[j].y) / 2;
            mesh->vert16s[j].z = (mesh->vert16s[j].z + mesh->vert16s[j].z) / 2;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert16s);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_indexed_swblending_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            mesh->vert8s[j].x = (mesh->vert8s[j].x + mesh->vert8s[j].x) / 2;
            mesh->vert8s[j].y = (mesh->vert8s[j].y + mesh->vert8s[j].y) / 2;
            mesh->vert8s[j].z = (mesh->vert8s[j].z + mesh->vert8s[j].z) / 2;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert8s);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_indexed_hwblending_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        alt_vertex16_t * const verts_data = static_cast<alt_vertex16_t*>(sceGuGetMemory(sizeof(alt_vertex16_t) * mesh->n_verts * 2));
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->vert16s[j].u;
            verts_data[j*2 + 0].v = mesh->vert16s[j].v;
            verts_data[j*2 + 1].u = mesh->vert16s[j].u;
            verts_data[j*2 + 1].v = mesh->vert16s[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->vert16s[j].x;
            verts_data[j*2 + 0].y       = mesh->vert16s[j].y;
            verts_data[j*2 + 0].z       = mesh->vert16s[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->vert16s[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->vert16s[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->vert16s[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->vert16s[j].x;
            verts_data[j*2 + 1].y       = mesh->vert16s[j].y;
            verts_data[j*2 + 1].z       = mesh->vert16s[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->vert16s[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->vert16s[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->vert16s[j].nor_z;
        }

        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, mesh->tri_verts, verts_data);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_indexed_hwblending_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        alt_vertex8_t * const verts_data = static_cast<alt_vertex8_t*>(sceGuGetMemory(sizeof(alt_vertex8_t) * mesh->n_verts * 2));
        for(uint32_t j = 0; j < mesh->n_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->vert8s[j].u;
            verts_data[j*2 + 0].v = mesh->vert8s[j].v;
            verts_data[j*2 + 1].u = mesh->vert8s[j].u;
            verts_data[j*2 + 1].v = mesh->vert8s[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->vert8s[j].x;
            verts_data[j*2 + 0].y       = mesh->vert8s[j].y;
            verts_data[j*2 + 0].z       = mesh->vert8s[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->vert8s[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->vert8s[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->vert8s[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->vert8s[j].x;
            verts_data[j*2 + 1].y       = mesh->vert8s[j].y;
            verts_data[j*2 + 1].z       = mesh->vert8s[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->vert8s[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->vert8s[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->vert8s[j].nor_z;
        }

        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, mesh->tri_verts, verts_data);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_indexed_static_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->verts);
    }
}


void draw_skeletal_model_indexed_static_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);

        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert16s);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_indexed_static_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);

        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert8s);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_unindexed_hwskinning_float32_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);

            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert32s_w32
            );
        }
    }
}


void draw_skeletal_model_unindexed_hwskinning_float32_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert32s_w16
            );
        }
    }
}


void draw_skeletal_model_unindexed_hwskinning_float32_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_float32(submesh, skeleton);
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert32s_w8
            );
        }
    }
}


void draw_skeletal_model_unindexed_swskinning_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_verts);
    }
}


void draw_skeletal_model_unindexed_swskinning_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert16s);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_unindexed_swskinning_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert8s);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_unindexed_hwskinning_int16_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert16s_w32
            );
            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_unindexed_hwskinning_int8_float32weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                // GU_TRIANGLES,GU_INDEX_16BIT|GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, 
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_32BITF|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert8s_w32
            );
            sceGumPopMatrix();
        }
    }
}


void draw_skeletal_model_unindexed_hwskinning_int16_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert16s_w16
            );
            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_unindexed_hwskinning_int8_int16weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_16BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert8s_w16
            );
            sceGumPopMatrix();
        }
    }
}



void draw_skeletal_model_unindexed_hwskinning_int16_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert16s_w8
            );
            sceGumPopMatrix();
        }
    }
}

void draw_skeletal_model_unindexed_hwskinning_int8_int8weights(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            bind_submesh_bones_int8_int16(submesh, skeleton);

            sceGumPushMatrix();
            ScePspFVector3 verts_ofs = {submesh->hw_verts_ofs.x,submesh->hw_verts_ofs.y,submesh->hw_verts_ofs.z};
            ScePspFVector3 verts_scale = {submesh->hw_verts_scale.x,submesh->hw_verts_scale.y,submesh->hw_verts_scale.z};
            sceGumTranslate(&verts_ofs);
            sceGumScale(&verts_scale);
            
            // GU_COLOR_8888
            sceGumDrawArray(
                GU_TRIANGLES,GU_WEIGHTS(8)|GU_WEIGHT_8BIT|GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, 
                submesh->n_tris * 3, 
                nullptr, 
                submesh->unindexed_skinning_vert8s_w8
            );
            sceGumPopMatrix();
        }
    }
}


void draw_skeletal_model_unindexed_swblending_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            mesh->unindexed_verts[j].x = 0.5 * (mesh->unindexed_verts[j].x + mesh->unindexed_verts[j].x);
            mesh->unindexed_verts[j].y = 0.5 * (mesh->unindexed_verts[j].y + mesh->unindexed_verts[j].y);
            mesh->unindexed_verts[j].z = 0.5 * (mesh->unindexed_verts[j].z + mesh->unindexed_verts[j].z);
        }
        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_verts);
    }
}

void draw_skeletal_model_unindexed_hwblending_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        vertex_t * const verts_data = static_cast<vertex_t*>(sceGuGetMemory(sizeof(vertex_t) * mesh->n_unindexed_verts * 2));
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->unindexed_verts[j].u;
            verts_data[j*2 + 0].v = mesh->unindexed_verts[j].v;
            verts_data[j*2 + 1].u = mesh->unindexed_verts[j].u;
            verts_data[j*2 + 1].v = mesh->unindexed_verts[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->unindexed_verts[j].x;
            verts_data[j*2 + 0].y       = mesh->unindexed_verts[j].y;
            verts_data[j*2 + 0].z       = mesh->unindexed_verts[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->unindexed_verts[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->unindexed_verts[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->unindexed_verts[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->unindexed_verts[j].x;
            verts_data[j*2 + 1].y       = mesh->unindexed_verts[j].y;
            verts_data[j*2 + 1].z       = mesh->unindexed_verts[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->unindexed_verts[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->unindexed_verts[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->unindexed_verts[j].nor_z;
        }

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, nullptr, verts_data);
    }
}


void draw_skeletal_model_unindexed_swblending_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            mesh->unindexed_vert16s[j].x = (mesh->unindexed_vert16s[j].x + mesh->unindexed_vert16s[j].x) / 2;
            mesh->unindexed_vert16s[j].y = (mesh->unindexed_vert16s[j].y + mesh->unindexed_vert16s[j].y) / 2;
            mesh->unindexed_vert16s[j].z = (mesh->unindexed_vert16s[j].z + mesh->unindexed_vert16s[j].z) / 2;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert16s);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_unindexed_swblending_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            mesh->unindexed_vert8s[j].x = (mesh->unindexed_vert8s[j].x + mesh->unindexed_vert8s[j].x) / 2;
            mesh->unindexed_vert8s[j].y = (mesh->unindexed_vert8s[j].y + mesh->unindexed_vert8s[j].y) / 2;
            mesh->unindexed_vert8s[j].z = (mesh->unindexed_vert8s[j].z + mesh->unindexed_vert8s[j].z) / 2;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert8s);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_unindexed_hwblending_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        alt_vertex16_t * const verts_data = static_cast<alt_vertex16_t*>(sceGuGetMemory(sizeof(alt_vertex16_t) * mesh->n_unindexed_verts * 2));
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->unindexed_vert16s[j].u;
            verts_data[j*2 + 0].v = mesh->unindexed_vert16s[j].v;
            verts_data[j*2 + 1].u = mesh->unindexed_vert16s[j].u;
            verts_data[j*2 + 1].v = mesh->unindexed_vert16s[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->unindexed_vert16s[j].x;
            verts_data[j*2 + 0].y       = mesh->unindexed_vert16s[j].y;
            verts_data[j*2 + 0].z       = mesh->unindexed_vert16s[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->unindexed_vert16s[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->unindexed_vert16s[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->unindexed_vert16s[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->unindexed_vert16s[j].x;
            verts_data[j*2 + 1].y       = mesh->unindexed_vert16s[j].y;
            verts_data[j*2 + 1].z       = mesh->unindexed_vert16s[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->unindexed_vert16s[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->unindexed_vert16s[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->unindexed_vert16s[j].nor_z;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, nullptr, verts_data);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_unindexed_hwblending_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        alt_vertex8_t * const verts_data = static_cast<alt_vertex8_t*>(sceGuGetMemory(sizeof(alt_vertex8_t) * mesh->n_unindexed_verts * 2));
        for(uint32_t j = 0; j < mesh->n_unindexed_verts; j++) {
            // Static fields:
            verts_data[j*2 + 0].u = mesh->unindexed_vert8s[j].u;
            verts_data[j*2 + 0].v = mesh->unindexed_vert8s[j].v;
            verts_data[j*2 + 1].u = mesh->unindexed_vert8s[j].u;
            verts_data[j*2 + 1].v = mesh->unindexed_vert8s[j].v;

            // Fake blending: (Pulled from the same mesh / vertices)
            verts_data[j*2 + 0].x       = mesh->unindexed_vert8s[j].x;
            verts_data[j*2 + 0].y       = mesh->unindexed_vert8s[j].y;
            verts_data[j*2 + 0].z       = mesh->unindexed_vert8s[j].z;
            verts_data[j*2 + 0].nor_x   = mesh->unindexed_vert8s[j].nor_x;
            verts_data[j*2 + 0].nor_y   = mesh->unindexed_vert8s[j].nor_y;
            verts_data[j*2 + 0].nor_z   = mesh->unindexed_vert8s[j].nor_z;
            verts_data[j*2 + 1].x       = mesh->unindexed_vert8s[j].x;
            verts_data[j*2 + 1].y       = mesh->unindexed_vert8s[j].y;
            verts_data[j*2 + 1].z       = mesh->unindexed_vert8s[j].z;
            verts_data[j*2 + 1].nor_x   = mesh->unindexed_vert8s[j].nor_x;
            verts_data[j*2 + 1].nor_y   = mesh->unindexed_vert8s[j].nor_y;
            verts_data[j*2 + 1].nor_z   = mesh->unindexed_vert8s[j].nor_z;
        }
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);

        // GU_COLOR_8888
        float blend = 0.5f;
        sceGuMorphWeight(0, 1 - blend);
        sceGuMorphWeight(1, blend);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D|GU_VERTICES(2), mesh->n_tris * 3, nullptr, verts_data);
        sceGumPopMatrix();
    }
}


void draw_skeletal_model_unindexed_static_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_verts);
    }
}


void draw_skeletal_model_unindexed_static_int16(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert16s);
        sceGumPopMatrix();
    }
}

void draw_skeletal_model_unindexed_static_int8(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        sceGumPushMatrix();
        ScePspFVector3 verts_ofs = {mesh->sw_verts_ofs.x,mesh->sw_verts_ofs.y,mesh->sw_verts_ofs.z};
        ScePspFVector3 verts_scale = {mesh->sw_verts_scale.x,mesh->sw_verts_scale.y,mesh->sw_verts_scale.z};
        sceGumTranslate(&verts_ofs);
        sceGumScale(&verts_scale);
        sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_8BIT|GU_NORMAL_8BIT|GU_VERTEX_8BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, nullptr, mesh->unindexed_vert8s);
        sceGumPopMatrix();
    }
}
// ----------------------------------------------------------------------------













int main(int argc, char *argv[]) {
    log_printf("Starting program\n");
    setupCallbacks();
    init_gu();
    pspDebugScreenInit();

    FILE *f = fopen("test.txt", "r");
    if(f == NULL) {
        pspDebugScreenPrintf("failed to open file\n");
    }
    else {
        char text[64];
        fgets(text, 64, f);
        // Drop newline:
        text[strcspn(text, "\n")] = 0;
        fscanf(f, "%s", text);
        pspDebugScreenPrintf("Reading file: \"test.txt\"\n");
        pspDebugScreenPrintf("Read chars from file: %d\n", strlen(text));
        pspDebugScreenPrintf("Read text: \"%s\"\n", text);
    }

    texture_t *zombie_tex = (texture_t*) malloc(sizeof(texture_t));
    texture_t *eyeglow_tex = (texture_t*) malloc(sizeof(texture_t));

    // int iqm_version = load_iqm_file("assets/zombie_with_anims.iqm");
    // iqm_header_t *iqm_header = load_iqm_file("assets/zombie_with_anims.iqm");

    // skeletal_model_t *iqm_model = load_iqm_file("assets/zombie_with_anims.iqm");
    // skeletal_model_t *iqm_model = load_iqm_file("assets/test_model.iqm");
    // skeletal_model_t *iqm_model = load_iqm_file("assets/test_model_2.iqm");
    log_printf("Loading zombie mesh\n");
    skeletal_model_t *zombie_model = load_iqm_file("assets/zombie_mesh.iqm");
    log_printf("Loading zombie anim 1\n");
    skeletal_model_t *zombie_anim_walk1 = load_iqm_file("assets/anim1_v2.iqm");
    log_printf("Loading zombie anim 2\n");
    skeletal_model_t *zombie_anim_walk2 = load_iqm_file("assets/anim2_v2.iqm");
    log_printf("Done loading IQM assets\n");
    skeletal_model_t *iqm_model = zombie_model;
    skeletal_model_t *iqm_anim = zombie_anim_walk1;
    // skeletal_model_t *iqm_anim = zombie_anim_walk2;
    float scale = 0.1f;
    // float scale = 1.0f;

    float rot_speed = 0.32f;
    // float rot_speed = 1.0f;


    log_printf("Creating skeleton for zombie mesh\n");
    skeletal_skeleton_t *iqm_skeleton = create_skeleton(iqm_model);
    log_printf("Done creating skeleton for zombie mesh\n");
    int framegroup_idx = 0;


    load_png_file("assets/zombie_tex_0.png", zombie_tex);
    load_png_file("assets/eyeglow_tex.png", eyeglow_tex);

    // texture_t *envmap_tex = (texture_t*) malloc(sizeof(texture_t));
    // load_png_file("assets/env/ndulf.png", envmap_tex);

    // --------------------------------------------
    // Setup PSP input    
    // --------------------------------------------
    SceCtrlData cur_input;
    uint32_t prev_input_buttons = 0;
    sceCtrlSetSamplingCycle(0);
    sceCtrlSetSamplingMode(0);
    // --------------------------------------------

    void *cur_draw_buffer = draw_buffer;
    fclose(f);
    unsigned int frame = 0;
    float frametime = 0;

    double start_epoch_time = get_epoch_time();
    // The number of frames over which to average the FPS
#define FPS_FRAMES_WINDOW 10
    double fps_frametimes[FPS_FRAMES_WINDOW];
    for(int i = 0; i < FPS_FRAMES_WINDOW; i++) {
        fps_frametimes[i] = start_epoch_time;
    }
    double peak_fps = 0.0;
    int peak_fps_delay = 60; // Delay measurement of peak fps by 60 frames


    const int DRAW_MODE_ANIMATION_SWBLENDING = 0;
    const int DRAW_MODE_ANIMATION_HWBLENDING = 1;
    const int DRAW_MODE_ANIMATION_SWSKINNING = 2;
    const int DRAW_MODE_ANIMATION_HWSKINNING = 3;
    const int DRAW_MODE_ANIMATION_STATIC = 4;
    const int N_DRAW_MODES_ANIMATION = 5;

    const int DRAW_MODE_VERTEX_SIZE_FLOAT32 = 0;
    const int DRAW_MODE_VERTEX_SIZE_INT16 = 1;
    const int DRAW_MODE_VERTEX_SIZE_INT8 = 2;
    const int N_DRAW_MODES_VERTEX_SIZE = 3;

    const int DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32 = 0;
    const int DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16 = 1;
    const int DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8 = 2;
    const int N_DRAW_MODES_HWSKINNING_WEIGHT_SIZE = 3;

    const int DRAW_MODE_INDEXED_NO = 0;
    const int DRAW_MODE_INDEXED_YES = 1;
    const int N_DRAW_MODES_INDEXED = 2;

    // Initial draw mode:
    int draw_mode_animation = DRAW_MODE_ANIMATION_SWBLENDING;
    int draw_mode_vertex_size = DRAW_MODE_VERTEX_SIZE_FLOAT32;
    int draw_mode_hwskinning_weight_size = DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32;
    int draw_mode_indexed = DRAW_MODE_INDEXED_NO;

    // -------------------------------



    while(running()) {
        const float DEG2RAD = (GU_PI/180.0f);

        sceGuStart(GU_DIRECT, display_list);
        // Smoothly fade between 0 and 1:
        // float fade = 0.5f * (sin((float)frame / 10.0f) + 1.0f);
        // float fade = 0;
        float fade = 1.0;
        sceGuClearColor((int)(0x0000ff * fade));
        sceGuClearDepth(0);
        sceGuClear(GU_COLOR_BUFFER_BIT | GU_DEPTH_BUFFER_BIT);

        // --------------------------------------------------------------------
        // Let's try drawing...
        // --------------------------------------------------------------------
        sceGumMatrixMode(GU_PROJECTION);
        sceGumLoadIdentity();
        sceGumPerspective(75.0f, 16.0f/9.0f, 0.5f, 1000.0f);

        // --------------------------------------------
        // Move the camera
        // --------------------------------------------
        // float sin_time = sin((float)frame / 10.0f);

        sceGumMatrixMode(GU_VIEW);
        sceGumLoadIdentity();
        ScePspFVector3 camera_pos = {0,10.0f, 20.0f};
        ScePspFVector3 camera_rot = {-20 * DEG2RAD, 0, 0};
        sceGumTranslate(&camera_pos);
        sceGumRotateXYZ(&camera_rot);
        sceGumFastInverse();
        // --------------------------------------------


        float grid_min_x = -10.0f;
        float grid_max_x = 10.0f;
        float grid_min_z = -10.0f;
        float grid_max_z = 10.0f;

        int grid_models_x = 5;
        int grid_models_z = 5;

        // Derived vars
        int grid_n_models = grid_models_x * grid_models_z;
        float grid_x_spacing = (grid_max_x - grid_min_x) / (grid_models_x - 1);
        float grid_z_spacing = (grid_max_z - grid_min_z) / (grid_models_z - 1);
        

        // Draw multiple models
        for(int i = 0; i < grid_n_models; i++) {
            int model_x_idx = (float) ((int) (i % grid_models_x));
            int model_z_idx = (float) ((int) i / grid_models_x);
            float model_pos_x = grid_min_x + (model_x_idx * grid_x_spacing);
            float model_pos_z = grid_min_z + (model_z_idx * grid_z_spacing);
            float model_pos_y = 0.0f;


            sceGumMatrixMode(GU_MODEL);
            sceGumLoadIdentity();
            ScePspFVector3 model_pos = {
                model_pos_x,
                model_pos_y,
                model_pos_z
            };
            // ScePspFVector3 model_rot = {frame * 0.79f * (GU_PI/180.0f), frame * 0.98f * (GU_PI/180.0f), frame * 1.32f * (GU_PI/180.0f)};
            // ScePspFVector3 model_rot = {0, frame * 1.32f * (GU_PI/180.0f), 0};
            // Disable rotation for more stable FPS measurements
            rot_speed = 0.0f;
            ScePspFVector3 model_rot = {-90 * DEG2RAD, 0, (frame * rot_speed + -90) * DEG2RAD};
            // ScePspFVector3 model_rot = {-90 * (GU_PI/180.0f), 0, (-90) * (GU_PI/180.0f)};
            // float scale = (sin((float)frame / 10.0f) + 1.0f) * 100;
            ScePspFVector3 model_scale = {scale,scale,scale};


            sceGumTranslate(&model_pos);
            sceGumScale(&model_scale);
            sceGumRotateXYZ(&model_rot);

            // // FIXME - Need to apply this ... PER mesh...
            // if(draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16 || draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8) {
            //     // Undo the float32 -> int16 and float32 -> int8 scaling
            //     // Only do it once for all meshes that use int16 or int8 vertices
            //     float undo_scale = 1.0f / iqm_model->meshes[0].verts_unit_scale;
            //     ScePspFVector3 undo_scale_vec3 = {undo_scale,undo_scale,undo_scale};
            //     sceGumScale(&undo_scale_vec3);
            // }

            // -----------------------------------
            // Set up envmap texture
            // -----------------------------------
            // sceGuTexMode(GU_PSM_8888,0,0,0);
            // sceGuTexImage(0,envmap_tex->width,envmap_tex->height,envmap_tex->width,envmap_tex->data);
            // sceGuTexFunc(GU_TFX_ADD, GU_TCC_RGB);
            // sceGuTexFilter(GU_LINEAR,GU_LINEAR);

            // // float envmap_angle = -2.0f * frame * (GU_PI / 180.0f);
            // float envmap_angle = -2.0f * (GU_PI / 180.0f);
            // float envmap_cos = cosf(envmap_angle);
            // float envmap_sin = sinf(envmap_angle);
            // ScePspFVector3 envmap_matrix_columns[2] = {
            //     {  envmap_cos, envmap_sin, 0.0f,},
            //     { -envmap_sin, envmap_cos, 0.0f }
            // };

            // sceGuLight( 1, GU_DIRECTIONAL, GU_DIFFUSE, &envmap_matrix_columns[0]);
            // sceGuLight( 2, GU_DIRECTIONAL, GU_DIFFUSE, &envmap_matrix_columns[1]);
            // sceGuTexMapMode(
            //     GU_ENVIRONMENT_MAP,
            //     1,
            //     2
            // );
            // -----------------------------------


            // -----------------------------------
            // Translate IQM model data to PSP format:
            // FIXME - This is bad...
            // -----------------------------------
            sceGuTexMode(GU_PSM_8888,0,0,0);
            sceGuTexImage(0,zombie_tex->width,zombie_tex->height,zombie_tex->width,zombie_tex->data);
            sceGuTexFunc(GU_TFX_ADD,GU_TCC_RGB);
            sceGuTexEnvColor(0xffff00);
            sceGuTexFilter(GU_LINEAR, GU_LINEAR);
            sceGuTexScale(1.0f,1.0f);
            sceGuTexOffset(0.0f,0.0f);
            sceGuAmbientColor(0xffffffff);


            // Set primitive color for all vertices:
            sceGuColor(0x00000000);



            // --------------------------------------------------------------------
            // Updating the model's animation / drawing state
            // --------------------------------------------------------------------
            frametime = (frame / 60.0f); // 60 FPS animation
            // frametime = 0;
            // Set the skeleton's current pose matrices from animation data


            // Process FTE animation events elapsed between the last and current frame
            // process_anim_events( iqm_model, framegroup_idx, prev_frametime, cur_frametime, event_callback);
            // --------------------------------------------------------------------



            if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_swblending_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_swblending_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_swblending_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_swblending_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_swblending_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_swblending_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_hwblending_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_hwblending_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_hwblending_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_hwblending_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_hwblending_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWBLENDING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_hwblending_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose_unindexed_float32( iqm_skeleton, iqm_model);
                draw_skeletal_model_unindexed_swskinning_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose( iqm_skeleton, iqm_model);
                draw_skeletal_model_indexed_swskinning_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose_unindexed_int16( iqm_skeleton, iqm_model);
                draw_skeletal_model_unindexed_swskinning_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose_int16( iqm_skeleton, iqm_model);
                draw_skeletal_model_indexed_swskinning_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose_unindexed_int8( iqm_skeleton, iqm_model);
                draw_skeletal_model_unindexed_swskinning_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_SWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                apply_skeleton_pose_int8( iqm_skeleton, iqm_model);
                draw_skeletal_model_indexed_swskinning_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_static_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_static_float32(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_static_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_static_int16(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO)   {
                draw_skeletal_model_unindexed_static_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_STATIC      &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES)  {
                draw_skeletal_model_indexed_static_int8(iqm_model);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_float32_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_float32_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_float32_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_float32_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_float32_int8weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_FLOAT32  &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_float32_int8weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int16_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int16_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int16_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int16_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int16_int8weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT16    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int16_int8weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int8_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int8_float32weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int8_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int8_int16weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_NO    &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)   {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_unindexed_hwskinning_int8_int8weights(iqm_model, iqm_skeleton);
            }
            else if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING  &&  draw_mode_vertex_size == DRAW_MODE_VERTEX_SIZE_INT8    &&  draw_mode_indexed == DRAW_MODE_INDEXED_YES   &&  draw_mode_hwskinning_weight_size == DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8)  {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
                draw_skeletal_model_indexed_hwskinning_int8_int8weights(iqm_model, iqm_skeleton);
            }
            // --------------------------------------------------------------------
        }


        sceGuFinish();


        // Draw debug screen on top of the current draw buffer
        // Set location in VRAM to draw debugscreen to:
        pspDebugScreenSetOffset((int) cur_draw_buffer);
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        // pspDebugScreenSetOffset((int) display_buffer);


        // --------------------------
        const char *draw_mode_animation_button = "X";
        const char *draw_mode_vertex_size_button = "[]";
        const char *draw_mode_hwskinning_weight_size_button = "/\\";
        const char *draw_mode_indexed_button = "()";


        pspDebugScreenSetXY(1,4);
        switch(draw_mode_animation) {
            case DRAW_MODE_ANIMATION_SWBLENDING:
                pspDebugScreenPrintf("%s Animation mode: Software blending (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
            case DRAW_MODE_ANIMATION_HWBLENDING:
                pspDebugScreenPrintf("%s Animation mode: Hardware blending (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
            case DRAW_MODE_ANIMATION_SWSKINNING:
                pspDebugScreenPrintf("%s Animation mode: Software skinning (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
            case DRAW_MODE_ANIMATION_HWSKINNING:
                pspDebugScreenPrintf("%s Animation mode: Hardware skinning (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
            case DRAW_MODE_ANIMATION_STATIC:
                pspDebugScreenPrintf("%s Animation mode: Static (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
            default:
                pspDebugScreenPrintf("%s Animation mode: ??? (%d)", draw_mode_animation_button, draw_mode_animation);
                break;
        }


        pspDebugScreenSetXY(1,5);
        switch(draw_mode_vertex_size) {
            case DRAW_MODE_VERTEX_SIZE_FLOAT32:
                pspDebugScreenPrintf("%s Vertex Struct Size: float32 (%d)", draw_mode_vertex_size_button, draw_mode_vertex_size);
                break;
            case DRAW_MODE_VERTEX_SIZE_INT16:
                pspDebugScreenPrintf("%s Vertex Struct Size: int16 (%d)", draw_mode_vertex_size_button, draw_mode_vertex_size);
                break;
            case DRAW_MODE_VERTEX_SIZE_INT8:
                pspDebugScreenPrintf("%s Vertex Struct Size: int8 (%d)", draw_mode_vertex_size_button, draw_mode_vertex_size);
                break;
            default:
                pspDebugScreenPrintf("%s Vertex Struct Size: ??? (%d)", draw_mode_vertex_size_button, draw_mode_vertex_size);
                break;
        }

        pspDebugScreenSetXY(1,6);
        switch(draw_mode_indexed) {
            case DRAW_MODE_INDEXED_NO:
                pspDebugScreenPrintf("%s Draw mode indexed? No (%d)", draw_mode_indexed_button, draw_mode_indexed);
                break;
            case DRAW_MODE_INDEXED_YES:
                pspDebugScreenPrintf("%s Draw mode indexed? Yes (%d)", draw_mode_indexed_button, draw_mode_indexed);
                break;
            default:
                pspDebugScreenPrintf("%s Draw mode indexed? ??? (%d)", draw_mode_indexed_button, draw_mode_indexed);
                break;
        }

        if(draw_mode_animation == DRAW_MODE_ANIMATION_HWSKINNING) {
            pspDebugScreenSetXY(1,7);
            switch(draw_mode_hwskinning_weight_size) {
                case DRAW_MODE_HWSKINNING_WEIGHT_SIZE_FLOAT32:
                    pspDebugScreenPrintf("%s Hardware Skinning Weight struct: float32 (%d)", draw_mode_hwskinning_weight_size_button, draw_mode_hwskinning_weight_size);
                    break;
                case DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT16:
                    pspDebugScreenPrintf("%s Hardware Skinning Weight struct: int16 (%d)", draw_mode_hwskinning_weight_size_button, draw_mode_hwskinning_weight_size);
                    break;
                case DRAW_MODE_HWSKINNING_WEIGHT_SIZE_INT8:
                    pspDebugScreenPrintf("%s Hardware Skinning Weight struct: int8 (%d)", draw_mode_hwskinning_weight_size_button, draw_mode_hwskinning_weight_size);
                    break;
                default:
                    pspDebugScreenPrintf("%s Hardware Skinning Weight struct: ??? (%d)", draw_mode_hwskinning_weight_size_button, draw_mode_hwskinning_weight_size);
                    break;
            }
        }


        // // pspDebugScreenSetXY(1,3);
        // switch(draw_mode) {
        //     case DRAW_MODE_SOFTWARE_SKINNING:
        //         pspDebugScreenPrintf("Draw mode: Software Skinning (%d)", draw_mode);
        //         break;
        //     case DRAW_MODE_HARDWARE_SKINNING:
        //         pspDebugScreenPrintf("Draw mode: Hardware Skinning (%d)", draw_mode);
        //         break;
        //     case DRAW_MODE_STATIC:
        //         pspDebugScreenPrintf("Draw mode: Static (%d)", draw_mode);
        //         break;
        //     case DRAW_MODE_STATIC_16B:
        //         pspDebugScreenPrintf("Draw mode: Static 16-bit (%d)", draw_mode);
        //         break;
        //     case DRAW_MODE_STATIC_SOFTWARE_BLEND:
        //         pspDebugScreenPrintf("Draw mode: Software Blending (%d)", draw_mode);
        //         break;
        //     case DRAW_MODE_STATIC_HARDWARE_BLEND:
        //         pspDebugScreenPrintf("Draw mode: Hardware Blending (%d)", draw_mode);
        //         break;
        //     default:
        //         pspDebugScreenPrintf("Draw mode: Unknown (%d)", draw_mode);
        //         break;
        // }



        pspDebugScreenSetXY(1,2);
        // pspDebugScreenPrintf("hello");
        double cur_epoch_time = get_epoch_time();

        // --- Average FPS over the last `FPS_FRAMES_WINDOW` frames ---
        int cur_frametime_idx = (frame) % FPS_FRAMES_WINDOW;
        int oldest_frametime_idx = (frame + 1) % FPS_FRAMES_WINDOW;
        fps_frametimes[cur_frametime_idx] = cur_epoch_time;
        double elapsed_time = fps_frametimes[cur_frametime_idx] - fps_frametimes[oldest_frametime_idx];
        double cur_fps =  (FPS_FRAMES_WINDOW - 1) / elapsed_time;
        if(peak_fps_delay > 0) {
            peak_fps_delay -= 1;
        }
        else {
            peak_fps = std::max(peak_fps, cur_fps);
        }
        // pspDebugScreenPrintf("FPS: %.2f", cur_fps);
        
        // --- Overall average FPS ---
        // double cur_epoch_time = get_epoch_time();
        double overall_fps = (frame) / (cur_epoch_time - start_epoch_time); 
        // double cur_fps = (frame) / (cur_epoch_time - start_epoch_time);
        // pspDebugScreenPrintf("FPS: %.2f", cur_fps);

        pspDebugScreenPrintf("FPS: %.2f (last %d frames), %.2f (overall), %.2f (peak)", cur_fps, FPS_FRAMES_WINDOW, overall_fps, peak_fps);

        


        // int vert_idx = frame % (iqm_model->meshes[mesh_idx].n_verts);
        // pspDebugScreenPrintf("mesh[%d] n_tris: %ld, n_verts: %ld", mesh_idx, iqm_model->meshes[mesh_idx].n_tris, iqm_model->meshes[mesh_idx].n_verts);
        // pspDebugScreenSetXY(1,4);
        // pspDebugScreenPrintf("vertex[%d]: (%.3f,%.3f,%.3f) (%.3f, %.3f)", vert_idx, iqm_model->meshes[mesh_idx].verts[vert_idx].x,iqm_model->meshes[mesh_idx].verts[vert_idx].y,iqm_model->meshes[mesh_idx].verts[vert_idx].z,iqm_model->meshes[mesh_idx].verts[vert_idx].u,iqm_model->meshes[mesh_idx].verts[vert_idx].v);

        // pspDebugScreenSetXY(40,3);  
        // pspDebugScreenPrintf("IQM Version: %d", iqm_header->version);
        // pspDebugScreenSetXY(40,4);
        // pspDebugScreenPrintf("IQM n_meshes: %d", iqm_header->n_meshes);
        // pspDebugScreenSetXY(40,5);
        // pspDebugScreenPrintf("IQM n_verts: %d", iqm_header->n_verts);

        // pspDebugScreenPrintf("Frame: \"%d\"", frame);
        // pspDebugScreenSetXY(40,5);

        // texture_t *debug_tex = zombie_tex;
        // texture_t *debug_tex = eyeglow_tex;
        // pspDebugScreenPrintf("tex size: (%dx%d)", debug_tex->width, debug_tex->height);
        // int n_pixels = debug_tex->width * debug_tex->height;
        // int pixel_idx = frame % n_pixels;
        // uint8_t *px_data = &((uint8_t*) (debug_tex->data))[pixel_idx * 4];
        // pspDebugScreenSetXY(20,6);
        // pspDebugScreenPrintf("tex pixel %d = RGBA(%d %d %d %d)", pixel_idx, px_data[0], px_data[1], px_data[2], px_data[3]);


        // sceDisplayWaitVblankStart();
        cur_draw_buffer = sceGuSwapBuffers();
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        frame += 1;




        // ---------------------------------------------------
        // Input handling
        // ---------------------------------------------------
        bool reset_fps = false;

        sceCtrlReadBufferPositive(&cur_input, 1);
        if(prev_input_buttons != cur_input.Buttons) {
            if(cur_input.Buttons & PSP_CTRL_CROSS && !(prev_input_buttons & PSP_CTRL_CROSS)) {
                draw_mode_animation = (draw_mode_animation + 1) % N_DRAW_MODES_ANIMATION;
                reset_fps = true;
            }
            if(cur_input.Buttons & PSP_CTRL_SQUARE && !(prev_input_buttons & PSP_CTRL_SQUARE)) {
                draw_mode_vertex_size = (draw_mode_vertex_size + 1) % N_DRAW_MODES_VERTEX_SIZE;
                reset_fps = true;
            }
            if(cur_input.Buttons & PSP_CTRL_TRIANGLE && !(prev_input_buttons & PSP_CTRL_TRIANGLE)) {
                draw_mode_hwskinning_weight_size = (draw_mode_hwskinning_weight_size + 1) % N_DRAW_MODES_HWSKINNING_WEIGHT_SIZE;
                reset_fps = true;
            }
            if(cur_input.Buttons & PSP_CTRL_CIRCLE && !(prev_input_buttons & PSP_CTRL_CIRCLE)) {
                draw_mode_indexed = (draw_mode_indexed + 1) % N_DRAW_MODES_INDEXED;
                reset_fps = true;
            }

            // if(cur_input.Buttons & PSP_CTRL_LTRIGGER && !(prev_input_buttons & PSP_CTRL_LTRIGGER)) {
            //     // Go back a draw mode
            //     draw_mode = ((draw_mode - 1) + N_DRAW_MODES) % N_DRAW_MODES;
            // }
            // if(cur_input.Buttons & PSP_CTRL_RTRIGGER && !(prev_input_buttons & PSP_CTRL_RTRIGGER)) {
            //     // Advance to next draw mode
            //     draw_mode = (draw_mode + 1) % N_DRAW_MODES;
            // }
        }
        prev_input_buttons = cur_input.Buttons;

        if(reset_fps) {
            for(int i = 0; i < FPS_FRAMES_WINDOW; i++) {
                double cur_epoch_time = get_epoch_time();
                fps_frametimes[i] = cur_epoch_time;
                peak_fps = 0.0;
                peak_fps_delay = 60; // Delay measurement of peak_fps by 60 frames
            }
        }

        // ---------------------------------------------------
    }

    sceGuTerm();
    sceKernelExitGame();
    return 0;
}