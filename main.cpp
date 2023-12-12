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
static unsigned int __attribute__((aligned(16))) display_list[524288];


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
    tspec.tv_sec;
    tspec.tv_nsec;
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


void draw_skeletal_model_indexed_hwskinning_float32(skeletal_model_t *model, skeletal_skeleton_t *skeleton) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        for(int submesh_idx = 0; submesh_idx < mesh->n_submeshes; submesh_idx++) {
            skeletal_mesh_t *submesh = &(mesh->submeshes[submesh_idx]);
            for(int submesh_bone_idx = 0; submesh_bone_idx < submesh->n_skinning_bones; submesh_bone_idx++) {
                // Get the index into the skeleton list of bones:
                int bone_idx = submesh->skinning_bone_idxs[submesh_bone_idx];
                mat3x4_t *bone_mat3x4 = &(skeleton->bone_transforms[bone_idx]);

                // Translate the mat3x4_t bone transform matrix to ScePspFMatrix4
                ScePspFMatrix4 bone_mat;

                // ScePspFMatrix4 Internal layout:
                // [ m.x.x  m.y.x  m.z.x  m.w.x ]
                // [ m.x.y  m.y.y  m.z.y  m.w.y ]
                // [ m.x.z  m.y.z  m.z.z  m.w.z ]
                // [ m.x.w  m.y.w  m.z.w  m.w.w ]

                // Identity Matrix
                // bone_mat.x.x = 1.0f; bone_mat.y.x = 0.0f; bone_mat.z.x = 0.0f; bone_mat.w.x = 0.0f;
                // bone_mat.x.y = 0.0f; bone_mat.y.y = 1.0f; bone_mat.z.y = 0.0f; bone_mat.w.y = 0.0f;
                // bone_mat.x.z = 0.0f; bone_mat.y.z = 0.0f; bone_mat.z.z = 1.0f; bone_mat.w.z = 0.0f;
                // bone_mat.x.w = 0.0f; bone_mat.y.w = 0.0f; bone_mat.z.w = 0.0f; bone_mat.w.w = 1.0f;

                // Pose Matrix
                bone_mat.x.x = bone_mat3x4->m[0];   bone_mat.y.x = bone_mat3x4->m[3];   bone_mat.z.x = bone_mat3x4->m[6];   bone_mat.w.x = bone_mat3x4->m[9];
                bone_mat.x.y = bone_mat3x4->m[1];   bone_mat.y.y = bone_mat3x4->m[4];   bone_mat.z.y = bone_mat3x4->m[7];   bone_mat.w.y = bone_mat3x4->m[10];
                bone_mat.x.z = bone_mat3x4->m[2];   bone_mat.y.z = bone_mat3x4->m[5];   bone_mat.z.z = bone_mat3x4->m[8];   bone_mat.w.z = bone_mat3x4->m[11];
                bone_mat.x.w = 0.0f;                bone_mat.y.w = 0.0f;                bone_mat.z.w = 0.0f;                bone_mat.w.w = 1.0f;
                sceGuBoneMatrix(submesh_bone_idx, &bone_mat);
            }
            
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


void draw_skeletal_model_indexed_swskinning_int16() {

}


void draw_skeletal_model_indexed_hwskinning_int16() {

}


void draw_skeletal_model_indexed_swskinning_int16_int8weights() {

}


void draw_skeletal_model_indexed_hwskinning_int16_int8weights() {

}


void draw_skeletal_model_indexed_swblending_float32(skeletal_model_t *model) {
    for(unsigned int i = 0; i < model->n_meshes; i++) {
        skeletal_mesh_t *mesh = &(model->meshes[i]);
        // Fake software blending
        for(int j = 0; j < mesh->n_verts; j++) {
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
        for(int j = 0; j < mesh->n_verts; j++) {
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

}


void draw_skeletal_model_indexed_hwblending_int16() {

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
        sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_16BIT|GU_NORMAL_16BIT|GU_VERTEX_16BIT|GU_TRANSFORM_3D, mesh->n_tris * 3, mesh->tri_verts, mesh->vert16s);
    }
}


void draw_skeletal_model_unindexed_swskinning_float32() {

}


void draw_skeletal_model_unindexed_hwskinning_float32() {

}


void draw_skeletal_model_unindexed_swskinning_int16() {

}


void draw_skeletal_model_unindexed_hwskinning_int16() {

}


void draw_skeletal_model_unindexed_swskinning_int16_int8weights() {

}


void draw_skeletal_model_unindexed_hwskinning_int16_int8weights() {

}


void draw_skeletal_model_unindexed_swblending_float32() {

}


void draw_skeletal_model_unindexed_hwblending_float32() {

}


void draw_skeletal_model_unindexed_swblending_int16() {

}


void draw_skeletal_model_unindexed_hwblending_int16() {

}


void draw_skeletal_model_unindexed_static_float32() {

}


void draw_skeletal_model_unindexed_static_int16() {

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


    const int DRAW_MODE_SOFTWARE_SKINNING = 0;
    const int DRAW_MODE_HARDWARE_SKINNING = 1;
    const int DRAW_MODE_STATIC = 2;
    const int DRAW_MODE_STATIC_16B = 3;
    const int DRAW_MODE_STATIC_SOFTWARE_BLEND = 4;
    const int DRAW_MODE_STATIC_HARDWARE_BLEND = 5;
    // const int DRAW_MODE_STATIC_16B_SOFTWARE_BLEND = 6;
    // const int DRAW_MODE_STATIC_16B_HARDWARE_BLEND = 7;
    const int N_DRAW_MODES = 6;
    int draw_mode = DRAW_MODE_SOFTWARE_SKINNING;
    // int draw_mode = DRAW_MODE_HARDWARE_SKINNING;
    // int draw_mode = DRAW_MODE_STATIC;
    // int draw_mode = DRAW_MODE_STATIC_SOFTWARE_BLEND;
    // int draw_mode = DRAW_MODE_STATIC_HARDWARE_BLEND;

    // TODO - Replace with these options:
    // Options:
    //      animation: sw blending, hw blending, hw skinning, sw skinning, static
    //      vertex struct size: float32, int16
    //      skinning weight struct size: int16, int8
    //      indexing: yes, no




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
            ScePspFVector3 model_rot = {-90 * DEG2RAD, 0, (frame * rot_speed + -90) * DEG2RAD};
            // ScePspFVector3 model_rot = {-90 * (GU_PI/180.0f), 0, (-90) * (GU_PI/180.0f)};
            // float scale = (sin((float)frame / 10.0f) + 1.0f) * 100;
            ScePspFVector3 model_scale = {scale,scale,scale};


            sceGumTranslate(&model_pos);
            sceGumScale(&model_scale);
            sceGumRotateXYZ(&model_rot);

            if(draw_mode == DRAW_MODE_STATIC_16B) {
                // Undo the float32 -> int16 scaling
                // Only do it once for all meshes
                float undo_scale = 1.0f / iqm_model->meshes[0].verts_unit_scale;
                ScePspFVector3 undo_scale_vec3 = {undo_scale,undo_scale,undo_scale};
                sceGumScale(&undo_scale_vec3);
            }

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


            // build_skeleton( iqm_skeleton, iqm_model, framegroup_idx, frametime);
            if(draw_mode == DRAW_MODE_HARDWARE_SKINNING || draw_mode == DRAW_MODE_SOFTWARE_SKINNING) {
                build_skeleton( iqm_skeleton, iqm_anim, framegroup_idx, frametime);
            }
            // Process FTE animation events elapsed between the last and current frame
            // process_anim_events( iqm_model, framegroup_idx, prev_frametime, cur_frametime, event_callback);
            if(draw_mode == DRAW_MODE_SOFTWARE_SKINNING) {
                apply_skeleton_pose( iqm_skeleton, iqm_model);
            }
            // Transform the model vertices to model-space using the skeleton's current pose matrices
            // --------------------------------------------------------------------





            // for(unsigned int i = 0; i < iqm_model->n_submeshes; i++) {
            unsigned int mesh_idx = ((unsigned int)((frame / 10.0f)) % iqm_model->n_meshes);


            // Let's list all of the draw mode functions we need...
            // void draw_skeletal_model_indexed_swskinning_float32();
            // void draw_skeletal_model_indexed_hwskinning_float32();
            // void draw_skeletal_model_indexed_swskinning_int16();
            // void draw_skeletal_model_indexed_hwskinning_int16();
            // void draw_skeletal_model_indexed_swskinning_int16_int8weights();
            // void draw_skeletal_model_indexed_hwskinning_int16_int8weights();
            // void draw_skeletal_model_indexed_swblending_float32();
            // void draw_skeletal_model_indexed_hwblending_float32();
            // void draw_skeletal_model_indexed_swblending_int16();
            // void draw_skeletal_model_indexed_hwblending_int16();
            // void draw_skeletal_model_indexed_static_float32();
            // void draw_skeletal_model_indexed_static_int16();
            // void draw_skeletal_model_unindexed_swskinning_float32();
            // void draw_skeletal_model_unindexed_hwskinning_float32();
            // void draw_skeletal_model_unindexed_swskinning_int16();
            // void draw_skeletal_model_unindexed_hwskinning_int16();
            // void draw_skeletal_model_unindexed_swskinning_int16_int8weights();
            // void draw_skeletal_model_unindexed_hwskinning_int16_int8weights();
            // void draw_skeletal_model_unindexed_swblending_float32();
            // void draw_skeletal_model_unindexed_hwblending_float32();
            // void draw_skeletal_model_unindexed_swblending_int16();
            // void draw_skeletal_model_unindexed_hwblending_int16();
            // void draw_skeletal_model_unindexed_static_float32();
            // void draw_skeletal_model_unindexed_static_int16();

            
            // Options:
            //      animation: sw blending, hw blending, hw skinning, sw skinning, static
            //      vertex struct size: float32, int16
            //      skinning weight struct size: int16, int8
            //      indexing: yes, no

            if(draw_mode == DRAW_MODE_SOFTWARE_SKINNING) {
                draw_skeletal_model_indexed_swskinning_float32(iqm_model);
            }
            else if(draw_mode == DRAW_MODE_HARDWARE_SKINNING) {
                draw_skeletal_model_indexed_hwskinning_float32(iqm_model, iqm_skeleton);
            }
            else if(draw_mode == DRAW_MODE_STATIC_SOFTWARE_BLEND) {
                draw_skeletal_model_indexed_swblending_float32(iqm_model);
            }
            else if(draw_mode == DRAW_MODE_STATIC_HARDWARE_BLEND) {
                draw_skeletal_model_indexed_hwblending_float32(iqm_model);
            }
            else if(draw_mode == DRAW_MODE_STATIC) {
                draw_skeletal_model_indexed_static_float32(iqm_model);
            }
            else if(draw_mode == DRAW_MODE_STATIC_16B) {
                draw_skeletal_model_indexed_static_int16(iqm_model);
            }

            // --------------------------------------------------------------------
        }


        sceGuFinish();


        // Draw debug screen on top of the current draw buffer
        // Set location in VRAM to draw debugscreen to:
        pspDebugScreenSetOffset((int) cur_draw_buffer);
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        // pspDebugScreenSetOffset((int) display_buffer);

        pspDebugScreenSetXY(1,3);
        switch(draw_mode) {
            case DRAW_MODE_SOFTWARE_SKINNING:
                pspDebugScreenPrintf("Draw mode: Software Skinning (%d)", draw_mode);
                break;
            case DRAW_MODE_HARDWARE_SKINNING:
                pspDebugScreenPrintf("Draw mode: Hardware Skinning (%d)", draw_mode);
                break;
            case DRAW_MODE_STATIC:
                pspDebugScreenPrintf("Draw mode: Static (%d)", draw_mode);
                break;
            case DRAW_MODE_STATIC_16B:
                pspDebugScreenPrintf("Draw mode: Static 16-bit (%d)", draw_mode);
                break;
            case DRAW_MODE_STATIC_SOFTWARE_BLEND:
                pspDebugScreenPrintf("Draw mode: Software Blending (%d)", draw_mode);
                break;
            case DRAW_MODE_STATIC_HARDWARE_BLEND:
                pspDebugScreenPrintf("Draw mode: Hardware Blending (%d)", draw_mode);
                break;
            default:
                pspDebugScreenPrintf("Draw mode: Unknown (%d)", draw_mode);
                break;
        }



        pspDebugScreenSetXY(1,5);
        // pspDebugScreenPrintf("hello");
        double cur_epoch_time = get_epoch_time();

        // --- Average FPS over the last `FPS_FRAMES_WINDOW` frames ---
        int cur_frametime_idx = (frame) % FPS_FRAMES_WINDOW;
        int oldest_frametime_idx = (frame + 1) % FPS_FRAMES_WINDOW;
        fps_frametimes[cur_frametime_idx] = cur_epoch_time;
        double elapsed_time = fps_frametimes[cur_frametime_idx] - fps_frametimes[oldest_frametime_idx];
        double cur_fps =  (FPS_FRAMES_WINDOW - 1) / elapsed_time;
        // pspDebugScreenPrintf("FPS: %.2f", cur_fps);
        
        // --- Overall average FPS ---
        // double cur_epoch_time = get_epoch_time();
        double overall_fps = (frame) / (cur_epoch_time - start_epoch_time); 
        // double cur_fps = (frame) / (cur_epoch_time - start_epoch_time);
        // pspDebugScreenPrintf("FPS: %.2f", cur_fps);

        pspDebugScreenPrintf("FPS: %.2f (last %d frames), %.2f (overall)", cur_fps, FPS_FRAMES_WINDOW, overall_fps);

        


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
        sceCtrlReadBufferPositive(&cur_input, 1);
        if(prev_input_buttons != cur_input.Buttons) {
            if(cur_input.Buttons & PSP_CTRL_LTRIGGER && !(prev_input_buttons & PSP_CTRL_LTRIGGER)) {
                // Go back a draw mode
                draw_mode = ((draw_mode - 1) + N_DRAW_MODES) % N_DRAW_MODES;
            }
            if(cur_input.Buttons & PSP_CTRL_RTRIGGER && !(prev_input_buttons & PSP_CTRL_RTRIGGER)) {
                // Advance to next draw mode
                draw_mode = (draw_mode + 1) % N_DRAW_MODES;
            }
        }
        prev_input_buttons = cur_input.Buttons;
        // ---------------------------------------------------
    }

    sceGuTerm();
    sceKernelExitGame();
    return 0;
}