#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


#include <pspkernel.h>
#include <pspdisplay.h>
#include <pspdebug.h>
#include <pspgu.h>
#include <pspgum.h>


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
static unsigned int __attribute__((aligned(16))) display_list[262144];


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


int main(int argc, char *argv[]) {
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
    model_t *iqm_model = load_iqm_file("assets/zombie_with_anims.iqm");


    load_png_file("assets/zombie_tex_0.png", zombie_tex);
    load_png_file("assets/eyeglow_tex.png", eyeglow_tex);



    void *cur_draw_buffer = draw_buffer;
    fclose(f);
    unsigned int frame = 0;
    while(running()) {
        sceGuStart(GU_DIRECT, display_list);
        // Smoothly fade between 0 and 1:
        // float fade = 0.5f * (sin((float)frame / 10.0f) + 1.0f);
        float fade = 0;
        sceGuClearColor((int)(0x0000ff * fade));
        sceGuClearDepth(0);
        // sceGuClear(GU_DEPTH_BUFFER_BIT);
        // sceGuClear(GU_COLOR_BUFFER_BIT);
        sceGuClear(GU_COLOR_BUFFER_BIT | GU_DEPTH_BUFFER_BIT);

        // --------------------------------------------------------------------
        // Let's try drawing...
        // --------------------------------------------------------------------
        sceGumMatrixMode(GU_PROJECTION);
        sceGumLoadIdentity();
        sceGumPerspective(75.0f, 16.0f/9.0f, 0.5f, 1000.0f);
        sceGumMatrixMode(GU_VIEW); // TODO Move the camera somewhere...
        sceGumLoadIdentity();
        sceGumMatrixMode(GU_MODEL);
        sceGumLoadIdentity();
        // ScePspFVector3 model_pos = {0,-10.0f,0.0f};
        // ScePspFVector3 model_pos = {0,0,-2.5f};
        // ScePspFVector3 model_pos = {10.0f,0,0.0f};
        // ScePspFVector3 model_pos = {0,-10.0f,0.0f};
        ScePspFVector3 model_pos = {0,0,-10.0f};
        // ScePspFVector3 model_rot = {frame * 0.79f * (GU_PI/180.0f), frame * 0.98f * (GU_PI/180.0f), frame * 1.32f * (GU_PI/180.0f)};
        // ScePspFVector3 model_rot = {0, frame * 1.32f * (GU_PI/180.0f), 0};
        ScePspFVector3 model_rot = {-90 * (GU_PI/180.0f), 0, (frame * 0.32f + -90) * (GU_PI/180.0f)};
        // ScePspFVector3 model_rot = {-90 * (GU_PI/180.0f), 0, (-90) * (GU_PI/180.0f)};
        // float scale = (sin((float)frame / 10.0f) + 1.0f) * 100;
        float scale = 0.1f;
        ScePspFVector3 model_scale = {scale,scale,scale};
        sceGumTranslate(&model_pos);
        sceGumScale(&model_scale);
        sceGumRotateXYZ(&model_rot);


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




        // for(unsigned int i = 0; i < iqm_model->n_submeshes; i++) {
        unsigned int mesh_idx = ((unsigned int)((frame / 10.0f)) % iqm_model->n_meshes);
        vertex_t *model_verts = iqm_model->verts;
 
        // for(unsigned int i = mesh_idx; i < mesh_idx+1; i++) {
        for(unsigned int i = 0; i < iqm_model->n_meshes; i++) {
            uint16_t *mesh_tri_vert_idxs = iqm_model->meshes[i].tri_verts;
            unsigned int mesh_n_tri_verts = iqm_model->meshes[i].n_tri_verts;
            unsigned int mesh_n_verts = iqm_model->meshes[i].n_verts;
            unsigned int mesh_first_vert = iqm_model->meshes[i].first_vert;  
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh_n_verts, mesh_tri_vert_idxs, model_verts + mesh_first_vert);
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh_n_verts, mesh_tri_vert_idxs, model_verts + mesh_first_vert);
            sceGumDrawArray(GU_TRIANGLES,GU_INDEX_16BIT|GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh_n_tri_verts, mesh_tri_vert_idxs, model_verts + mesh_first_vert);
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh_n_verts, 0, model_verts + mesh_first_vert);
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, mesh_n_verts, 0, model_verts + mesh_first_vert);
            // sceGumDrawArray(GU_POINTS,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, submesh_n_verts, submesh_tri_vert_idxs, model_verts + submesh_first_vert);

            // Okay, so now the vertices being drawn are correct... but the indices are still wrong..


            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, submesh_n_tri_verts, submesh_tri_vert_idxs, submesh_verts);
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D, submesh_n_tri_verts, 0, submesh_verts);
            // FIXME - The issue may be that it's drawing the other vertices...

            // FIXME - The issue is that the list of VERTICES must specify the the triangles...
            // I'm not sure why the triangle indices are being ignored.


            // for(unsigned int j = 0; j < iqm_model->submeshes[i].n_tris * 3; j++) {
            //     submesh_verts[j].u = iqm_model->verts_uv[iqm_model->submeshes[i].tri_verts[j]].pos[0];
            //     submesh_verts[j].v = iqm_model->verts_uv[iqm_model->submeshes[i].tri_verts[j]].pos[1];
            //     submesh_verts[j].x = iqm_model->verts_pos[iqm_model->submeshes[i].tri_verts[j]].pos[0];
            //     submesh_verts[j].y = iqm_model->verts_pos[iqm_model->submeshes[i].tri_verts[j]].pos[1];
            //     submesh_verts[j].z = iqm_model->verts_pos[iqm_model->submeshes[i].tri_verts[j]].pos[2];
            //     submesh_verts[j].color = 0xff7f0000;
            // }
            // // FIXME - horrible memory leak
            // size_t n_floats = iqm_model->submeshes[i].n_tris * 3;
            // sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_COLOR_8888|GU_VERTEX_32BITF|GU_TRANSFORM_3D,n_floats, 0, submesh_verts);
        }
        // --------------------------------------------------------------------


        sceGuFinish();


        // Draw debug screen on top of the current draw buffer
        // Set location in VRAM to draw debugscreen to:
        pspDebugScreenSetOffset((int) cur_draw_buffer);
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        // pspDebugScreenSetOffset((int) display_buffer);


        pspDebugScreenSetXY(1,3);
        int vert_idx = frame % (iqm_model->n_verts);
        pspDebugScreenPrintf("mesh[%d] n_tris: %d, n_verts: %d, n_tri_verts: %d", mesh_idx, iqm_model->meshes[mesh_idx].n_tris, iqm_model->meshes[mesh_idx].n_verts, iqm_model->meshes[mesh_idx].n_tri_verts);
        pspDebugScreenSetXY(1,4);
        pspDebugScreenPrintf("vertex[%d]: (%.3f,%.3f,%.3f) (%.3f, %.3f)", vert_idx, iqm_model->verts[vert_idx].x,iqm_model->verts[vert_idx].y,iqm_model->verts[vert_idx].z,iqm_model->verts[vert_idx].u,iqm_model->verts[vert_idx].v);

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
        sceDisplayWaitVblankStart();
        cur_draw_buffer = sceGuSwapBuffers();
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        frame += 1;
    }

    sceGuTerm();
    sceKernelExitGame();
    return 0;
}