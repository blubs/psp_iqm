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
#include "png.h"



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
    sceGuSync(GU_SYNC_WHAT_DONE,0);
    // Wait for vertical blank start
    sceDisplayWaitVblankStart();
    // Turn display on
    sceGuDisplay(GU_TRUE);
}



typedef struct texture_s {
    int width;
    int height;
    void *data; // Pointer to RGBA 8888 texture data (4 * width * height) bytes
} texture_t;


// https://gist.github.com/niw/5963798
int load_png_file(char *file, texture_t *tex) {
    if(tex == NULL) {
        return -1;
    }
    if(tex->data != NULL) {
        return -2;
    }
    FILE *fp = fopen(file, "rb");
    png_structp png_read_struct = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png_read_struct) {
        return -3;
    }

    png_infop png_info = png_create_info_struct(png_read_struct);
    if(!png_info) {
        return -4;
    }
    if(setjmp(png_jmpbuf(png_read_struct))) {
        return -5;
    }
    png_init_io(png_read_struct, fp);
    png_read_info(png_read_struct, png_info);
    int img_width = png_get_image_width(png_read_struct, png_info);
    int img_height = png_get_image_height(png_read_struct, png_info);
    int img_color_type = png_get_color_type(png_read_struct, png_info);
    int img_bit_depth = png_get_bit_depth(png_read_struct, png_info);

    // Convert color type to RGBA 8888 format
    if(img_bit_depth == 16) {
        png_set_strip_16(png_read_struct);
    }
    if(img_color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_read_struct);
    }
    if(img_color_type == PNG_COLOR_TYPE_GRAY && img_bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(png_read_struct);
    }
    if(png_get_valid(png_read_struct, png_info, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha(png_read_struct);
    }
    // Add alpha channel to image formats that don't have one
    if(img_color_type == PNG_COLOR_TYPE_RGB || img_color_type == PNG_COLOR_TYPE_GRAY || img_color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_filter(png_read_struct, 0xFF, PNG_FILLER_AFTER);
    }
    if(img_color_type == PNG_COLOR_TYPE_GRAY || img_color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_read_struct);
    }
    png_read_update_info(png_read_struct, png_info);

    // Allocate enough data to read the PNG file
    png_bytep *row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * img_height);
    for(int row_idx = 0; row_idx < img_height; row_idx++) {
        row_pointers[row_idx] = (png_byte*) malloc(png_get_rowbytes(png_read_struct,png_info));
    }
    png_read_image(png_read_struct, row_pointers);
    fclose(fp);
    
    // ------------------------------------------------------------------------
    // Copy RGBA8888 data to contiguous memory for texture
    // ------------------------------------------------------------------------
    tex->width = img_width;
    tex->height = img_width;
    tex->data = (void*) malloc(4 * img_width * img_height);

    for(int row_idx = 0; row_idx < img_height; row_idx++) {
        // Copy pixel values for the entire image row:
        size_t n_bytes = 4 * img_width;
        memcpy(tex->data + (row_idx * n_bytes), row_pointers[row_idx], n_bytes);
    }
    // ------------------------------------------------------------------------
    png_destroy_read_struct(&png_read_struct, &png_info, NULL);
    return 0;
}


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

    texture_t *zombie_tex = malloc(sizeof(texture_t));
    texture_t *eyeglow_tex = malloc(sizeof(texture_t));


    load_png_file("assets/zombie_tex_0.png", zombie_tex);
    load_png_file("assets/eyeglow_tex.png", eyeglow_tex);



    void *cur_draw_buffer = draw_buffer;
    fclose(f);
    unsigned int frame = 0;
    while(running()) {
        sceGuStart(GU_DIRECT, display_list);
        // Smoothly fade between 0 and 1:
        float fade = 0.5f * (sin((float)frame / 10.0f) + 1.0f);
        sceGuClearColor((int)(0x0000ff * fade));
        sceGuClearDepth(0);
        // sceGuClear(GU_DEPTH_BUFFER_BIT);
        // sceGuClear(GU_COLOR_BUFFER_BIT);
        sceGuClear(GU_COLOR_BUFFER_BIT | GU_DEPTH_BUFFER_BIT);
        sceGuFinish();


        // Draw debug screen on top of the current draw buffer
        // Set location in VRAM to draw debugscreen to:
        pspDebugScreenSetOffset((int) cur_draw_buffer);
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        // pspDebugScreenSetOffset((int) display_buffer);
        pspDebugScreenSetXY(40,3);
        pspDebugScreenPrintf("Reading file: \"test.txt\"");
        pspDebugScreenSetXY(40,4);
        pspDebugScreenPrintf("Frame: \"%d\"", frame);
        pspDebugScreenSetXY(40,5);
        pspDebugScreenPrintf("Zombie tex size: (%dx%d)", zombie_tex->width, zombie_tex->height);
        int n_pixels = zombie_tex->width * zombie_tex->height;
        int pixel_n = frame % n_pixels;
        unsigned char *px_data = &((unsigned char*) (zombie_tex->data))[pixel_n * 4];
        pspDebugScreenSetXY(20,6);
        pspDebugScreenPrintf("Zombie tex pixel %d = RGBA(%d %d %d %d)", pixel_n, px_data[0], px_data[1], px_data[2], px_data[3]);
        sceDisplayWaitVblankStart();
        cur_draw_buffer = sceGuSwapBuffers();
        sceGuSync(GU_SYNC_FINISH,GU_SYNC_WHAT_DONE);
        frame += 1;
    }

    sceGuTerm();
    sceKernelExitGame();
    return 0;
}