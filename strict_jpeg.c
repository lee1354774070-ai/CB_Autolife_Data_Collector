/* Private, warning-fatal libjpeg adapter. Build: cc -O2 -fPIC -shared
 * -Wall -Wextra -Werror strict_jpeg.c -o libstrictjpeg.so -ljpeg
 * No libjpeg error or longjmp crosses the Python/ctypes boundary. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <setjmp.h>
#include <jpeglib.h>

struct decode_context {
    struct jpeg_decompress_struct jpeg;
    struct jpeg_error_mgr error;
    jmp_buf jump;
    int created;
    char message[JMSG_LENGTH_MAX];
};

static void fail(j_common_ptr jpeg) {
    struct decode_context *ctx = (struct decode_context *)jpeg;
    (*jpeg->err->format_message)(jpeg, ctx->message);
    longjmp(ctx->jump, 1);
}

static void message(j_common_ptr jpeg, int level) {
    if (level < 0) fail(jpeg);  /* Never conceal libjpeg's corruption recovery. */
}

int strict_jpeg_decode(const unsigned char *input, unsigned long size,
                       unsigned int width, unsigned int height,
                       unsigned char *output, size_t output_size,
                       char *error, size_t error_size) {
    struct decode_context *ctx = calloc(1, sizeof(*ctx));
    volatile int result = -1;
    if (!ctx) {
        snprintf(error, error_size, "JPEG context allocation failed");
        return -1;
    }
    ctx->jpeg.err = jpeg_std_error(&ctx->error);
    ctx->error.error_exit = fail;
    ctx->error.emit_message = message;
    if (setjmp(ctx->jump)) goto cleanup;
    ctx->created = 1;
    jpeg_create_decompress(&ctx->jpeg);
    jpeg_mem_src(&ctx->jpeg, input, size);
    if (jpeg_read_header(&ctx->jpeg, TRUE) != JPEG_HEADER_OK) goto cleanup;
    if (ctx->jpeg.image_width != width || ctx->jpeg.image_height != height ||
        !width || !height || width > 8192 || height > 8192 ||
        (uint64_t)width * height * 3 != output_size) {
        snprintf(ctx->message, sizeof(ctx->message), "JPEG dimensions differ from SHM or exceed bounds");
        goto cleanup;
    }
    ctx->jpeg.out_color_space = JCS_RGB;
    ctx->jpeg.dct_method = JDCT_ISLOW;
    ctx->jpeg.do_fancy_upsampling = TRUE;
    if (!jpeg_start_decompress(&ctx->jpeg) || ctx->jpeg.output_components != 3)
        goto cleanup;
    while (ctx->jpeg.output_scanline < height) {
        JSAMPROW row = output + (size_t)ctx->jpeg.output_scanline * width * 3;
        if (jpeg_read_scanlines(&ctx->jpeg, &row, 1) != 1) goto cleanup;
    }
    if (!jpeg_finish_decompress(&ctx->jpeg)) goto cleanup;
    result = 0;
cleanup:
    if (result) snprintf(error, error_size, "%s", ctx->message[0] ? ctx->message : "Incomplete JPEG decode");
    if (ctx->created) jpeg_destroy_decompress(&ctx->jpeg);
    free(ctx);
    return result;
}
