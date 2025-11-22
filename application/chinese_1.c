/*******************************************************************************
 * Size: 19 px
 * Bpp: 2
 * Opts: --bpp 2 --size 19 --no-compress --stride 1 --align 1 --font SourceHanSans-Bold.ttf --symbols 7S后开始侦测设备ID:0123456789ABCDEF  --format lvgl -o chinese.c
 ******************************************************************************/

#define LV_LVGL_H_INCLUDE_SIMPLE

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif



#ifndef CHINESE
#define CHINESE 1
#endif

#if CHINESE



/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+002D "-" */
    0x7c, 0x0,

    /* U+0030 "0" */
    0x1f, 0x43, 0x6c, 0x70, 0xdb, 0xd, 0xb0, 0xd7,
    0xd, 0x36, 0xc1, 0xf4,

    /* U+0031 "1" */
    0x1f, 0x2, 0xf0, 0xb, 0x0, 0xb0, 0xb, 0x0,
    0xb0, 0x1b, 0x43, 0xfd,

    /* U+0032 "2" */
    0x2f, 0x6, 0x7c, 0x1, 0xc0, 0x2c, 0x3, 0x40,
    0xe0, 0x3d, 0x4b, 0xfd,

    /* U+0033 "3" */
    0x2f, 0x42, 0x6c, 0x2, 0xc0, 0xf4, 0x2, 0xc0,
    0xd, 0x66, 0xc3, 0xf4,

    /* U+0034 "4" */
    0x7, 0x80, 0xf8, 0x1e, 0x83, 0x68, 0x72, 0x8f,
    0xfe, 0x2, 0xc0, 0x28,

    /* U+0035 "5" */
    0x3f, 0xc3, 0x94, 0x30, 0x3, 0xf4, 0x12, 0xc0,
    0xd, 0x66, 0xc3, 0xf4,

    /* U+0036 "6" */
    0xf, 0x83, 0x98, 0x70, 0x7, 0xb8, 0x75, 0xd7,
    0xd, 0x39, 0xd1, 0xf8,

    /* U+0037 "7" */
    0xbf, 0xd1, 0x6c, 0x3, 0x40, 0x70, 0xa, 0x0,
    0xe0, 0xd, 0x0, 0xd0,

    /* U+0038 "8" */
    0x1f, 0x43, 0x5c, 0x30, 0xc2, 0xf8, 0x3b, 0xc7,
    0xd, 0x71, 0xd2, 0xf8,

    /* U+0039 "9" */
    0x2f, 0x7, 0x6c, 0xb0, 0xc7, 0x1d, 0x3e, 0xd0,
    0xc, 0x26, 0xc3, 0xe0,

    /* U+003A ":" */
    0x34, 0xd0, 0x0, 0x34, 0xd0,

    /* U+0041 "A" */
    0x3, 0xc0, 0x3, 0xe0, 0x7, 0xb0, 0xa, 0x70,
    0xd, 0x34, 0x1f, 0xf8, 0x2c, 0x2c, 0x38, 0xd,

    /* U+0042 "B" */
    0x3f, 0xd0, 0xe2, 0xc3, 0x4b, 0xf, 0xf4, 0x34,
    0xb0, 0xd0, 0xd3, 0x8b, 0xf, 0xf4,

    /* U+0043 "C" */
    0xb, 0xd0, 0xb5, 0x83, 0x40, 0x1c, 0x0, 0x70,
    0x0, 0xd0, 0x3, 0xd7, 0x2, 0xf8,

    /* U+0044 "D" */
    0x3f, 0x80, 0xe7, 0xc3, 0x43, 0x4d, 0xe, 0x34,
    0x38, 0xd0, 0xd3, 0x9f, 0xf, 0xe0,

    /* U+0045 "E" */
    0x3f, 0xd3, 0x94, 0x34, 0x3, 0xfc, 0x38, 0x3,
    0x40, 0x39, 0x43, 0xfe,

    /* U+0046 "F" */
    0x3f, 0xd3, 0x94, 0x34, 0x3, 0x40, 0x3f, 0xc3,
    0x80, 0x34, 0x3, 0x40,

    /* U+0049 "I" */
    0x34, 0xd3, 0x4d, 0x34, 0xd3, 0x4d,

    /* U+5907 "备" */
    0x1, 0x80, 0x0, 0x1f, 0xfe, 0x2, 0xf4, 0xb0,
    0x4, 0xbf, 0x40, 0x7f, 0x9b, 0xf0, 0xba, 0xab,
    0x41, 0xdb, 0x6c, 0x7, 0xff, 0xf0, 0x1c, 0x71,
    0xc0, 0x7f, 0xff, 0x0,

    /* U+8BBE "设" */
    0x0, 0x0, 0x1, 0x83, 0xf4, 0x9, 0x33, 0x41,
    0x4a, 0x39, 0x7c, 0x81, 0x80, 0xcf, 0xfc, 0xc,
    0x72, 0x80, 0xf2, 0xf0, 0x1d, 0x7f, 0x80, 0x19,
    0x18
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 36, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 59, .box_w = 4, .box_h = 2, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 2, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 14, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 26, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 38, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 50, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 62, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 74, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 86, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 98, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 110, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 122, .adv_w = 52, .box_w = 3, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 127, .adv_w = 103, .box_w = 8, .box_h = 8, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 143, .adv_w = 109, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 157, .adv_w = 105, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 171, .adv_w = 114, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 185, .adv_w = 98, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 197, .adv_w = 94, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 209, .adv_w = 53, .box_w = 3, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 215, .adv_w = 160, .box_w = 11, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 243, .adv_w = 160, .box_w = 10, .box_h = 10, .ofs_x = 0, .ofs_y = -1}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0xd, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
    0x16, 0x17, 0x18, 0x19, 0x1a, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x29, 0x58e7, 0x8b9e
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 35743, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 22, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 2,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif

};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t chinese_1 = {
#else
lv_font_t chinese = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 10,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    //.static_bitmap = 0,
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};




#endif /*#if CHINESE*/



