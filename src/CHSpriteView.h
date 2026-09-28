#pragma once
/*
 * CHSpriteView - draw packed 4 bpp sprites from microSD into CHGfx's
 * framebuffer, fast enough to animate from the card every frame.
 *
 * Originally by Simon (filmote). Optimised for the CHGame board; every
 * change is described where it is made, and the story as a whole is in
 * the header comment of CHSpriteView.cpp.
 *
 * ---------------------------------------------------------------------------
 * FILE FORMAT (unchanged, as written by the PNG -> CHGfx converter)
 * ---------------------------------------------------------------------------
 *   byte 0      width  w (pixels, 1..255)
 *   byte 1      height h (pixels, 1..255)
 *   then        h rows of ceil(w/2) bytes, 2 px per byte, even x in the
 *               LOW nibble - exactly CHGfx's framebuffer packing, which is
 *               what lets opaque sprites be copied into it byte for byte.
 *
 * ---------------------------------------------------------------------------
 * TWO WAYS TO DRAW
 * ---------------------------------------------------------------------------
 *   1. Load once, draw many (the fast one - use this for animation):
 *
 *        SpriteFile fire[16];
 *        spriteLoad(fire[i], "FIRE/FIRE_00.BIN");        // in setup()
 *        ...
 *        SpriteDirty d;  spriteDirtyReset(d);
 *        spriteDraw(fire[frame], 0, 64, -1, &d);          // in loop()
 *        spriteFlushDirty(d);                             // only what changed
 *
 *      spriteLoad() opens the file ONCE, finds where it physically lives
 *      on the card, and remembers that (20 bytes). spriteDraw() then never
 *      touches the file system again: one CMD18 streams the blocks straight
 *      in, with the next block arriving by DMA while the previous one is
 *      being copied.
 *
 *   2. drawSpriteFile(path, x, y) - Simon's original one-shot call, kept
 *      for convenience. It does a spriteLoad() + spriteDraw() each time, so
 *      it pays for the directory walk on every call.
 *
 * Call SD.begin() once in setup() before either. Never call it per access.
 */
#include <CHGfx.h>
#include "SD/SD.h"

/* ------------------------------------------------------------------------ */
/* Tuning                                                                    */
/* ------------------------------------------------------------------------ */
/* Header size of the sprite file format. */
#define SPRITE_HEADER_BYTES 2

/* Tail cache, per loaded sprite. A sprite's pixel data rarely ends exactly
 * on a 512-byte block boundary. If only a few bytes spill into its last
 * block, spriteLoad() keeps those bytes in RAM so spriteDraw() can stop the
 * card stream on a block boundary instead of reading (and discarding) a
 * whole extra block every frame. The fire frames are 2 + 4096 bytes: the
 * last 2 pixel bytes would otherwise cost a 512-byte block each frame
 * (~0.17 ms at 24 MHz, ~4% of the frame). 8 covers that with room to
 * spare; each byte here costs one byte of RAM per SpriteFile. */
#ifndef SPRITE_TAIL_MAX
#define SPRITE_TAIL_MAX 8
#endif

/* ------------------------------------------------------------------------ */
/* Results                                                                   */
/* ------------------------------------------------------------------------ */
/* Return status codes for spriteLoad / spriteDraw / drawSpriteFile */
enum SpriteResult {
    SPRITE_OK                 =  0,
    SPRITE_ERR_FILE_OPEN      = -1,
    SPRITE_ERR_HEADER_READ    = -2,
    SPRITE_ERR_BAD_DIMENSIONS = -3,
    SPRITE_ERR_TOO_WIDE       = -4,   /* kept for compatibility; no longer raised */
    SPRITE_ERR_TRUNCATED      = -5,   /* file shorter than its header claims      */
    SPRITE_ERR_FRAGMENTED     = -6,   /* clusters not contiguous: cannot stream   */
    SPRITE_ERR_IO             = -7,   /* card read failed mid-draw                */
    SPRITE_ERR_NOT_LOADED     = -8    /* spriteDraw() on an empty SpriteFile      */
};

/* ------------------------------------------------------------------------ */
/* A sprite located on the card                                              */
/* ------------------------------------------------------------------------ */
struct SpriteFile {
    uint32_t firstBlock;              /* LBA of file byte 0                    */
    uint16_t blocks;                  /* blocks to stream (tail excluded)      */
    uint8_t  w, h;                    /* from the header; 0 = not loaded       */
    uint8_t  tailLen;                 /* pixel bytes held in tail[], 0..MAX    */
    uint8_t  tail[SPRITE_TAIL_MAX];   /* the bytes after the last full block   */
};

/* ------------------------------------------------------------------------ */
/* Dirty region                                                              */
/* ------------------------------------------------------------------------ */
/* Screen area whose framebuffer bytes actually CHANGED during a draw.
 *
 * Opaque draws diff every row against what is already in the framebuffer,
 * so an animation frame reports only the pixels that differ from the
 * previous one. Flushing just those to the panel is the biggest saving
 * there is, because the LCD transfer is as large a slice of the frame as
 * the SD read. Transparent draws report their whole on-screen rows.
 *
 * Why bands rather than one rectangle: a single bounding box of the fire's
 * changes is ~3750 px, but only ~1560 px really change - the flames are
 * ragged, not rectangular. Keeping one box per 16-row screen band follows
 * the outline more closely; spriteFlushDirty() then merges neighbouring
 * bands whenever one bigger rectangle is cheaper than two (each rectangle
 * costs a window command and a pipeline restart).
 *
 * Measured on the device (12 bpp, fire demo), pixels sent / fps:
 *   one bounding box        3750 px  221 fps
 *   16-row bands            3055 px  231 fps
 *    8-row bands            2900 px  233 fps   <- default
 *    4-row bands            2830 px  230 fps   (more rectangles than it saves)
 * 16 bands x 8 bytes = 128 bytes, on the caller's stack. */
#ifndef SPRITE_BAND_ROWS
#define SPRITE_BAND_ROWS 8             /* power of two, divides GFX_H */
#endif
#define SPRITE_BANDS (GFX_H / SPRITE_BAND_ROWS)

/* Estimated fixed cost of flushing one extra rectangle, in bytes of wire
 * time (window setup + first chunk conversion before the DMA starts).
 * Swept 48 / 96 / 200 on the device: 48 merges too little, 96-200 are
 * within noise of each other. */
#ifndef SPRITE_RECT_OVERHEAD_BYTES
#define SPRITE_RECT_OVERHEAD_BYTES 192
#endif

/* Half-open [x0, x1) x [y0, y1); empty when x1 <= x0. */
struct SpriteRect {
    int16_t x0, y0, x1, y1;
};

struct SpriteDirty {
    SpriteRect band[SPRITE_BANDS];
};

void spriteDirtyReset(SpriteDirty &d);
bool spriteDirtyEmpty(const SpriteDirty &d);

/* Send every dirty part of the framebuffer to the panel and return the
 * number of pixels sent. All rectangles but the last are flushed blocking
 * (the bus is the bottleneck either way); the last is flushed async, so
 * the caller can get on with the next frame's logic while it drains. */
uint32_t spriteFlushDirty(const SpriteDirty &d);

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */
/* Open `path`, read its header, and record where its data sits on the card.
 * The file is closed again before returning; the SpriteFile is all that is
 * kept. Fails with SPRITE_ERR_FRAGMENTED if the file is not stored in
 * consecutive clusters (re-copy it to a freshly formatted card). */
int spriteLoad(SpriteFile &s, const char *path);

/* Draw a loaded sprite into the framebuffer at (x, y).
 *   transparent  -1 = opaque; 0..15 = palette index to leave undrawn.
 *   dirty        optional; grown to cover every pixel that changed.
 * Only touches gfx_fb - call gfx_flush*() yourself afterwards. Waits for
 * any async flush in flight first, because it borrows CHGfx's two 512-byte
 * DMA chunk buffers as landing space for the card data. */
int spriteDraw(const SpriteFile &s, int x, int y, int transparent = -1,
               SpriteDirty *dirty = nullptr);

/* Simon's original one-shot API: load + draw in one call. */
int drawSpriteFile(const char *path, int x, int y, int transparent = -1,
                   SpriteDirty *dirty = nullptr);
