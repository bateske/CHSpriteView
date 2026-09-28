/*
 * CHSpriteView.cpp - stream packed 4 bpp sprites from microSD into CHGfx's
 * framebuffer.
 *
 * ===========================================================================
 * WHAT CHANGED FROM THE ORIGINAL, AND WHY
 * ===========================================================================
 * The original read each sprite like this, per frame:
 *
 *     sdBegin()  -> SD.begin(PB11)      <- full card re-initialisation
 *     SD.open(path)                     <- directory walk
 *     sdBegin() / read header / sdEnd()
 *     sdBegin() / read 512 B / sdEnd()  x 8
 *     sdBegin() / close / sdEnd()
 *     gfx_blit() each row
 *
 * sdBegin() re-ran SD.begin() - CMD0, the ACMD41 wake-up loop and a volume
 * mount, all at 250 kHz - eleven times a frame, and SD.begin() then left
 * the bus at 3 MHz. That is where "super slow" came from. In order of
 * impact, this version:
 *
 *  1. Mounts the card ONCE (in setup), at 24 MHz with fall-back.
 *  2. Uses the rewritten SD transport (src/SD/utility/Sd2Card.cpp):
 *     register-level SPI and DMA instead of SPIClass::transfer() per byte.
 *  3. Resolves each file to a raw block address ONCE (spriteLoad), so a
 *     draw never touches the FAT or a directory again.
 *  4. Reads the frame with ONE CMD18 multi-block command, pipelined: DMA
 *     fills one 512-byte buffer while the CPU copies the previous one out.
 *  5. Copies opaque, byte-aligned rows straight into gfx_fb (the file's
 *     packing IS the framebuffer's) instead of calling gfx_blit per row,
 *     and diffs each row while copying, so the caller learns exactly which
 *     rectangle changed and can send only that to the LCD.
 *  6. Skips card blocks that are wholly above or below the screen, and ends
 *     the stream on a block boundary thanks to a small per-sprite tail
 *     cache, so no block is read that is not needed.
 *
 * The removed sdBegin()/sdEnd()/spiClaimForLcd() are not needed any more:
 * the SD transport claims SPI1 from CHGfx and hands it back by itself
 * (busClaim()/busRelease() in Sd2Card.cpp), including waiting out an async
 * flush that is still on the wire.
 * ===========================================================================
 */
#include "CHSpriteView.h"

/* Same SRAM-execution trick as CHGfx and Sd2Card: flash is 3 wait states
 * at 48 MHz, and the row copy/diff below is a tight byte loop. */
#define SV_RAMFUNC __attribute__((section(".srodata.svramfunc"), noinline))

/* The row consumer around rowCopyDiff (block -> rows -> framebuffer) can run
 * from SRAM too. It is only a few instructions per ROW, not per byte, so
 * this is a RAM-for-speed trade decided by measurement: in SRAM 233 fps,
 * in flash 231 fps (within noise), and flash saves 516 bytes of RAM. Only
 * the per-byte loop, rowCopyDiff, earns its place in SRAM. */
#ifndef SPRITE_CONSUMER_IN_RAM
#define SPRITE_CONSUMER_IN_RAM 0
#endif
#if SPRITE_CONSUMER_IN_RAM
  #define SV_CONSUMER SV_RAMFUNC
#else
  #define SV_CONSUMER
#endif

/* ------------------------------------------------------------------------ */
/* Draw context                                                              */
/* ------------------------------------------------------------------------ */
/*
 * The card delivers the file as a flat byte stream in 512-byte blocks; the
 * sprite is rows of rowBytes. Rows straddle block boundaries (the 2-byte
 * header alone guarantees that), so the consumer below reassembles rows:
 * whole rows are used in place in the block buffer, and only a row that is
 * split across two blocks is copied into `carry` first.
 *
 * All offsets are FILE offsets, so clipping, the header and the tail cache
 * are all just ranges of the same number line:
 *
 *   0          2                     startByte             endByte
 *   | header   | rows above screen   | visible rows ...    | rows below |
 */
struct DrawCtx {
    const SpriteFile *s;
    int       x, y;            /* screen position of the sprite            */
    int       transparent;     /* -1 opaque, else skipped palette index    */
    uint16_t  rowBytes;        /* ceil(w / 2)                              */
    uint16_t  row;             /* sprite row the next complete row is      */
    uint32_t  off;             /* file offset of the next byte fed in      */
    uint32_t  startByte;       /* first byte of the first visible row      */
    uint32_t  endByte;         /* one past the last byte of the last one   */
    bool      direct;          /* opaque byte copy into gfx_fb possible    */
    uint16_t  carryLen;        /* bytes of a split row gathered so far     */
    SpriteDirty *dirty;        /* where to accumulate changes (may be null)*/
    uint8_t   carry[128];      /* one row: 255 px max -> 128 bytes         */
};

/* Mark [x0,x1) of screen row y as changed, in that row's band. */
static inline void dirtyAdd(SpriteDirty *d, int x0, int y, int x1)
{
    if (!d) return;
    SpriteRect *r = &d->band[y / SPRITE_BAND_ROWS];
    if (x0 < r->x0) r->x0 = (int16_t)x0;
    if (y  < r->y0) r->y0 = (int16_t)y;
    if (x1 > r->x1) r->x1 = (int16_t)x1;
    if (y  >= r->y1) r->y1 = (int16_t)(y + 1);
}

void spriteDirtyReset(SpriteDirty &d)
{
    for (uint8_t i = 0; i < SPRITE_BANDS; i++) {
        d.band[i].x0 = d.band[i].y0 = 0x7FFF;
        d.band[i].x1 = d.band[i].y1 = -1;
    }
}

static inline bool rectEmpty(const SpriteRect &r) { return r.x1 <= r.x0 || r.y1 <= r.y0; }

bool spriteDirtyEmpty(const SpriteDirty &d)
{
    for (uint8_t i = 0; i < SPRITE_BANDS; i++)
        if (!rectEmpty(d.band[i])) return false;
    return true;
}

/* What flushing r would cost, in bytes of wire time: CHGfx rounds x out to
 * 8 px in 12 bpp (2 px in 16 bpp), then 1.5 or 2 bytes a pixel, plus the
 * fixed per-rectangle overhead. */
static uint32_t rectCost(const SpriteRect &r)
{
    const bool b12 = gfx_colorMode() == GFX_12BPP;
    const int  a   = b12 ? 8 : 2;
    const int  x0  = r.x0 & ~(a - 1);
    const int  x1  = (r.x1 + a - 1) & ~(a - 1);
    const uint32_t px = (uint32_t)(x1 - x0) * (uint32_t)(r.y1 - r.y0);
    return (b12 ? px * 3u / 2u : px * 2u) + SPRITE_RECT_OVERHEAD_BYTES;
}

static uint32_t flushOne(const SpriteRect &r, bool async)
{
    const int w = r.x1 - r.x0, h = r.y1 - r.y0;
    if (async) gfx_flushRectAsync(r.x0, r.y0, w, h);
    else       gfx_flushRect(r.x0, r.y0, w, h);
    return (uint32_t)w * (uint32_t)h;
}

uint32_t spriteFlushDirty(const SpriteDirty &d)
{
    /* Greedy, top to bottom: grow the pending rectangle by the next band
     * whenever their union costs no more than sending them separately;
     * otherwise send the pending one and start again from this band. */
    SpriteRect cur;
    bool have = false;
    uint32_t px = 0;
    for (uint8_t i = 0; i < SPRITE_BANDS; i++) {
        const SpriteRect &b = d.band[i];
        if (rectEmpty(b)) continue;
        if (!have) { cur = b; have = true; continue; }
        SpriteRect u;
        u.x0 = cur.x0 < b.x0 ? cur.x0 : b.x0;
        u.y0 = cur.y0;
        u.x1 = cur.x1 > b.x1 ? cur.x1 : b.x1;
        u.y1 = b.y1;
        if (rectCost(u) <= rectCost(cur) + rectCost(b)) {
            cur = u;
        } else {
            px += flushOne(cur, false);
            cur = b;
        }
    }
    if (have) px += flushOne(cur, true);
    return px;
}

/*
 * Copy one row into the framebuffer, touching only what differs.
 *
 * Scan in from the left for the first differing unit and in from the right
 * for the last, then copy just that span. The scans are the diff: an
 * unchanged row costs two comparisons per unit and no stores, and the
 * caller learns the changed column range for free. Returns false if the
 * row is identical to what is already there. *lo / *hi are BYTE offsets.
 *
 * MEASURED, NOT ASSUMED: the first version compared a byte at a time, on
 * the theory that it would hide inside the ~171 us each 512-byte block
 * takes to arrive by DMA. The on-device profile (-DSD_PROFILE) said
 * otherwise: ~220 us per block, about 20 cycles per byte - longer than the
 * DMA, so it became THE bottleneck of the whole frame. Hence word units.
 *
 * Alignment. The framebuffer row is word-aligned whenever x is a multiple
 * of 8. The source row sits wherever it falls in the 512-byte block
 * buffer: the fire files' 2-byte header puts every row at 2 mod 4. The
 * QingKe core does not promise cheap misaligned word loads, but a
 * misaligned WORD at an even address is two aligned HALFWORDS, so:
 *
 *   src % 4 == 0   plain word loads                    (1 load  / 4 bytes)
 *   src % 4 == 2   word assembled from two halfwords   (2 loads / 4 bytes)
 *   anything else  the original byte loop              (fallback)
 *
 * Word granularity means the dirty span is reported in 8-pixel steps,
 * which is exactly the 12 bpp flush alignment CHGfx rounds to anyway.
 */
static inline uint32_t ldWord(const uint8_t *p, bool half)
{
    if (!half) return *(const uint32_t *)p;
    const uint16_t *h = (const uint16_t *)p;
    return (uint32_t)h[0] | ((uint32_t)h[1] << 16);    /* little-endian */
}

static SV_RAMFUNC bool rowCopyDiff(uint8_t *dst, const uint8_t *src, uint16_t n,
                                   uint16_t *lo, uint16_t *hi)
{
    if ((((uintptr_t)dst | n) & 3) == 0 && ((uintptr_t)src & 1) == 0) {
        const bool half  = ((uintptr_t)src & 2) != 0;
        uint32_t  *d     = (uint32_t *)dst;
        const uint16_t words = n >> 2;

        uint16_t i = 0;
        while (i < words && ldWord(src + 4 * i, half) == d[i]) i++;
        if (i == words) return false;
        uint16_t j = words - 1;
        while (ldWord(src + 4 * j, half) == d[j]) j--;     /* stops at i at the latest */
        for (uint16_t k = i; k <= j; k++) d[k] = ldWord(src + 4 * k, half);
        *lo = (uint16_t)(4 * i);
        *hi = (uint16_t)(4 * j + 3);
        return true;
    }

    /* Byte fallback: odd source address, or a width that is not a whole
     * number of words. */
    uint16_t i = 0;
    while (i < n && dst[i] == src[i]) i++;
    if (i == n) return false;
    uint16_t j = n - 1;
    while (dst[j] == src[j]) j--;
    *lo = i;
    *hi = j;
    for (uint16_t k = i; k <= j; k++) dst[k] = src[k];
    return true;
}

/* One complete sprite row has arrived: put it on screen (in the framebuffer). */
static SV_CONSUMER void emitRow(DrawCtx &c, const uint8_t *src)
{
    const int sy = c.y + c.row;
    c.row++;
    if (sy < 0 || sy >= GFX_H) return;     /* belt and braces: range is pre-clipped */

    const int w = c.s->w;

    if (c.direct) {
        /* Opaque, even x, even width, fully inside horizontally: the row's
         * bytes ARE the framebuffer bytes. */
        uint8_t *dst = gfx_fb + (uint32_t)sy * GFX_FB_STRIDE + (c.x >> 1);
        uint16_t lo, hi;
        if (rowCopyDiff(dst, src, c.rowBytes, &lo, &hi)) {
            /* byte b covers pixels x + 2b and x + 2b + 1 */
            dirtyAdd(c.dirty, c.x + 2 * lo, sy, c.x + 2 * hi + 2);
        }
    } else {
        /* Transparency, odd alignment or horizontal clipping: let CHGfx's
         * nibble-aware blit handle it, one row at a time. We cannot cheaply
         * tell what changed, so report the whole visible span. */
        gfx_blit(src, c.x, sy, w, 1, c.transparent);
        int x0 = c.x < 0 ? 0 : c.x;
        int x1 = c.x + w > GFX_W ? GFX_W : c.x + w;
        dirtyAdd(c.dirty, x0, sy, x1);
    }
}

/*
 * Feed the next n bytes of the file (starting at file offset c.off).
 * Bytes before startByte (header, clipped rows, the unused front of the
 * first block) and after endByte are dropped; the rest is cut into rows.
 */
static SV_CONSUMER void consume(DrawCtx &c, const uint8_t *p, uint32_t n)
{
    /* Leading bytes we do not want. */
    if (c.off < c.startByte) {
        uint32_t k = c.startByte - c.off;
        if (k > n) k = n;
        p += k; n -= k; c.off += k;
    }
    /* Trailing bytes we do not want. */
    if (c.off >= c.endByte) {
        c.off += n;
        return;
    }
    if (c.off + n > c.endByte) n = c.endByte - c.off;
    c.off += n;

    while (n) {
        if (c.carryLen == 0 && n >= c.rowBytes) {
            emitRow(c, p);                 /* whole row in place: no copy */
            p += c.rowBytes;
            n -= c.rowBytes;
            continue;
        }
        /* Row split across a block boundary: gather it. */
        uint16_t k = c.rowBytes - c.carryLen;
        if (k > n) k = (uint16_t)n;
        memcpy(c.carry + c.carryLen, p, k);
        c.carryLen += k;
        p += k;
        n -= k;
        if (c.carryLen == c.rowBytes) {
            emitRow(c, c.carry);
            c.carryLen = 0;
        }
    }
}

/* readBlocksPipelined() callback: runs while the NEXT block is arriving. */
static SV_CONSUMER void onBlock(const uint8_t *block, void *user)
{
    consume(*static_cast<DrawCtx *>(user), block, 512);
}

/* ------------------------------------------------------------------------ */
/* Clipping / setup shared by both draw paths                                */
/* ------------------------------------------------------------------------ */
/* Returns false if nothing of the sprite is on screen. */
static bool setupCtx(DrawCtx &c, const SpriteFile &s, int x, int y,
                     int transparent, SpriteDirty *dirty)
{
    c.s           = &s;
    c.x           = x;
    c.y           = y;
    c.transparent = transparent;
    c.rowBytes    = (uint16_t)((s.w + 1) >> 1);
    c.carryLen    = 0;
    c.dirty       = dirty;

    /* Fully off screen? */
    if (x >= GFX_W || y >= GFX_H || x + s.w <= 0 || y + s.h <= 0) return false;

    /* Visible rows [firstRow, lastRow). Rows above/below the screen are not
     * just skipped when drawing - they are left out of the byte range, so
     * the blocks that hold only them are never read from the card. */
    int firstRow = y < 0 ? -y : 0;
    int lastRow  = (y + s.h > GFX_H) ? GFX_H - y : s.h;
    c.row       = (uint16_t)firstRow;
    c.startByte = SPRITE_HEADER_BYTES + (uint32_t)firstRow * c.rowBytes;
    c.endByte   = SPRITE_HEADER_BYTES + (uint32_t)lastRow  * c.rowBytes;

    /* The fast path needs the row to be exactly a run of framebuffer bytes. */
    c.direct = transparent < 0 && !(x & 1) && !(s.w & 1) && x >= 0 && x + s.w <= GFX_W;
    return true;
}

/* ------------------------------------------------------------------------ */
/* spriteLoad                                                                */
/* ------------------------------------------------------------------------ */
int spriteLoad(SpriteFile &s, const char *path)
{
    s.w = s.h = 0;
    s.blocks = 0;
    s.tailLen = 0;

    File f = SD.open(path);
    if (!f) return SPRITE_ERR_FILE_OPEN;

    uint8_t header[SPRITE_HEADER_BYTES];
    if (f.read(header, SPRITE_HEADER_BYTES) != SPRITE_HEADER_BYTES) {
        f.close();
        return SPRITE_ERR_HEADER_READ;
    }
    const uint8_t w = header[0], h = header[1];
    if (w == 0 || h == 0) {
        f.close();
        return SPRITE_ERR_BAD_DIMENSIONS;
    }

    const uint32_t rowBytes = (w + 1u) >> 1;
    const uint32_t end      = SPRITE_HEADER_BYTES + rowBytes * h;   /* file bytes used */
    if (f.size() < end) {
        f.close();
        return SPRITE_ERR_TRUNCATED;
    }

    /* Where does it live? This walks the file's FAT chain once. */
    uint32_t first, last;
    if (!f.contiguousRange(first, last)) {
        f.close();
        return SPRITE_ERR_FRAGMENTED;
    }

    /* Blocks to stream, and whether the few bytes after the last whole
     * block can be cached instead of streaming one more block per draw. */
    const uint32_t full = end >> 9;
    const uint16_t rem  = (uint16_t)(end & 511u);
    if (rem == 0) {
        s.blocks = (uint16_t)full;
    } else if (rem <= SPRITE_TAIL_MAX && full > 0) {
        s.blocks = (uint16_t)full;
        if (!f.seek(full << 9) || f.read(s.tail, rem) != (int)rem) {
            f.close();
            return SPRITE_ERR_TRUNCATED;
        }
        s.tailLen = (uint8_t)rem;
    } else {
        s.blocks = (uint16_t)(full + 1);
    }

    f.close();
    s.firstBlock = first;
    s.w = w;
    s.h = h;
    return SPRITE_OK;
}

/* ------------------------------------------------------------------------ */
/* spriteDraw                                                                */
/* ------------------------------------------------------------------------ */
int spriteDraw(const SpriteFile &s, int x, int y, int transparent, SpriteDirty *dirty)
{
    if (s.w == 0) return SPRITE_ERR_NOT_LOADED;

    DrawCtx c;
    if (!setupCtx(c, s, x, y, transparent, dirty)) return SPRITE_OK;

    /* Everything below writes gfx_fb, and the card read borrows CHGfx's two
     * 512-byte flush chunk buffers as DMA landing space. Both are only safe
     * once the previous frame's async flush has finished reading them. */
    gfx_wait();

    /* Block range covering [startByte, endByte), limited to what is streamed;
     * anything past the streamed blocks comes from the tail cache. */
    const uint32_t streamEnd  = (uint32_t)s.blocks << 9;
    const uint32_t startBlk   = c.startByte >> 9;
    const uint32_t wantEnd    = c.endByte < streamEnd ? c.endByte : streamEnd;
    const uint32_t endBlk     = (wantEnd + 511u) >> 9;          /* exclusive */
    const uint16_t nBlk       = endBlk > startBlk ? (uint16_t)(endBlk - startBlk) : 0;
    c.off = startBlk << 9;

    if (nBlk) {
        uint8_t *buf0 = gfx_chunkScratch();
        uint8_t *buf1 = buf0 + GFX_CHUNK_BYTES;
        if (!SD.rawCard().readBlocksPipelined(s.firstBlock + startBlk, nBlk,
                                              buf0, buf1, onBlock, &c)) {
            return SPRITE_ERR_IO;
        }
    }

    /* Bytes beyond the last streamed block, from RAM. consume() ignores
     * them if the visible range ended earlier. */
    if (s.tailLen && c.endByte > streamEnd) {
        c.off = streamEnd;
        consume(c, s.tail, s.tailLen);
    }
    return SPRITE_OK;
}

/* ------------------------------------------------------------------------ */
/* drawSpriteFile - Simon's original one-shot API                            */
/* ------------------------------------------------------------------------ */
/* Fallback for fragmented files: the ordinary File API into one of the
 * chunk buffers, fed through the same row consumer. Slower (FAT lookups,
 * one command per block) but still correct and still zero-copy per row. */
static int drawViaFileApi(const char *path, int x, int y, int transparent,
                          SpriteDirty *dirty)
{
    File f = SD.open(path);
    if (!f) return SPRITE_ERR_FILE_OPEN;

    uint8_t header[SPRITE_HEADER_BYTES];
    if (f.read(header, SPRITE_HEADER_BYTES) != SPRITE_HEADER_BYTES) {
        f.close();
        return SPRITE_ERR_HEADER_READ;
    }
    SpriteFile s;
    s.w = header[0];
    s.h = header[1];
    s.tailLen = 0;
    if (s.w == 0 || s.h == 0) { f.close(); return SPRITE_ERR_BAD_DIMENSIONS; }

    DrawCtx c;
    if (!setupCtx(c, s, x, y, transparent, dirty)) { f.close(); return SPRITE_OK; }

    gfx_wait();
    uint8_t *buf = gfx_chunkScratch();
    c.off = c.startByte;
    if (!f.seek(c.startByte)) { f.close(); return SPRITE_ERR_TRUNCATED; }
    while (c.off < c.endByte) {
        uint32_t want = c.endByte - c.off;
        if (want > 512) want = 512;
        int got = f.read(buf, (uint16_t)want);
        if (got <= 0) { f.close(); return SPRITE_ERR_TRUNCATED; }
        consume(c, buf, (uint32_t)got);
    }
    f.close();
    return SPRITE_OK;
}

int drawSpriteFile(const char *path, int x, int y, int transparent, SpriteDirty *dirty)
{
    SpriteFile s;
    int r = spriteLoad(s, path);
    if (r == SPRITE_ERR_FRAGMENTED) return drawViaFileApi(path, x, y, transparent, dirty);
    if (r != SPRITE_OK) return r;
    return spriteDraw(s, x, y, transparent, dirty);
}
