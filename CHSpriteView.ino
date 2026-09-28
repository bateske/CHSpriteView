/*
 * CHSpriteView - a 16-frame fire animation streamed from microSD every frame,
 * on the CHGame board (CH32X035 @ 48 MHz, ST7735 128x128, SD on shared SPI1).
 *
 * Originally by Simon (filmote): https://github.com/filmote/CHSpriteView
 * Optimised for maximum frame rate; see src/CHSpriteView.cpp for the list of
 * changes and src/SD/utility/Sd2Card.cpp for the SD transport rewrite.
 *
 * ---------------------------------------------------------------------------
 * SD CARD
 * ---------------------------------------------------------------------------
 *   /FIRE/FIRE_00.BIN ... /FIRE/FIRE_15.BIN   (from sample/FIRE in this folder)
 *   FAT16 or FAT32. Copy onto a freshly formatted card so the files are
 *   stored contiguously; any file that is not is reported on screen/Serial.
 *
 * ---------------------------------------------------------------------------
 * CONTROLS (for A/B-ing the optimisations live)
 * ---------------------------------------------------------------------------
 *   A      toggle dirty-rectangle flush  <-> full 128x64 fire band flush
 *   B      toggle 12 bpp <-> 16 bpp panel colour mode
 *   START  toggle the 60 fps cap (off = run as fast as the hardware allows)
 *
 * The screen shows fps and where each frame's time goes. With the USB serial
 * port open, the same figures are printed once a second, plus the card and
 * per-file details at boot.
 *
 * ---------------------------------------------------------------------------
 * ANATOMY OF A FRAME  (measured on the device, 12 bpp, uncapped)
 * ---------------------------------------------------------------------------
 * SD card and LCD share ONE SPI bus at 24 MHz (~3 MB/s), so a frame costs
 * the card's read plus the panel's write; they cannot overlap.
 *
 *   SD   2.45 ms  = 0.03 CMD18 + 0.58 card access latency + 0.27 gaps
 *                   between blocks + 8 x 0.17 DMA + 0.10 last block copy
 *   LCD  1.80 ms  ~2900 px that actually changed, in 8-row bands, 1.5 B/px
 *   ----------------------------------------------------------------------
 *        ~4.3 ms  -> 232 fps
 *
 * Everything the CPU does overlaps a DMA transfer: copying/diffing block k
 * into the framebuffer runs while block k+1 arrives, and pixel conversion
 * runs while the previous LCD chunk is on the wire. The ~0.85 ms of card
 * latency is the card's own and the one cost software cannot touch.
 *
 * Simon's original on the same device and card: 2 fps (621 ms per frame in
 * the SD code). See README.md for the full table and how each step moved it.
 *
 * Build with Tools > Optimize > "Faster (-O2)" (232 fps, 31 KB flash).
 * -O1 measures the same, -Os gives 215 fps in 27 KB, and -O3 nearly fills
 * the flash (48.4 KB) for nothing.
 */
#include "src/CHGame.h"
#include "src/CHSpriteView.h"

/* ------------------------------------------------------------------------ */
/* Tuning knobs                                                              */
/* ------------------------------------------------------------------------ */
/* 12 bpp sends 1.5 bytes per pixel instead of 2 - 25% less LCD traffic.
 * RGB444 quantises the palette slightly; B switches live to compare. */
#ifndef DEMO_START_12BPP
#define DEMO_START_12BPP 1
#endif

/* Start uncapped so the fps counter shows what the hardware can do.
 * START toggles a 60 fps cap (Simon's original setting). */
#ifndef DEMO_START_CAPPED
#define DEMO_START_CAPPED 0
#endif

/* Start in dirty-rectangle mode (A toggles). */
#ifndef DEMO_START_RECT
#define DEMO_START_RECT 1
#endif

#define FIRE_FRAMES 16
#define FIRE_X      0
#define FIRE_Y      64

/* ------------------------------------------------------------------------ */
/* Palette                                                                   */
/* ------------------------------------------------------------------------ */
/* Simon's palette. The fire data uses indices 0,1,2,3,5,6,9,10,11; 12..14
 * were unused (black) and are now the HUD's colours. */
enum Colors : uint8_t { Transparent, Blue, White, Red, Navy,
                 BLANK_2, BLANK_3, BLANK_4, BLANK_5, BLANK_6,
                 BLANK_7, BLANK_8, HUD_ACCENT, HUD_DIM, HUD_TEXT, Black };

static const uint16_t palette[16] = {
    0x0000, 0x0001, 0x0020, 0xF949, 0xF128,
    0xFD00, 0xFF45, 0xF129, 0xF4E0, 0xF4E1,
    0xF724, 0xF725,
    0xFE60,     /* 12 HUD_ACCENT  (was 0x0000, unused) */
    0x8410,     /* 13 HUD_DIM     (was 0x0000, unused) */
    0xFFFF,     /* 14 HUD_TEXT    (was 0x0000, unused) */
    0x0000
};

static const uint8_t Sprite_InMem_X = 16;
static const uint8_t Sprite_InMem_Y = 16;
static const uint8_t Sprite_InMem[] PROGMEM
{
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x10, 0x01, 0x10, 0x01, 0x00, 0x10,
    0x01, 0x00, 0x10, 0x01, 0x10, 0x01, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x01, 0x00, 0x00, 0x10, 0x00, 0x10,
    0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10,
    0x01, 0x01, 0x01, 0x00, 0x00, 0x10, 0x10, 0x10,
    0x01, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x11, 0x11, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
};

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */
CHGame chGame;

static SpriteFile fire[FIRE_FRAMES];   /* 20 bytes each: where every frame lives */
static uint8_t    fireCount = 0;
static uint8_t    fireIdx   = 0;

static bool useDirtyRect = DEMO_START_RECT;
static bool use12bpp     = DEMO_START_12BPP;
static bool capped       = DEMO_START_CAPPED;

/* Per-second accumulators. */
static uint32_t statStart;
static uint32_t statFrames;
static uint32_t statSdUs;      /* time in spriteDraw (card -> framebuffer)       */
static uint32_t statLcdUs;     /* time waiting on / starting the panel transfer  */
static uint32_t statPixels;    /* pixels sent to the panel                        */
static uint32_t statErrors;

/* ------------------------------------------------------------------------ */
/* Tiny formatting helpers (no printf: keeps flash and RAM down)             */
/* ------------------------------------------------------------------------ */
static char *putU32(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = tmp[--n];
    *p = 0;
    return p;
}

/* microseconds -> "m.dd" milliseconds */
static char *putMs(char *p, uint32_t us)
{
    p = putU32(p, us / 1000);
    *p++ = '.';
    uint32_t frac = (us % 1000) / 10;
    *p++ = (char)('0' + frac / 10);
    *p++ = (char)('0' + frac % 10);
    *p = 0;
    return p;
}

static char *putStr(char *p, const char *s)
{
    while (*s) *p++ = *s++;
    *p = 0;
    return p;
}

/* ------------------------------------------------------------------------ */
/* Screen furniture                                                          */
/* ------------------------------------------------------------------------ */
#define HUD_X   20
#define HUD_Y0  18
#define HUD_H   (FIRE_Y - HUD_Y0)

static void drawTitle(void)
{
    gfx_fillRect(HUD_X, 0, GFX_W - HUD_X, 16, Black);
    gfx_text(HUD_X, 1, "SD SPRITE STREAM", HUD_ACCENT);
    gfx_text(HUD_X, 9, "A:rect B:bpp S:cap", HUD_DIM);
}

/* Redraw the stats block and push just that part of the screen. */
static void drawHud(uint32_t fps, uint32_t sdUs, uint32_t lcdUs, uint32_t px)
{
    char line[24], *p;
    gfx_fillRect(0, HUD_Y0, GFX_W, HUD_H, Black);

    p = putStr(line, "fps ");    p = putU32(p, fps);
    if (statErrors) { p = putStr(p, "  err "); putU32(p, statErrors); }
    gfx_text(2, HUD_Y0 + 0, line, HUD_TEXT);

    p = putStr(line, "sd  ");    p = putMs(p, sdUs);   putStr(p, " ms");
    gfx_text(2, HUD_Y0 + 9, line, HUD_TEXT);

    p = putStr(line, "lcd ");    p = putMs(p, lcdUs);  putStr(p, " ms");
    gfx_text(2, HUD_Y0 + 18, line, HUD_TEXT);

    p = putStr(line, "px  ");    p = putU32(p, px);    putStr(p, "/8192");
    gfx_text(2, HUD_Y0 + 27, line, HUD_TEXT);

    p = putStr(line, use12bpp ? "12bpp " : "16bpp ");
    p = putStr(p, useDirtyRect ? "rect " : "band ");
    putStr(p, capped ? "60cap" : "free");
    gfx_text(2, HUD_Y0 + 36, line, HUD_DIM);

    gfx_flushRect(0, HUD_Y0, GFX_W, HUD_H);
}

/* Full-screen message for fatal problems (no card, no frames). */
static void fatal(const char *l1, const char *l2)
{
    gfx_clear(Black);
    gfx_text(4, 50, l1, Red);
    if (l2) gfx_text(4, 62, l2, HUD_TEXT);
    gfx_text(4, 80, "START = retry", HUD_DIM);
    gfx_flush();
}

/* ------------------------------------------------------------------------ */
/* Card + frame loading                                                      */
/* ------------------------------------------------------------------------ */
static const char *const sdTypeName[] = { "?", "SD1", "SD2", "SDHC" };

static bool loadAll(void)
{
    /* Mount once. 24 MHz, stepping down to 12/6 MHz only if the card cannot
     * read its own boot sector reliably at speed. */
    if (!SD.begin(PIN_SD_CS)) {
        fatal("SD mount failed", "card? FAT16/32?");
        if (Serial) Serial.println(F("SD mount failed"));
        return false;
    }

    Sd2Card &card = SD.rawCard();
    if (Serial) {
        Serial.print(F("SD: "));
        Serial.print(sdTypeName[card.type() & 3]);
        Serial.print(F("  SPI "));
        Serial.print(24u >> card.sckRateId());
        Serial.print(F(" MHz  DMA "));
        Serial.println(Sd2Card::dmaEnabled() ? F("on") : F("off"));
    }

    /* Resolve every frame to its raw block address, once. */
    char path[] = "FIRE/FIRE_00.BIN";
    fireCount = 0;
    uint8_t bad = 0;
    for (uint8_t i = 0; i < FIRE_FRAMES; i++) {
        path[10] = (char)('0' + i / 10);
        path[11] = (char)('0' + i % 10);
        int r = spriteLoad(fire[fireCount], path);
        if (Serial) {
            Serial.print(path);
            if (r == SPRITE_OK) {
                Serial.print(F("  lba "));    Serial.print(fire[fireCount].firstBlock);
                Serial.print(F("  blocks ")); Serial.print(fire[fireCount].blocks);
                Serial.print(F("  tail "));   Serial.println(fire[fireCount].tailLen);
            } else {
                Serial.print(F("  ERROR "));  Serial.println(r);
            }
        }
        if (r == SPRITE_OK) fireCount++;
        else bad++;
    }

    if (fireCount == 0) {
        fatal("No FIRE frames", "/FIRE/FIRE_00.BIN");
        return false;
    }
    if (bad && Serial) {
        Serial.print(bad);
        Serial.println(F(" frame(s) skipped - fragmented files report -6; recopy to a fresh card"));
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* setup / loop                                                              */
/* ------------------------------------------------------------------------ */
static void drawStaticScreen(void)
{
    Gfx.clear(Black);
    Gfx.drawSprite(Sprite_InMem, 0, 0, Sprite_InMem_X, Sprite_InMem_Y, 0);
    drawTitle();
    Gfx.display();
}

void setup()
{
    /* 24 MHz SPI (HCLK/2). The panel mode is the one real trade-off here:
     * 12 bpp is 25% less panel traffic at the cost of 4-bit colour channels. */
    Gfx.begin(GFX_DIV2, use12bpp ? GFX_12BPP : GFX_16BPP);
    Gfx.setPalette(palette, 16);

    chGame.setFrameRate(60);
    chGame.boot();

    drawStaticScreen();
    gfx_text(2, HUD_Y0, "mounting SD...", HUD_TEXT);
    gfx_flushRect(0, HUD_Y0, GFX_W, 10);

    /* Give a host a moment to open the serial port so the boot report is
     * visible; costs nothing when nothing is plugged in. */
    Serial.waitForPC(1500);

    while (!loadAll()) {
        /* No card or no frames: wait for START, then try again. */
        do { chGame.pollButtons(); delay(10); } while (!chGame.justPressed(START_BUTTON));
        drawStaticScreen();
    }

    drawStaticScreen();
    statStart = millis();
}

void loop()
{
    /* ---- input: live A/B switches ------------------------------------ */
    /* Uncapped, this loop runs every few ms - faster than a switch stops
     * bouncing - so a toggle is only accepted 250 ms after the last one. */
    static uint32_t lastToggle;
    chGame.pollButtons();
    if (millis() - lastToggle > 250) {
        if (chGame.justPressed(A_BUTTON))     { useDirtyRect = !useDirtyRect; lastToggle = millis(); }
        if (chGame.justPressed(START_BUTTON)) { capped = !capped;             lastToggle = millis(); }
        if (chGame.justPressed(B_BUTTON)) {
            use12bpp = !use12bpp;
            gfx_setColorMode(use12bpp ? GFX_12BPP : GFX_16BPP);  /* waits for the bus */
            lastToggle = millis();
        }
    }

    if (capped && !chGame.nextFrame()) return;

    /* ---- 1. wait for the previous frame's panel transfer ------------- */
    uint32_t t0 = micros();
    gfx_wait();
    uint32_t t1 = micros();

    /* ---- 2. card -> framebuffer, noting which pixels changed --------- */
    SpriteDirty d;
    spriteDirtyReset(d);
    if (spriteDraw(fire[fireIdx], FIRE_X, FIRE_Y, -1, &d) != SPRITE_OK) statErrors++;
    uint32_t t2 = micros();

    /* ---- 3. framebuffer -> panel, asynchronously ---------------------- */
    /* Returns as soon as the first chunk is on the wire; the rest is fed by
     * the DMA interrupt while this loop carries on. Step 1 of the next frame
     * (or the SD bus claim itself) waits for it to finish. */
    uint32_t px;
    if (useDirtyRect) {
        px = spriteFlushDirty(d);                     /* identical frame: sends nothing */
    } else {
        gfx_flushRectAsync(FIRE_X, FIRE_Y, 128, 64);  /* the original: whole band */
        px = 128u * 64u;
    }
    uint32_t t3 = micros();

    statSdUs   += t2 - t1;
    statLcdUs  += (t1 - t0) + (t3 - t2);
    statPixels += px;
    statFrames++;

    if (++fireIdx >= fireCount) fireIdx = 0;

    /* ---- once a second: report ---------------------------------------- */
    uint32_t now = millis();
    if (now - statStart >= 1000) {
        uint32_t f = statFrames ? statFrames : 1;
        uint32_t fps   = (statFrames * 1000u + (now - statStart) / 2) / (now - statStart);
        uint32_t sdUs  = statSdUs / f;
        uint32_t lcdUs = statLcdUs / f;
        uint32_t pxAvg = statPixels / f;

        gfx_wait();                         /* the HUD shares the bus too */
        drawHud(fps, sdUs, lcdUs, pxAvg);

        if (Serial) {
            Serial.print(F("fps "));      Serial.print(fps);
            Serial.print(F("  sd "));     Serial.print(sdUs);
            Serial.print(F("us  lcd "));  Serial.print(lcdUs);
            Serial.print(F("us  px "));   Serial.print(pxAvg);
            Serial.print(use12bpp ? F("  12bpp") : F("  16bpp"));
            Serial.print(useDirtyRect ? F(" rect") : F(" band"));
            Serial.print(capped ? F(" 60cap") : F(" free"));
            Serial.print(F("  err "));    Serial.println(statErrors);
#ifdef SD_PROFILE
            /* Per-read phase breakdown in microseconds (see Sd2Card.h). */
            if (sdProf.calls) {
                const uint32_t div = sdProf.calls * (F_CPU / 1000000);
                Serial.print(F("  sd phases us: cmd "));   Serial.print(sdProf.cmd / div);
                Serial.print(F("  1stTok "));              Serial.print(sdProf.firstToken / div);
                Serial.print(F("  nextToks "));            Serial.print(sdProf.nextTokens / div);
                Serial.print(F("  fn "));                  Serial.print(sdProf.fn / div);
                Serial.print(F("  dmaWait "));             Serial.print(sdProf.dmaWait / div);
                Serial.print(F("  stop "));                Serial.print(sdProf.stop / div);
                Serial.print(F("  lastFn "));              Serial.print(sdProf.lastFn / div);
                Serial.print(F("  total "));               Serial.println(sdProf.total / div);
            }
            memset(&sdProf, 0, sizeof(sdProf));
#endif
        }

        statStart  = millis();              /* exclude the report itself */
        statFrames = statSdUs = statLcdUs = statPixels = 0;
    }
}
