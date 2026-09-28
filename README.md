# CHSpriteView

Streams packed 4 bpp (16-colour) sprites from microSD into the CHGfx
framebuffer on the **CHGame** board (CH32X035 @ 48 MHz, ST7735 128×128 LCD,
microSD, with the LCD and SD card sharing one SPI bus).

The demo plays a 16-frame, 128×64 fire animation read from the SD card
**every frame**.

This is an optimised fork of Simon's (filmote)
[CHSpriteView](https://github.com/filmote/CHSpriteView), based on commit
`d2108c3`. Measured on the same board and card, it goes from
**2 fps to 232 fps**.

---

## Quick start

### 1. Put the frames on the SD card

Copy `sample/FIRE` to the **root** of a FAT16 or FAT32 card:

```
/FIRE/FIRE_00.BIN
/FIRE/FIRE_01.BIN
  ...
/FIRE/FIRE_15.BIN
```

A freshly formatted card is best. The fast path needs each file stored in
consecutive clusters, which small files on a clean card always are. Any file
that isn't is reported at boot (error `-6`) and skipped.

### 2. Build and upload

Use **Tools ▸ Optimize ▸ Faster (-O2)**.

```
arduino-cli compile -b CHGame:ch32v:CHGame:opt=o2std,rtlib=nano .
arduino-cli upload  -b CHGame:ch32v:CHGame -p COMx .
```

### 3. Controls

| Button | Toggles |
|---|---|
| A | flush changed rows only ↔ flush the whole 128×64 fire band |
| B | 12 bpp ↔ 16 bpp panel colour mode |
| START | 60 fps cap ↔ uncapped |

The screen shows fps, SD time, LCD time and pixels sent per frame. With the
USB serial port open, the same figures print once a second, after a boot
report: card type, SPI clock, DMA state, and each frame's block address.

The ST7735 only scans its glass at roughly 100 Hz, so rates above that are
headroom for game logic, not extra visible frames. Uncapped, the fire will
look very fast; press START for the intended look.

---

## The original implementation

Simon's version worked like this for every frame:

```
loop():
    Gfx.clear()                             clear the whole 8 KB framebuffer
    drawSpriteFile("FIRE/FIRE_xx.BIN")
        sdBegin() -> SD.begin(PB11)          full card re-initialisation
        SD.open(path)                        walk root dir, then FIRE dir
        sdEnd()
        sdBegin() -> SD.begin(PB11)
        read 2-byte header
        sdEnd()
        repeat 8 times:
            sdBegin() -> SD.begin(PB11)
            read 512 bytes                   one CMD17 per 512-byte block
            sdEnd()
            gfx_blit() each of the 8 rows    CPU copies row by row
        sdBegin() / close / sdEnd()
    draw 16x16 sprite from flash
    Gfx.display()                           send all 128x128 px to the LCD, 16 bpp
```

It used the stock Arduino SD 1.3.0 library, with one fix in `Sd2PinMap.h`
so it would compile for RISC-V at all.

### Why it ran at 2 fps

Measured: **621 ms per frame** in the SD code.

1. **The card was re-initialised ~11 times per frame.** `sdBegin()` called
   `SD.begin()`, which runs the full SD start-up: CMD0, the ACMD41 wake-up
   loop, then re-reading the MBR and boot sector. All of it runs at 250 kHz.
   It was there because the SD library and the LCD driver share SPI1, and
   re-initialising looked like the way to get the bus back into SD mode.
2. **`SD.begin(cs)` leaves the bus at 3 MHz.** The stock library asks for
   `SPI_HALF_SPEED`, and the core's SPI library rounds that to 3 MHz, not
   the 24 MHz the hardware can do.
3. **Every byte went through `SPIClass::transfer()`.** That means a virtual
   call, a pin-settings lookup, and a `spi_transfer()` that polls the status
   flags through function calls and checks a 64-bit timeout on every byte.
   All of it runs from flash with 3 wait states: several microseconds per
   byte, whatever the clock.
4. **One command per 512-byte block.** Each `CMD17` pays the card's
   access latency again.
5. **The directory was walked every frame**, to find the same 16 files.
6. **The whole screen was sent every frame**, even though only part of
   the fire changes.

An earlier bug is also explained here: `SD.begin()` with no pin argument
used `SS`, which on the CHGame variant is **PA4, the LCD's chip select**.
The card was never selected, and SD commands went to the display. That is
why passing `PB11` was needed.

---

## The optimised implementation

Per frame now:

```
loop():
    gfx_wait()                               previous LCD transfer finished
    spriteDraw(fire[i], 0, 64, &dirty)
        CMD18 at the frame's raw block address    one command, no FAT work
        8 blocks arrive by DMA into two 512-byte buffers, ping-pong;
        while block k+1 arrives, the CPU copies block k's rows straight
        into the framebuffer, comparing as it goes, noting what changed
        last 2 pixel bytes come from a RAM cache (no 9th block)
        CMD12
    spriteFlushDirty(dirty)                   send ONLY the changed rows, 12 bpp,
                                              asynchronously
```

It still uses the CHGfx 4 bpp framebuffer (8 KB). Nothing about the
framebuffer design changed.

### What changed, and why

#### SD library (`src/SD`, a vendored and modified Arduino SD 1.3.0)

Every change is marked `CHGAME:` in the source.

| Change | File | Why |
|---|---|---|
| Register-level SPI1 byte exchange | `utility/Sd2Card.cpp` | Removes the per-byte `SPIClass` call chain. The core SPI library is no longer linked (~2.8 KB flash saved). |
| DMA for bulk reads: DMA1 CH2 receives, CH3 clocks out `0xFF` (CH32X035 RM §9.2.3) | `utility/Sd2Card.cpp` | Bytes move back-to-back at the full 24 MHz (~3 MB/s) with no CPU. Falls back to polled SPI automatically if DMA ever fails. |
| Bus hand-off: `busClaim()` / `busRelease()` save the LCD's SPI1 setup, apply the card's, and restore it | `utility/Sd2Card.cpp` | Replaces the `SD.begin()`-per-access workaround and the hard-coded `sdEnd()`. It also waits out an async CHGfx flush still on the wire, so SD calls are safe even without `gfx_wait()`. |
| `CMD18` multi-block streaming: `readStart()`, `readStream()`, `readStop()` | `utility/Sd2Card.cpp`, `utility/SdInfo.h` | One command and one access latency for a whole run of blocks. |
| `readBlocksPipelined()`: two buffers, each block handed to a callback while the next arrives by DMA | `utility/Sd2Card.cpp` | Whatever is done with the data (copying, diffing) costs no extra time as long as it is faster than the wire. |
| `SD.begin()` mounts at 24 MHz and steps down to 12, then 6 MHz, if the card can't read its boot sector reliably | `SD.cpp` | Full speed by default, safe on slow cards or wiring. |
| `SD.begin()` with no argument selects `PIN_SD_CS` (PB11) | `utility/Sd2Card.h` | Fixes the PA4 / LCD chip-select bug. |
| `File::contiguousRange()` and `SD.rawCard()` | `SD.h`, `File.cpp` | Lets a file be streamed as raw blocks without the FAT. The underlying `SdFile::contiguousRange()` already existed but was never exposed. |
| `File.read()` of two or more whole blocks uses `CMD18` | `utility/SdFile.cpp` | Speeds up ordinary file reads in every sketch, not just this one. |
| CH32 pin-map branch | `utility/Sd2PinMap.h` | Stock SD has none for RISC-V and won't compile. |

Define `SD_CH32_DISABLE_FAST` to get the stock transport back for A/B
testing. Define `SD_PROFILE` to get a per-phase timing breakdown of each
streamed read.

#### Sprite layer (`src/CHSpriteView.cpp`)

| Change | Why |
|---|---|
| Removed `sdBegin()` / `sdEnd()` / `spiClaimForLcd()`. The SD library handles the bus itself. | The main cause of the slowdown. |
| New `spriteLoad()`: opens a file **once**, reads its header, and records its raw block address (a 20-byte `SpriteFile`) | No directory walk or FAT lookup per frame. |
| New `spriteDraw()`: one pipelined `CMD18` read per frame | See above. |
| Opaque, byte-aligned rows are copied straight into `gfx_fb` instead of through `gfx_blit()` | The file's packing *is* the framebuffer's packing, so no conversion is needed. |
| Rows are compared with the framebuffer while being copied, a 32-bit word at a time | The comparison produces the exact changed region for free. The first version compared one byte at a time; on-device profiling showed that was slower than the DMA and had become the bottleneck, so it is now word-wide. |
| Dirty region tracked per 8-row screen band; `spriteFlushDirty()` merges neighbouring bands when one rectangle is cheaper | The fire's changed area is ragged. One bounding box sends ~3,750 px, bands send ~2,900 px. |
| Tail cache: the few pixel bytes past the last whole block are kept in RAM | The fire frames are 2 + 4096 bytes. Without it, every frame would read a whole extra 512-byte block for 2 bytes. |
| Vertical clipping skips whole blocks | Rows above or below the screen are never read from the card. |
| `drawSpriteFile(path, …)` kept | Simon's original one-shot API still works. It falls back to the normal File API for fragmented files. |

#### Sketch and helpers

| Change | Where | Why |
|---|---|---|
| `SD.begin()` once in `setup()` | `CHSpriteView.ino` | |
| No per-frame `Gfx.clear()`; the static sprite and title are drawn once | `CHSpriteView.ino` | Only the fire band changes. |
| 12 bpp panel mode by default | `CHSpriteView.ino` | 1.5 bytes per pixel to the LCD instead of 2. With a 16-colour palette there's no visible cost. |
| Async flush of only the changed rows | `CHSpriteView.ino` | |
| On-screen and serial instrumentation, live A/B toggles | `CHSpriteView.ino` | |
| Fixed uninitialised button mask, inverted `justReleased()`, 64-bit timer | `src/CHGame.cpp`, `src/CHGame.h` | Bug fixes. |
| `bench.py` | | Builds a variant, uploads it, and captures the serial report. Every number below came from it. |

---

## Results

All measured on the device: same board, same SDHC card, same 16 frames,
uncapped.

### How each step moved it

| Build | fps | SD read / frame | LCD / frame |
|---|---:|---:|---:|
| Simon's original (`-O3`) | **2** | 621 ms | not measured |
| Original + `SD.begin()` once in `setup()` only | 6 | 173 ms | not measured |
| This version, but stock SPI-library SD transport | 16 | 60.7 ms | 1.8 ms |
| This version, first cut (byte-wise diff, one dirty box) | 203 | 2.87 ms | 2.0 ms |
| + word-wide copy/diff | 221 | 2.44 ms | 2.05 ms |
| + dirty region in 16-row bands | 231 | 2.44 ms | 1.82 ms |
| **+ 8-row bands (final)** | **232** | **2.45 ms** | **1.80 ms** |

The obvious one-line fix ("call `SD.begin()` once") only gets 3×. The SD
transport rewrite is what makes the difference: the same sprite code on the
stock transport manages 16 fps.

### Display modes (final build)

| Panel mode | Flush | Pixels sent | LCD / frame | fps |
|---|---|---:|---:|---:|
| **12 bpp** | **changed rows only** | **~2,900** | **1.80 ms** | **232** |
| 16 bpp | changed rows only | ~2,900 | 2.25 ms | 209 |
| 12 bpp | whole 128×64 band | 8,192 | 4.25 ms | 147 |
| 16 bpp | whole 128×64 band | 8,192 | 5.55 ms | 123 |
| 12 bpp, 60 fps cap | changed rows only | ~2,900 | 1.05 ms | 62.5 |

`CHGame::setFrameRate(60)` works in whole milliseconds (1000/60 = 16 ms),
so "60" is really 62.5. With idle time between reads, this card also
responds faster: SD time falls to 1.5 ms.

### Where the 4.3 ms goes (`-DSD_PROFILE`)

```
SD   CMD18 command                          0.03 ms
     card access latency to 1st block       0.58 ms   <- the card's own
     gaps between the 8 blocks              0.27 ms   <- the card's own
     8 blocks by DMA at 24 MHz              1.37 ms   <- the wire
     copy/compare of last block             0.10 ms
     CMD12                                  0.02 ms
LCD  ~2900 px x 1.5 B at 24 MHz + windows   1.80 ms   <- the wire
                                           ---------
                                            ~4.3 ms  = 232 fps
```

About 0.85 ms is the card's own latency and 3.2 ms is bytes moving on a
shared 24 MHz bus. What remains in software is about 0.15 ms: this is at the
hardware's limit for this card.

### Build size

| Optimisation | Flash | Static RAM | fps |
|---|---:|---:|---:|
| `-Os` | 26,948 B (52%) | 13,920 B | 215 |
| `-O1` | 30,744 B (60%) | 14,780 B | 231 |
| **`-O2`** | **31,324 B (61%)** | **14,812 B** | **232** |
| `-O3` | 48,408 B (95%) | 15,244 B | not worth running |

About 3 KB of this is the on-screen HUD and serial report. The SD library
plus SPI code is ~1.1 KB *smaller* than the stock pair.

### Tried and rejected

- **Prefetch.** Send the next frame's `CMD18` early, deselect the card, and
  let the LCD use the bus while the card fetches. On this card about half
  the reads failed, and the rest still waited ~480 µs, because the card
  does not fetch while deselected. The SD spec requires chip select to stay
  low for the whole read, so this is card-dependent by nature.
- **Row-copy code in SRAM.** 233 vs 231 fps, within noise. It stays in
  flash and saves 516 B of RAM; only the per-byte compare loop runs from
  SRAM.
- **4-row bands.** They send fewer pixels (2,830) but pay for more
  rectangles: 230 fps.

---

## Using it in your own sketch

```cpp
#include "src/CHSpriteView.h"

SpriteFile hero;

void setup() {
    Gfx.begin(GFX_DIV2, GFX_12BPP);
    SD.begin(PIN_SD_CS);                    // once
    spriteLoad(hero, "HERO.BIN");           // once per sprite
}

void loop() {
    SpriteDirty d;
    spriteDirtyReset(d);
    spriteDraw(hero, x, y, -1, &d);         // -1 = opaque, 0..15 = transparent index
    spriteFlushDirty(d);                    // send only what changed
}
```

The sprite file format is unchanged: byte 0 = width, byte 1 = height, then
`height` rows of `ceil(width / 2)` bytes. That is 2 pixels per byte, with
even x in the low nibble, the same packing as the CHGfx framebuffer.

The fastest path, a direct copy with change tracking, applies to opaque
sprites at an even x with an even width that fit horizontally on screen.
Everything else goes through `gfx_blit()` a row at a time, and its whole
visible area is reported as changed.

## Troubleshooting

- **"SD mount failed"**: check the card is FAT16/32 and seated. Mount tries
  24, then 12, then 6 MHz before giving up.
- **Serial shows `DMA off`**: a DMA transfer timed out once and everything
  has fallen back to polled SPI. It still works, just slower.
- **`err` counter rising**: card reads failing mid-frame. Try another card,
  or build with `-DSD_CH32_DISABLE_FAST` to compare against the stock
  transport.

## Credits

Original CHSpriteView by Simon (filmote). SD library: Arduino SD 1.3.0
(SparkFun / William Greiman sdfatlib), GPL v3.
