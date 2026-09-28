/* Arduino Sd2Card Library
   Copyright (C) 2009 by William Greiman

   This file is part of the Arduino Sd2Card Library

   This Library is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with the Arduino Sd2Card Library.  If not, see
   <http://www.gnu.org/licenses/>.
*/
/*
 * CHGAME: this file is where nearly all of the speed-up lives. Read the
 * banner in Sd2Card.h first for the why. In short:
 *
 *   - "Transport" (the first section below) is new for the CH32X035: SPI1
 *     is driven by register, bulk reads go through DMA, and the bus is
 *     borrowed from / returned to the LCD driver around every transaction.
 *     The stock SPI-library and software-SPI transports are kept intact
 *     under #if for other boards.
 *   - Everything from cardCommand() down is the original sdfatlib protocol
 *     code. Where it moved bytes one spiRec() at a time over a buffer, it
 *     now calls spiRecvBulk(), which is one DMA transfer on the CH32.
 *   - readStart()/readStream()/readStop()/readBlocksPipelined() at the end
 *     are new: CMD18 multi-block streaming.
 *
 * Originally this file began with "#define USE_SPI_LIB". That decision now
 * lives in Sd2Card.h, which knows whether the CH32 transport is in use.
 */
#include <Arduino.h>
#include "Sd2Card.h"

#if SD_CH32_FAST
//==============================================================================
// CHGAME: CH32X035 TRANSPORT
//==============================================================================
extern "C" {
#include "ch32x035.h"
}

/*
 * Register bits used below. Named locally, in the same style as CHGfx, so
 * the code does not depend on which of the vendor header's many aliases a
 * given SDK version spells out.
 */
/* SPI1 CTLR1 (RM 16.4.1) */
#define SDSPI_MSTR     (1u << 2)            /* master mode                     */
#define SDSPI_BR(n)    ((uint16_t)((n) & 7u) << 3)  /* SCK = HCLK / 2^(n+1)    */
#define SDSPI_BR_MASK  (7u << 3)
#define SDSPI_SPE      (1u << 6)            /* peripheral enable               */
#define SDSPI_SSI      (1u << 8)            /* internal NSS level (held high)  */
#define SDSPI_SSM      (1u << 9)            /* software NSS: CS is our GPIO    */
/* SPI1 CTLR2 */
#define SDSPI_RXDMAEN  (1u << 0)
#define SDSPI_TXDMAEN  (1u << 1)
/* SPI1 STATR */
#define SDSPI_RXNE     (1u << 0)
#define SDSPI_TXE      (1u << 1)
#define SDSPI_BSY      (1u << 7)
/* DMA channel CFGR (RM 9.4) */
#define SDDMA_EN       (1u << 0)
#define SDDMA_DIR_M2P  (1u << 4)            /* memory -> peripheral            */
#define SDDMA_MINC     (1u << 7)            /* memory address increments       */
#define SDDMA_PL_HIGH  (2u << 12)
#define SDDMA_PL_VHIGH (3u << 12)
/* DMA1 INTFR / INTFCR: four flags per channel (GIF, TCIF, HTIF, TEIF),
   channel n at bit 4*(n-1). */
#define SDDMA_CH2_ALL  (0xFu << 4)
#define SDDMA_CH2_TCIF (1u << 5)
#define SDDMA_CH2_TEIF (1u << 7)
#define SDDMA_CH3_ALL  (0xFu << 8)

/* SD identification must run at 100-400 kHz. BR = 7 is HCLK/256, which is
   187.5 kHz at 48 MHz. */
#define SD_BR_IDENTIFY 7

/* Transfers shorter than this are cheaper to poll than to set up DMA for
   (DMA setup is ~20 register writes, a polled byte is ~20 cycles). */
#ifndef SD_DMA_MIN_BYTES
  #define SD_DMA_MIN_BYTES 16
#endif

/*
 * Hot loops run from SRAM, same trick and same section name as CHGfx: the
 * core's linker script routes *(.srodata*) into .data, which startup copies
 * from flash to RAM. Flash on this part is 3 wait states at 48 MHz, which
 * would otherwise more than double the per-byte cost of the polled loop.
 */
#define SD_RAMFUNC __attribute__((section(".srodata.sdramfunc"), noinline))

/* --- transport state (one card per sketch, as with stock SD) ------------- */
static GPIO_TypeDef* s_csPort;          /* chip-select port, from the pin map  */
static uint32_t      s_csBit;           /* chip-select bit mask                */
static uint8_t       s_br = SD_BR_IDENTIFY;  /* BR field for card traffic      */
static uint8_t       s_busOwned;        /* SD currently holds SPI1             */
static uint16_t      s_hostCtlr1;       /* SPI1 CTLR1 to restore on release    */
static uint8_t       s_dmaDead;         /* a DMA transfer failed; poll forever */
static uint8_t       s_txFiller = 0xFF; /* DMA source for the 0xFF clock bytes */
static uint8_t       s_rxSink;          /* DMA destination for skipped bytes   */

/* Direct BSHR/BCR writes: single store, no read-modify-write, no pin-map
   walk (digitalWrite() on this core looks the pin up every call). */
#define SD_CS_LOW()   (s_csPort->BCR  = s_csBit)
#define SD_CS_HIGH()  (s_csPort->BSHR = s_csBit)

/*
 * SPI1 pins. CHGfx configures SCK and MOSI but leaves MISO at its reset
 * default (floating input) because the panel never talks back. Configure
 * all three here so this library also works on its own, with MISO pulled
 * up so an empty slot reads 0xFF (= "busy/no data") rather than noise.
 *
 * All three are in CFGLR (pins 0-7), which reads back correctly, so a
 * read-modify-write is safe - unlike CFGHR, whose shadow problem is covered
 * in CHGfx.cpp.
 *   PA5 SCK  : 0xB  alternate-function push-pull, 50 MHz
 *   PA6 MISO : 0x8  input with pull-up/down; OUTDR bit 6 = 1 selects up
 *   PA7 MOSI : 0xB  alternate-function push-pull, 50 MHz
 */
static void spiPinsInit(void) {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1;
  RCC->AHBPCENR  |= RCC_AHBPeriph_DMA1;
  GPIOA->CFGLR = (GPIOA->CFGLR & ~((0xFu << 20) | (0xFu << 24) | (0xFu << 28)))
                 | (0xBu << 20) | (0x8u << 24) | (0xBu << 28);
  GPIOA->BSHR = 1u << 6;
}

/*
 * Bus hand-off. SPI1 is shared with the ST7735, and the two sides want the
 * peripheral set up differently, so every card transaction is bracketed:
 *
 *   busClaim()   wait for the bus to go idle, remember the host's CTLR1,
 *                program the card's settings
 *   busRelease() put the host's CTLR1 back exactly as it was
 *
 * Neither side has to know the other's configuration. The old sketch-level
 * sdEnd() hard-coded CHGfx's register values; this does not need to.
 */
static void busClaim(void) {
  if (s_busOwned) {
    return;
  }
  /* 1. An async CHGfx flush owns the bus while its DMA runs. CHGfx keeps
        TXDMAEN set from the first chunk to the last (its ISR re-arms chunk
        after chunk) and clears it only once the final chunk has landed, so
        TXDMAEN is an exact "panel transfer in flight" flag. Waiting on it
        makes SD calls safe even if the sketch forgot gfx_wait(). */
  while (SPI1->CTLR2 & SDSPI_TXDMAEN) { }
  /* 2. Let the last byte finish shifting out. */
  while (!(SPI1->STATR & SDSPI_TXE)) { }
  while (SPI1->STATR & SDSPI_BSY) { }

  /* 3. Swap in the card's setup: master, mode 0, MSB first, 8-bit frames,
        software NSS. BR and DFF may only change while SPE = 0 (RM 16.4.1). */
  s_hostCtlr1 = SPI1->CTLR1;
  SPI1->CTLR1 = s_hostCtlr1 & ~SDSPI_SPE;
  SPI1->CTLR2 = 0;
  SPI1->CTLR1 = SDSPI_MSTR | SDSPI_SSM | SDSPI_SSI | SDSPI_BR(s_br);
  SPI1->CTLR1 |= SDSPI_SPE;

  /* 4. The LCD only transmits, so the receive side is full of stale data
        with the overrun flag set. Reading DATAR then STATR clears both
        (RM 16.2.9); otherwise the first byte "received" would be old. */
  (void)SPI1->DATAR;
  (void)SPI1->STATR;
  s_busOwned = 1;
}

static void busRelease(void) {
  if (!s_busOwned) {
    return;
  }
  while (SPI1->STATR & SDSPI_BSY) { }
  SPI1->CTLR1 &= ~SDSPI_SPE;
  SPI1->CTLR2 = 0;
  SPI1->CTLR1 = s_hostCtlr1 & ~SDSPI_SPE;
  SPI1->CTLR1 = s_hostCtlr1;          /* re-enables only if it was enabled */
  s_busOwned = 0;
}

/*
 * One full-duplex byte. Write, wait for the byte to come back, read it.
 * Precondition: RXNE clear, which busClaim() and every function below
 * leave true (each byte clocked out is read back before the next is sent).
 */
static inline __attribute__((always_inline)) uint8_t spiXfer(uint8_t b) {
  SPI1->DATAR = b;
  while (!(SPI1->STATR & SDSPI_RXNE)) { }
  return (uint8_t)SPI1->DATAR;
}

/** Send a byte to the card */
static inline void spiSend(uint8_t b) {
  (void)spiXfer(b);
}
/** Receive a byte from the card */
static inline uint8_t spiRec(void) {
  return spiXfer(0XFF);
}

/* Polled bulk receive, from SRAM. Used for short transfers and as the
   fallback if DMA ever fails. dst == NULL discards. */
static SD_RAMFUNC void rxPolled(uint8_t* dst, uint32_t n) {
  if (dst) {
    while (n--) {
      *dst++ = spiXfer(0XFF);
    }
  } else {
    while (n--) {
      (void)spiXfer(0XFF);
    }
  }
}

/*
 * DMA bulk receive, split into start and wait so readBlocksPipelined() can
 * do useful work in between.
 *
 * SPI master receive needs clocks, and clocks only come from transmitting,
 * so two channels run together:
 *
 *   DMA1_CH3 (SPI1_TX): memory -> DATAR, MINC off: sends s_txFiller (0xFF)
 *                       n times - the card needs MOSI high while it talks.
 *   DMA1_CH2 (SPI1_RX): DATAR -> memory, MINC on: stores each byte that
 *                       arrives. MINC off plus a 1-byte sink = skip bytes.
 *
 * RX gets the higher priority so it is never starved while TX runs ahead
 * (TX can be at most two bytes ahead: one in the shift register, one in
 * the buffer), which rules out overrun. Neither channel enables an
 * interrupt: CHGfx has an ISR on channel 3, and with TCIE clear it is
 * never invoked for these transfers. Completion is polled from TCIF2 -
 * the RX side finishing means every byte has been clocked AND stored.
 *
 * CHGfx programs channel 3 from scratch (CFGR, MADDR, CNTR) on every
 * transfer and sets PADDR to the same &SPI1->DATAR we use, so borrowing
 * the channel leaves nothing behind that it depends on.
 */
static void rxDmaStart(uint8_t* dst, uint16_t n) {
  DMA1_Channel2->CFGR = 0;
  DMA1_Channel3->CFGR = 0;
  DMA1->INTFCR = SDDMA_CH2_ALL | SDDMA_CH3_ALL;

  DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
  DMA1_Channel2->MADDR = (uint32_t)(dst ? dst : &s_rxSink);
  DMA1_Channel2->CNTR  = n;
  DMA1_Channel2->CFGR  = SDDMA_PL_VHIGH | (dst ? SDDMA_MINC : 0);

  DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;
  DMA1_Channel3->MADDR = (uint32_t)&s_txFiller;
  DMA1_Channel3->CNTR  = n;
  DMA1_Channel3->CFGR  = SDDMA_DIR_M2P | SDDMA_PL_HIGH;

  /* Arm both channels, THEN raise the SPI requests. TXE is already set, so
     the TX request fires immediately and starts the clock; RX must be
     listening before that first byte completes. */
  DMA1_Channel2->CFGR |= SDDMA_EN;
  DMA1_Channel3->CFGR |= SDDMA_EN;
  SPI1->CTLR2 = SDSPI_RXDMAEN | SDSPI_TXDMAEN;
}

/* Wait for the transfer rxDmaStart() began. Returns false (and turns DMA
   off for the rest of the session) if it does not finish. The bound is
   generous: 512 poll iterations per byte covers even the slowest clock. */
static uint8_t rxDmaWait(uint16_t n) {
  uint32_t spin = (uint32_t)n * 512u + 4096u;
  uint8_t ok = 1;
  while (!(DMA1->INTFR & SDDMA_CH2_TCIF)) {
    if ((DMA1->INTFR & SDDMA_CH2_TEIF) || --spin == 0) {
      ok = 0;
      break;
    }
  }
  SPI1->CTLR2 = 0;
  DMA1_Channel2->CFGR = 0;
  DMA1_Channel3->CFGR = 0;
  DMA1->INTFCR = SDDMA_CH2_ALL | SDDMA_CH3_ALL;
  if (!ok) {
    s_dmaDead = 1;
    /* Leave the receive side clean for the polled path. */
    while (SPI1->STATR & SDSPI_BSY) { }
    (void)SPI1->DATAR;
    (void)SPI1->STATR;
  }
  return ok;
}

/*
 * The one entry point the protocol code uses for "clock in n bytes".
 * DMA for anything worth it, polling otherwise. dst == NULL discards.
 */
static uint8_t spiRecvBulk(uint8_t* dst, uint16_t n) {
  if (n >= SD_DMA_MIN_BYTES && !s_dmaDead) {
    rxDmaStart(dst, n);
    return rxDmaWait(n);
  }
  rxPolled(dst, n);
  return true;
}

uint8_t Sd2Card::dmaEnabled(void) {
  return !s_dmaDead;
}

#elif !defined(SOFTWARE_SPI)
//==============================================================================
// Stock transport: Arduino SPI library (unchanged apart from spiRecvBulk)
//==============================================================================
#ifdef USE_SPI_LIB

  #ifndef SDCARD_SPI
    #define SDCARD_SPI SPI
  #endif

  #include <SPI.h>
  static SPISettings settings;
#endif
// functions for hardware SPI
/** Send a byte to the card */
static void spiSend(uint8_t b) {
  #ifndef USE_SPI_LIB
  SPDR = b;
  while (!(SPSR & (1 << SPIF)))
    ;
  #else
  SDCARD_SPI.transfer(b);
  #endif
}
/** Receive a byte from the card */
static  uint8_t spiRec(void) {
  #ifndef USE_SPI_LIB
  spiSend(0XFF);
  return SPDR;
  #else
  return SDCARD_SPI.transfer(0xFF);
  #endif
}
#else  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/** nop to tune soft SPI timing */
#define nop asm volatile ("nop\n\t")
//------------------------------------------------------------------------------
/** Soft SPI receive */
uint8_t spiRec(void) {
  uint8_t data = 0;
  // no interrupts during byte receive - about 8 us
  cli();
  // output pin high - like sending 0XFF
  fastDigitalWrite(SPI_MOSI_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, HIGH);

    // adjust so SCK is nice
    nop;
    nop;

    data <<= 1;

    if (fastDigitalRead(SPI_MISO_PIN)) {
      data |= 1;
    }

    fastDigitalWrite(SPI_SCK_PIN, LOW);
  }
  // enable interrupts
  sei();
  return data;
}
//------------------------------------------------------------------------------
/** Soft SPI send */
void spiSend(uint8_t data) {
  // no interrupts during byte send - about 8 us
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, LOW);

    fastDigitalWrite(SPI_MOSI_PIN, data & 0X80);

    data <<= 1;

    fastDigitalWrite(SPI_SCK_PIN, HIGH);
  }
  // hold SCK high for a few ns
  nop;
  nop;
  nop;
  nop;

  fastDigitalWrite(SPI_SCK_PIN, LOW);
  // enable interrupts
  sei();
}
#endif  // SOFTWARE_SPI

#if !SD_CH32_FAST
/* CHGAME: the byte-at-a-time equivalent of the CH32 bulk receive, so the
   shared protocol code below reads the same on every transport. */
static uint8_t spiRecvBulk(uint8_t* dst, uint16_t n) {
  if (dst) {
    while (n--) {
      *dst++ = spiRec();
    }
  } else {
    while (n--) {
      spiRec();
    }
  }
  return true;
}

uint8_t Sd2Card::dmaEnabled(void) {
  return false;
}
#endif

//==============================================================================
// SD PROTOCOL
//==============================================================================
//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  // end read if in partialBlockRead mode
  readEnd();

  // CHGAME: a command in the middle of a CMD18 stream would be read as data
  // by nobody and ignored by the card; close the stream properly first.
  // readStop() clears streaming_ before it sends its own CMD12, so this
  // does not recurse.
  if (streaming_ && cmd != CMD12) {
    readStop();
  }

  // select card
  chipSelectLow();

  // wait up to 300 ms if busy
  // CHGAME: except for CMD12. STOP_TRANSMISSION is sent while the card is
  // still streaming data, so MISO is carrying data bytes, not a busy
  // indication; waiting for 0xFF there would only burn time reading data
  // nobody wants. (SdFat skips the wait for CMD12 for the same reason.)
  if (cmd != CMD12) {
    waitNotBusy(300);
  }

  // send command
  spiSend(cmd | 0x40);

  // send argument
  for (int8_t s = 24; s >= 0; s -= 8) {
    spiSend(arg >> s);
  }

  // send CRC
  uint8_t crc = 0XFF;
  if (cmd == CMD0) {
    crc = 0X95;  // correct crc for CMD0 with arg 0
  }
  if (cmd == CMD8) {
    crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
  }
  spiSend(crc);

  // CHGAME: the byte after CMD12 is a "stuff byte" (SD spec 7.3.1.3) and
  // must be discarded before looking for the R1 response.
  if (cmd == CMD12) {
    spiRec();
  }

  // wait for response
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++)
    ;
  return status_;
}
//------------------------------------------------------------------------------
/**
   Determine the size of an SD flash memory card.

   \return The number of 512 byte data blocks in the card
           or zero if an error occurs.
*/
uint32_t Sd2Card::cardSize(void) {
  csd_t csd;
  if (!readCSD(&csd)) {
    return 0;
  }
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}
//------------------------------------------------------------------------------
#if SD_CH32_FAST
/*
 * CHGAME: chip select also claims/releases SPI1 from the LCD. The claim is
 * idempotent, so the protocol code can call chipSelectLow() as often as it
 * likes; the first call of a transaction does the swap.
 */
void Sd2Card::chipSelectHigh(void) {
  if (s_csPort) {                 // nothing to do before init() has run
    SD_CS_HIGH();
  }
  if (s_busOwned) {
    // 8 clocks with CS high so the card releases MISO (SD spec 6.4.1.1).
    spiXfer(0XFF);
    busRelease();
  }
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
  if (!s_csPort) {                // card used before init(): refuse quietly
    return;
  }
  busClaim();
  SD_CS_LOW();
}
#else
static uint8_t chip_select_asserted = 0;

void Sd2Card::chipSelectHigh(void) {
  digitalWrite(chipSelectPin_, HIGH);
  #ifdef USE_SPI_LIB
  if (chip_select_asserted) {
    chip_select_asserted = 0;
    SDCARD_SPI.endTransaction();
  }
  #endif
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
  #ifdef USE_SPI_LIB
  if (!chip_select_asserted) {
    chip_select_asserted = 1;
    SDCARD_SPI.beginTransaction(settings);
  }
  #endif
  digitalWrite(chipSelectPin_, LOW);
}
#endif
//------------------------------------------------------------------------------
/** Erase a range of blocks.

   \param[in] firstBlock The address of the first block in the range.
   \param[in] lastBlock The address of the last block in the range.

   \note This function requests the SD card to do a flash erase for a
   range of blocks.  The data on the card after an erase operation is
   either 0 or 1, depends on the card vendor.  The card must support
   single block erase.

   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
  if (!eraseSingleBlockEnable()) {
    error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
    goto fail;
  }
  if (type_ != SD_CARD_TYPE_SDHC) {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock)
      || cardCommand(CMD33, lastBlock)
      || cardCommand(CMD38, 0)) {
    error(SD_CARD_ERROR_ERASE);
    goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.

   \return The value one, true, is returned if single block erase is supported.
   The value zero, false, is returned if single block erase is not supported.
*/
uint8_t Sd2Card::eraseSingleBlockEnable(void) {
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
   Initialize an SD flash memory card.

   \param[in] sckRateID SPI clock rate selector. See setSckRate().
   \param[in] chipSelectPin SD chip select pin number.

   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.  The reason for failure
   can be determined by calling errorCode() and errorData().
*/
uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
  errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
  streaming_ = crcPending_ = 0;   // CHGAME: forget any half-finished stream
  blockRemain_ = 0;
  chipSelectPin_ = chipSelectPin;
  // 16-bit init start time allows over a minute
  unsigned int t0 = millis();
  uint32_t arg;

  #if SD_CH32_FAST
  // CHGAME: resolve the Arduino pin number to a port and bit once, so the
  // hot path can toggle CS with one store. Level first, then pinMode(): the
  // pin goes output already high, and pinMode() goes through the core's
  // GPIO_Init(), which keeps the CFGHR shadow correct for PB8..PB15 (the
  // CHGfx.cpp comment explains why that matters for LCD_RST on PB12).
  s_csPort = digitalPinToPort(chipSelectPin_);
  s_csBit  = digitalPinToBitMask(chipSelectPin_);
  SD_CS_HIGH();
  pinMode(chipSelectPin_, OUTPUT);
  SD_CS_HIGH();
  spiPinsInit();

  // Identification runs at ~187 kHz; setSckRate() at the end raises it.
  s_br = SD_BR_IDENTIFY;

  // must supply min of 74 clock cycles with CS high.
  busClaim();
  for (uint8_t i = 0; i < 10; i++) {
    spiSend(0XFF);
  }
  busRelease();
  #else
  // set pin modes
  pinMode(chipSelectPin_, OUTPUT);
  digitalWrite(chipSelectPin_, HIGH);
  #ifndef USE_SPI_LIB
  pinMode(SPI_MISO_PIN, INPUT);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);
  #endif

  #ifndef SOFTWARE_SPI
  #ifndef USE_SPI_LIB
  // SS must be in output mode even it is not chip select
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // disable any SPI device using hardware SS pin
  // Enable SPI, Master, clock rate f_osc/128
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
  // clear double speed
  SPSR &= ~(1 << SPI2X);
  #else // USE_SPI_LIB
  SDCARD_SPI.begin();
  settings = SPISettings(250000, MSBFIRST, SPI_MODE0);
  #endif // USE_SPI_LIB
  #endif // SOFTWARE_SPI

  // must supply min of 74 clock cycles with CS high.
  #ifdef USE_SPI_LIB
  SDCARD_SPI.beginTransaction(settings);
  #endif
  for (uint8_t i = 0; i < 10; i++) {
    spiSend(0XFF);
  }
  #ifdef USE_SPI_LIB
  SDCARD_SPI.endTransaction();
  #endif
  #endif // SD_CH32_FAST

  chipSelectLow();

  // command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    unsigned int d = millis() - t0;
    if (d > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }
  // check SD version
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
    type(SD_CARD_TYPE_SD1);
  } else {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++) {
      status_ = spiRec();
    }
    if (status_ != 0XAA) {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  // initialize card and send host supports SDHC if SD2
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    // check for timeout
    unsigned int d = millis() - t0;
    if (d > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  // if SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((spiRec() & 0XC0) == 0XC0) {
      type(SD_CARD_TYPE_SDHC);
    }
    // discard rest of ocr - contains allowed voltage range
    for (uint8_t i = 0; i < 3; i++) {
      spiRec();
    }
  }
  chipSelectHigh();

  #ifndef SOFTWARE_SPI
  return setSckRate(sckRateID);
  #else  // SOFTWARE_SPI
  return true;
  #endif  // SOFTWARE_SPI

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
   Enable or disable partial block reads.

   Enabling partial block reads improves performance by allowing a block
   to be read over the SPI bus as several sub-blocks.  Errors may occur
   if the time between reads is too long since the SD card may timeout.
   The SPI SS line will be held low until the entire block is read or
   readEnd() is called.

   Use this for applications like the Adafruit Wave Shield.

   \param[in] value The value TRUE (non-zero) or FALSE (zero).)
*/
void Sd2Card::partialBlockRead(uint8_t value) {
  readEnd();
  partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
   Read a 512 byte block from an SD card device.

   \param[in] block Logical block to be read.
   \param[out] dst Pointer to the location that will receive the data.

   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
   Read part of a 512 byte block from an SD card.

   \param[in] block Logical block to be read.
   \param[in] offset Number of bytes to skip at start of block
   \param[out] dst Pointer to the location that will receive the data.
   \param[in] count Number of bytes to read
   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::readData(uint32_t block,
                          uint16_t offset, uint16_t count, uint8_t* dst) {
  if (count == 0) {
    return true;
  }
  if ((count + offset) > 512) {
    goto fail;
  }
  if (!inBlock_ || block != block_ || offset < offset_) {
    block_ = block;
    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC) {
      block <<= 9;
    }
    if (cardCommand(CMD17, block)) {
      error(SD_CARD_ERROR_CMD17);
      goto fail;
    }
    if (!waitStartBlock()) {
      goto fail;
    }
    offset_ = 0;
    inBlock_ = 1;
  }

  // CHGAME: was two per-byte spiRec() loops (skip to offset, then copy).
  // Each is now a single bulk transfer - DMA on the CH32 - and the skip
  // uses the discard form so no buffer is needed for it.
  if (offset_ < offset) {
    if (!spiRecvBulk(NULL, offset - offset_)) {
      error(SD_CARD_ERROR_DMA);
      goto fail;
    }
    offset_ = offset;
  }
  if (!spiRecvBulk(dst, count)) {
    error(SD_CARD_ERROR_DMA);
    goto fail;
  }

  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512) {
    // read rest of data, checksum and set chip select high
    readEnd();
  }
  return true;

fail:
  inBlock_ = 0;   // CHGAME: a failed read must not look like an open block
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void) {
  if (inBlock_) {
    // skip data and crc
    // CHGAME: one bulk discard of (rest of block + 2 CRC bytes) instead of
    // a spiRec() loop. The original counted offset_ up to 514 the same way.
    spiRecvBulk(NULL, 514 - offset_);
    chipSelectHigh();
    inBlock_ = 0;
  }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }
  if (!waitStartBlock()) {
    goto fail;
  }
  // transfer data
  for (uint16_t i = 0; i < 16; i++) {
    dst[i] = spiRec();
  }
  spiRec();  // get first crc byte
  spiRec();  // get second crc byte
  chipSelectHigh();
  return true;

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
   Set the SPI clock rate.

   \param[in] sckRateID A value in the range [0, 6].

   The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
   SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
   for \a scsRateID = 6.

   CHGAME: on the CH32 that formula is exact, because it is literally the
   SPI1 BR field: 0 = 24 MHz, 1 = 12 MHz, 2 = 6 MHz ... 6 = 375 kHz.

   \return The value one, true, is returned for success and the value zero,
   false, is returned for an invalid value of \a sckRateID.
*/
uint8_t Sd2Card::setSckRate(uint8_t sckRateID) {
  if (sckRateID > 6) {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }
  sckRateId_ = sckRateID;
  #if SD_CH32_FAST
  // Takes effect at the next busClaim(), i.e. the next transaction.
  s_br = sckRateID;
  #elif !defined(USE_SPI_LIB)
  // see avr processor datasheet for SPI register bit definitions
  if ((sckRateID & 1) || sckRateID == 6) {
    SPSR &= ~(1 << SPI2X);
  } else {
    SPSR |= (1 << SPI2X);
  }
  SPCR &= ~((1 << SPR1) | (1 << SPR0));
  SPCR |= (sckRateID & 4 ? (1 << SPR1) : 0)
          | (sckRateID & 2 ? (1 << SPR0) : 0);
  #else // USE_SPI_LIB
  switch (sckRateID) {
    case 0:  settings = SPISettings(25000000, MSBFIRST, SPI_MODE0); break;
    case 1:  settings = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;
    case 2:  settings = SPISettings(2000000, MSBFIRST, SPI_MODE0); break;
    case 3:  settings = SPISettings(1000000, MSBFIRST, SPI_MODE0); break;
    case 4:  settings = SPISettings(500000, MSBFIRST, SPI_MODE0); break;
    case 5:  settings = SPISettings(250000, MSBFIRST, SPI_MODE0); break;
    default: settings = SPISettings(125000, MSBFIRST, SPI_MODE0);
  }
  #endif // USE_SPI_LIB
  return true;
}
//------------------------------------------------------------------------------
// set the SPI clock frequency
uint8_t Sd2Card::setSpiClock(uint32_t clock) {
  #ifdef USE_SPI_LIB
  settings = SPISettings(clock, MSBFIRST, SPI_MODE0);
  return true;
  #else
  // CHGAME: fastest F_CPU/2^(id+1) that does not exceed the request.
  uint8_t id = 0;
  while (id < 6 && (F_CPU >> (id + 1)) > clock) {
    id++;
  }
  return setSckRate(id);
  #endif
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(unsigned int timeoutMillis) {
  unsigned int t0 = millis();
  unsigned int d;
  do {
    if (spiRec() == 0XFF) {
      return true;
    }
    d = millis() - t0;
  } while (d < timeoutMillis);
  return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void) {
  unsigned int t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    unsigned int d = millis() - t0;
    if (d > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  return true;

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
   Writes a 512 byte block to an SD card.

   \param[in] blockNumber Logical block to be written.
   \param[in] src Pointer to the location of the data to be written.
   \param[in] blocking If the write should be blocking.
   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src, uint8_t blocking) {
  #if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
  #endif  // SD_PROTECT_BLOCK_ZERO

  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) {
    blockNumber <<= 9;
  }
  if (cardCommand(CMD24, blockNumber)) {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) {
    goto fail;
  }
  if (blocking) {
    // wait for flash programming to complete
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
      error(SD_CARD_ERROR_WRITE_TIMEOUT);
      goto fail;
    }
    // response is r2 so get and check two bytes for nonzero
    if (cardCommand(CMD13, 0) || spiRec()) {
      error(SD_CARD_ERROR_WRITE_PROGRAMMING);
      goto fail;
    }
  }
  chipSelectHigh();
  return true;

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t* src) {
  // wait for previous write to finish
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_MULTIPLE);
    chipSelectHigh();
    return false;
  }
  return writeData(WRITE_MULTIPLE_TOKEN, src);
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
// CHGAME: writes are left on the polled path. They are rare in a game (save
// files), the card's programming time dwarfs the transfer anyway, and the
// fast spiSend() is already ~10x the old per-byte cost.
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src) {
  spiSend(token);
  for (uint16_t i = 0; i < 512; i++) {
    spiSend(src[i]);
  }
  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc

  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }
  return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.

   \param[in] blockNumber Address of first block in sequence.
   \param[in] eraseCount The number of blocks to be pre-erased.

   \note This function is used with writeData() and writeStop()
   for optimized multiple block writes.

   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
  #if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
  #endif  // SD_PROTECT_BLOCK_ZERO
  // send pre-erase count
  if (cardAcmd(ACMD23, eraseCount)) {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) {
    blockNumber <<= 9;
  }
  if (cardCommand(CMD25, blockNumber)) {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  return true;

fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.

  \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t Sd2Card::writeStop(void) {
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    goto fail;
  }
  spiSend(STOP_TRAN_TOKEN);
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    goto fail;
  }
  chipSelectHigh();
  return true;

fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Check if the SD card is busy

  \return The value one, true, is returned when is busy and
   the value zero, false, is returned for when is NOT busy.
*/
uint8_t Sd2Card::isBusy(void) {
  chipSelectLow();
  byte b = spiRec();
  chipSelectHigh();

  return (b != 0XFF);
}

//==============================================================================
// CHGAME: MULTI-BLOCK STREAMING (CMD18)
//==============================================================================
/*
 * How a CMD18 stream looks on MISO, and what the three state fields track:
 *
 *   CMD18 -> R1 | 0xFF.. 0xFE [512 data] [CRC CRC] | 0xFF.. 0xFE [512] [CRC CRC] | ...
 *                  ^ access    ^ token    ^ blockRemain_ ^ crcPending_
 *                    latency
 *
 * blockRemain_ counts the data bytes still to come in the current block;
 * 0 means the next thing on the wire is (CRC of the previous block, if
 * crcPending_) followed by 0xFF filler and the next start token. The
 * card keeps sending blocks until it gets CMD12.
 */
//------------------------------------------------------------------------------
/**
   CHGAME: begin streaming consecutive blocks from \a block.
   The card is left selected (holding SPI1) until readStop().
*/
uint8_t Sd2Card::readStart(uint32_t block) {
  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) {
    block <<= 9;
  }
  if (cardCommand(CMD18, block)) {
    error(SD_CARD_ERROR_CMD18);
    chipSelectHigh();
    return false;
  }
  streaming_ = 1;
  crcPending_ = 0;
  blockRemain_ = 0;
  return true;
}
//------------------------------------------------------------------------------
/**
   CHGAME: the next \a count bytes of the stream into \a dst, or skip them
   if \a dst is NULL. May be called any number of times with any sizes; it
   handles the token and CRC at each 512-byte boundary.
*/
uint8_t Sd2Card::readStream(uint8_t* dst, uint32_t count) {
  if (!streaming_) {
    return false;
  }
  while (count) {
    if (blockRemain_ == 0) {
      if (crcPending_) {
        spiRec();          // CRC is not checked (CRC is off in SPI mode)
        spiRec();
        crcPending_ = 0;
      }
      if (!waitStartBlock()) {
        streamAbort();
        return false;
      }
      blockRemain_ = 512;
    }
    uint16_t n = count < blockRemain_ ? (uint16_t)count : blockRemain_;
    if (!spiRecvBulk(dst, n)) {
      error(SD_CARD_ERROR_DMA);
      streamAbort();
      return false;
    }
    if (dst) {
      dst += n;
    }
    count -= n;
    blockRemain_ -= n;
    if (blockRemain_ == 0) {
      crcPending_ = 1;
    }
  }
  return true;
}
//------------------------------------------------------------------------------
/**
   CHGAME: end the stream. The rest of a partly-read block is drained first
   so CMD12 always lands on a block boundary - the SD spec allows CMD12 at
   any point, but "between blocks" is the case every card handles, and it
   costs at most one block of reading. Callers that stop on a boundary pay
   nothing (see the tail cache in CHSpriteView.cpp).

   The card is left in its brief post-CMD12 busy state; the next command
   waits for that in cardCommand(), so the busy time overlaps whatever the
   sketch does next (an LCD flush, usually).
*/
uint8_t Sd2Card::readStop(void) {
  if (!streaming_) {
    return true;
  }
  streaming_ = 0;   // first: cardCommand() below must not recurse into us
  if (blockRemain_) {
    spiRecvBulk(NULL, blockRemain_);
    blockRemain_ = 0;
    crcPending_ = 1;
  }
  if (crcPending_) {
    spiRec();
    spiRec();
    crcPending_ = 0;
  }
  cardCommand(CMD12, 0);
  chipSelectHigh();
  // Any R1 (bit 7 clear) is accepted: some cards flag "out of range" when
  // the stop arrives as they pre-fetch past the last block, which is
  // harmless. Only no response at all is an error.
  if (status_ & 0X80) {
    error(SD_CARD_ERROR_CMD12);
    return false;
  }
  return true;
}
//------------------------------------------------------------------------------
/**
   CHGAME: abandon a stream after an error. The card is still in multi-block
   mode and will ignore anything but CMD12, so send that - but do not try to
   drain the current block first, because after a lost token or a failed DMA
   the byte position is unknown. waitStartBlock() may already have
   deselected the card; cardCommand() re-selects it.
*/
void Sd2Card::streamAbort(void) {
  streaming_ = 0;
  blockRemain_ = 0;
  crcPending_ = 0;
  uint8_t code = errorCode_;   // keep the original failure, not CMD12's
  cardCommand(CMD12, 0);
  chipSelectHigh();
  errorCode_ = code;
}
//------------------------------------------------------------------------------
/**
   CHGAME: stream \a count blocks from \a block through a two-buffer
   pipeline. See the declaration in Sd2Card.h for the contract.

   Timeline for block k (DMA path):

     CPU:  wait token k | start DMA k -> buf[k&1] | fn(block k-1) | wait DMA k | CRC k
     SPI:  0xFF..0xFE   | <------------- 512 bytes of block k -------------->  | CRC

   fn() for block k-1 runs while block k is on the wire, so for a consumer
   faster than the bus, the whole read costs only its bus time.
*/
#ifdef SD_PROFILE
SdProfile sdProf;
/* Cycle timestamp: the core's millisecond count times 48000 plus SysTick,
   which counts HCLK (48 MHz) up to 47999 and resets every millisecond. */
extern "C" volatile uint64_t msTick;
uint32_t sdCycles(void) {
  uint32_t m, c;
  do {
    m = (uint32_t)msTick;
    c = *(volatile uint32_t*)&SysTick->CNT;   // low word is enough: < 48000
  } while (m != (uint32_t)msTick);
  return m * (F_CPU / 1000) + c;
}
  #define PROF_T(v)      uint32_t v = sdCycles()
  #define PROF_ADD(f, a) sdProf.f += sdCycles() - (a)
#else
  #define PROF_T(v)
  #define PROF_ADD(f, a)
#endif

uint8_t Sd2Card::readBlocksPipelined(uint32_t block, uint16_t count,
                                     uint8_t* buf0, uint8_t* buf1,
                                     BlockFn fn, void* user) {
  if (count == 0) {
    return true;
  }
  PROF_T(tStart);
  if (!readStart(block)) {
    return false;
  }
  PROF_ADD(cmd, tStart);
  uint8_t* bufs[2] = { buf0, buf1 };
  const uint8_t* pending = NULL;    // received, not yet handed to fn()

  for (uint16_t k = 0; k < count; k++) {
    uint8_t* cur = bufs[k & 1];

    PROF_T(tTok);
    if (!waitStartBlock()) {
      streamAbort();
      return false;
    }
    #ifdef SD_PROFILE
    if (k == 0) { PROF_ADD(firstToken, tTok); } else { PROF_ADD(nextTokens, tTok); }
    #endif
    #if SD_CH32_FAST
    if (!s_dmaDead) {
      rxDmaStart(cur, 512);
      PROF_T(tFn);
      if (pending) {
        fn(pending, user);          // overlaps the DMA above
      }
      PROF_ADD(fn, tFn);
      PROF_T(tDma);
      if (!rxDmaWait(512)) {
        error(SD_CARD_ERROR_DMA);
        streamAbort();
        return false;
      }
      PROF_ADD(dmaWait, tDma);
    } else
    #endif
    {
      spiRecvBulk(cur, 512);
      if (pending) {
        fn(pending, user);
      }
    }
    spiRec();                       // CRC
    spiRec();
    pending = cur;
  }

  // Stop first, then consume the last block: the card's post-CMD12 busy
  // time runs in parallel with fn().
  blockRemain_ = 0;
  crcPending_ = 0;
  PROF_T(tStop);
  uint8_t ok = readStop();
  PROF_ADD(stop, tStop);
  PROF_T(tLast);
  fn(pending, user);
  PROF_ADD(lastFn, tLast);
  PROF_ADD(total, tStart);
  #ifdef SD_PROFILE
  sdProf.calls++;
  #endif
  return ok;
}
