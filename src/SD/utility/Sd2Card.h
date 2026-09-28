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
#ifndef Sd2Card_h
#define Sd2Card_h
/**
   \file
   Sd2Card class
*/
#include "Sd2PinMap.h"
#include "SdInfo.h"
/*
 * =============================================================================
 * CHGAME CHANGES TO THIS FILE (CH32X035 / CHGame board)
 * =============================================================================
 * This is Arduino SD 1.3.0 with a CH32X035-specific transport bolted
 * underneath it. Everything above the transport - FAT, directories, File -
 * is unchanged apart from the few spots marked "CHGAME:".
 *
 * Why the stock library is slow on this chip:
 *
 *   1. Every byte went through SPIClass::transfer(): a virtual call, a
 *      pin-settings lookup, and spi_transfer(), which polls TXE/RXNE through
 *      function calls and checks a 64-bit millisecond timeout on every
 *      byte, all executing from flash with 3 wait states. That costs a few
 *      microseconds per byte no matter how fast SCK is; at 24 MHz the wire
 *      itself needs 0.33 us per byte.
 *   2. SD.begin(cs) left the card at SPI_HALF_SPEED, which the core's SPI
 *      library rounds to 3 MHz.
 *   3. Every 512-byte block was its own CMD17 with its own access latency.
 *   4. SS on the CHGame variant is PA4 - the LCD's chip select - so a bare
 *      SD.begin() talked to the display instead of the card.
 *
 * What replaces it (SD_CH32_FAST):
 *
 *   - Register-level SPI1 byte exchange, so the per-byte cost is a store,
 *     a status poll and a load.
 *   - DMA for bulk transfers: DMA1 channel 2 receives (SPI_RX) while
 *     channel 3 clocks out 0xFF filler (SPI_TX), per CH32X035 RM 9.2.3.
 *     The bus runs back-to-back at the full 24 MHz, ~3 MB/s.
 *   - A bus hand-off (claim/release) that saves the LCD's SPI1 setup,
 *     applies the card's, and puts the LCD's back afterwards, so this
 *     library and CHGfx share SPI1 without either knowing the other's
 *     settings. The claim also waits out an async CHGfx flush in flight.
 *   - CMD18 multi-block streaming (readStart / readStream / readStop), and
 *     a pipelined variant that hands each block to a callback while the
 *     NEXT block is already arriving by DMA.
 *   - SD_CHIP_SELECT_PIN defaults to PIN_SD_CS (PB11) when the variant
 *     defines it, so SD.begin() with no argument works.
 *
 * Define SD_CH32_DISABLE_FAST before including SD.h to get the stock
 * SPI-library transport back, e.g. to A/B the two.
 */
#if defined(CH32X035) && !defined(SD_CH32_DISABLE_FAST)
  #define SD_CH32_FAST 1
#else
  #define SD_CH32_FAST 0
#endif
/** Set SCK to max rate of F_CPU/2. See Sd2Card::setSckRate().
    CHGAME: 24 MHz on the CH32X035 (BR = 000). */
uint8_t const SPI_FULL_SPEED = 0;
/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate().
    CHGAME: 12 MHz. (The stock SPI-library path turned this into 3 MHz.) */
uint8_t const SPI_HALF_SPEED = 1;
/** Set SCK rate to F_CPU/8. Sd2Card::setSckRate(). CHGAME: 6 MHz. */
uint8_t const SPI_QUARTER_SPEED = 2;
/**
   USE_SPI_LIB: if set, use the SPI library bundled with Arduino IDE, otherwise
   run with a standalone driver for AVR.

   CHGAME: not set on the fast CH32 path, which drives SPI1 itself. That also
   keeps the core's SPI library (~2.8 KB of flash) out of the build.
*/
#if !SD_CH32_FAST
  #define USE_SPI_LIB
#endif
/**
   Define MEGA_SOFT_SPI non-zero to use software SPI on Mega Arduinos.
   Pins used are SS 10, MOSI 11, MISO 12, and SCK 13.

   MEGA_SOFT_SPI allows an unmodified Adafruit GPS Shield to be used
   on Mega Arduinos.  Software SPI works well with GPS Shield V1.1
   but many SD cards will fail with GPS Shield V1.0.
*/
#define MEGA_SOFT_SPI 0
//------------------------------------------------------------------------------
#if MEGA_SOFT_SPI && (defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  #define SOFTWARE_SPI
#endif  // MEGA_SOFT_SPI
//------------------------------------------------------------------------------
// SPI pin definitions
//
#ifndef SOFTWARE_SPI
  // hardware pin defs

  // include pins_arduino.h or variant.h depending on architecture, via Arduino.h
  #include <Arduino.h>

  /**
  SD Chip Select pin

  Warning if this pin is redefined the hardware SS will pin will be enabled
  as an output by init().  An avr processor will not function as an SPI
  master unless SS is set to output mode.
  */
  /* CHGAME: on the CHGame variant SS is PA4, the LCD's chip select, so the
     stock default made SD.begin() clock card commands into the ST7735 while
     the card itself was never selected. The variant names the real card
     select as PIN_SD_CS (PB11); prefer it whenever it exists. */
  #if !defined(SDCARD_SS_PIN) && defined(PIN_SD_CS)
    #define SDCARD_SS_PIN PIN_SD_CS
  #endif

  #ifndef SDCARD_SS_PIN
    /** The default chip select pin for the SD card is SS. */
    uint8_t const  SD_CHIP_SELECT_PIN = SS;
  #else
    uint8_t const  SD_CHIP_SELECT_PIN = SDCARD_SS_PIN;
  #endif

  // The following three pins must not be redefined for hardware SPI,
  // so ensure that they are taken from pins_arduino.h or variant.h, depending on architecture.
  #ifndef SDCARD_MOSI_PIN
    /** SPI Master Out Slave In pin */
    uint8_t const  SPI_MOSI_PIN = MOSI;
    /** SPI Master In Slave Out pin */
    uint8_t const  SPI_MISO_PIN = MISO;
    /** SPI Clock pin */
    uint8_t const  SPI_SCK_PIN = SCK;
  #else
    uint8_t const  SPI_MOSI_PIN = SDCARD_MOSI_PIN;
    uint8_t const  SPI_MISO_PIN = SDCARD_MISO_PIN;
    uint8_t const  SPI_SCK_PIN = SDCARD_SCK_PIN;
  #endif

  /** optimize loops for hardware SPI */
  #ifndef USE_SPI_LIB
    #define OPTIMIZE_HARDWARE_SPI
  #endif

#else  // SOFTWARE_SPI
  // define software SPI pins so Mega can use unmodified GPS Shield
  /** SPI chip select pin */
  uint8_t const SD_CHIP_SELECT_PIN = 10;
  /** SPI Master Out Slave In pin */
  uint8_t const SPI_MOSI_PIN = 11;
  /** SPI Master In Slave Out pin */
  uint8_t const SPI_MISO_PIN = 12;
  /** SPI Clock pin */
  uint8_t const SPI_SCK_PIN = 13;
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/** Protect block zero from write if nonzero */
#define SD_PROTECT_BLOCK_ZERO 1
/** init timeout ms */
unsigned int const SD_INIT_TIMEOUT = 2000;
/** erase timeout ms */
unsigned int const SD_ERASE_TIMEOUT = 10000;
/** read timeout ms */
unsigned int const SD_READ_TIMEOUT = 300;
/** write time out ms */
unsigned int const SD_WRITE_TIMEOUT = 600;
//------------------------------------------------------------------------------
// SD card errors
/** timeout error for command CMD0 */
uint8_t const SD_CARD_ERROR_CMD0 = 0X1;
/** CMD8 was not accepted - not a valid SD card*/
uint8_t const SD_CARD_ERROR_CMD8 = 0X2;
/** card returned an error response for CMD17 (read block) */
uint8_t const SD_CARD_ERROR_CMD17 = 0X3;
/** card returned an error response for CMD24 (write block) */
uint8_t const SD_CARD_ERROR_CMD24 = 0X4;
/**  WRITE_MULTIPLE_BLOCKS command failed */
uint8_t const SD_CARD_ERROR_CMD25 = 0X05;
/** card returned an error response for CMD58 (read OCR) */
uint8_t const SD_CARD_ERROR_CMD58 = 0X06;
/** SET_WR_BLK_ERASE_COUNT failed */
uint8_t const SD_CARD_ERROR_ACMD23 = 0X07;
/** card's ACMD41 initialization process timeout */
uint8_t const SD_CARD_ERROR_ACMD41 = 0X08;
/** card returned a bad CSR version field */
uint8_t const SD_CARD_ERROR_BAD_CSD = 0X09;
/** erase block group command failed */
uint8_t const SD_CARD_ERROR_ERASE = 0X0A;
/** card not capable of single block erase */
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0X0B;
/** Erase sequence timed out */
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0X0C;
/** card returned an error token instead of read data */
uint8_t const SD_CARD_ERROR_READ = 0X0D;
/** read CID or CSD failed */
uint8_t const SD_CARD_ERROR_READ_REG = 0X0E;
/** timeout while waiting for start of read data */
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X0F;
/** card did not accept STOP_TRAN_TOKEN */
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X10;
/** card returned an error token as a response to a write operation */
uint8_t const SD_CARD_ERROR_WRITE = 0X11;
/** attempt to write protected block zero */
uint8_t const SD_CARD_ERROR_WRITE_BLOCK_ZERO = 0X12;
/** card did not go ready for a multiple block write */
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X13;
/** card returned an error to a CMD13 status check after a write */
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X14;
/** timeout occurred during write programming */
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X15;
/** incorrect rate selected */
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X16;
/** CHGAME: card rejected READ_MULTIPLE_BLOCK */
uint8_t const SD_CARD_ERROR_CMD18 = 0X17;
/** CHGAME: no valid response to STOP_TRANSMISSION */
uint8_t const SD_CARD_ERROR_CMD12 = 0X18;
/** CHGAME: an SPI DMA transfer never completed. The transport falls back to
    polled transfers for the rest of the session (see dmaEnabled()). */
uint8_t const SD_CARD_ERROR_DMA = 0X19;
//------------------------------------------------------------------------------
// card types
/** Standard capacity V1 SD card */
uint8_t const SD_CARD_TYPE_SD1 = 1;
/** Standard capacity V2 SD card */
uint8_t const SD_CARD_TYPE_SD2 = 2;
/** High Capacity SD card */
uint8_t const SD_CARD_TYPE_SDHC = 3;
//------------------------------------------------------------------------------
/**
   \class Sd2Card
   \brief Raw access to SD and SDHC flash memory cards.
*/
class Sd2Card {
  public:
    /** Construct an instance of Sd2Card. */
    Sd2Card(void) : errorCode_(0), inBlock_(0), partialBlockRead_(0), type_(0),
      streaming_(0), crcPending_(0), blockRemain_(0), sckRateId_(SPI_HALF_SPEED) {}
    uint32_t cardSize(void);
    uint8_t erase(uint32_t firstBlock, uint32_t lastBlock);
    uint8_t eraseSingleBlockEnable(void);
    /**
       \return error code for last error. See Sd2Card.h for a list of error codes.
    */
    uint8_t errorCode(void) const {
      return errorCode_;
    }
    /** \return error data for last error. */
    uint8_t errorData(void) const {
      return status_;
    }
    /**
       Initialize an SD flash memory card with default clock rate and chip
       select pin.  See sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin).
    */
    uint8_t init(void) {
      return init(SPI_FULL_SPEED, SD_CHIP_SELECT_PIN);
    }
    /**
       Initialize an SD flash memory card with the selected SPI clock rate
       and the default SD chip select pin.
       See sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin).
    */
    uint8_t init(uint8_t sckRateID) {
      return init(sckRateID, SD_CHIP_SELECT_PIN);
    }
    uint8_t init(uint8_t sckRateID, uint8_t chipSelectPin);
    void partialBlockRead(uint8_t value);
    /** Returns the current value, true or false, for partial block read. */
    uint8_t partialBlockRead(void) const {
      return partialBlockRead_;
    }
    uint8_t readBlock(uint32_t block, uint8_t* dst);
    uint8_t readData(uint32_t block,
                     uint16_t offset, uint16_t count, uint8_t* dst);
    /**
       Read a cards CID register. The CID contains card identification
       information such as Manufacturer ID, Product name, Product serial
       number and Manufacturing date. */
    uint8_t readCID(cid_t* cid) {
      return readRegister(CMD10, cid);
    }
    /**
       Read a cards CSD register. The CSD contains Card-Specific Data that
       provides information regarding access to the card's contents. */
    uint8_t readCSD(csd_t* csd) {
      return readRegister(CMD9, csd);
    }
    void readEnd(void);
    uint8_t setSckRate(uint8_t sckRateID);
    /** CHGAME: now available on every transport, not only USE_SPI_LIB.
        On the CH32 path it picks the fastest divider that does not exceed
        \a clock (24, 12, 6, 3 MHz ...). */
    uint8_t setSpiClock(uint32_t clock);
    /** CHGAME: the rate ID currently in use (0 = F_CPU/2 = 24 MHz). */
    uint8_t sckRateId(void) const {
      return sckRateId_;
    }

    //--------------------------------------------------------------------------
    // CHGAME: multi-block streaming reads (CMD18 / CMD12)
    //
    // A file whose clusters are contiguous (see SdFile::contiguousRange())
    // is just a run of raw blocks, and these read such a run with ONE
    // command. The card is only asked for the first block; the rest arrive
    // back to back with no per-block command or access latency.
    //
    //   readStart(lba)       send CMD18, leave the card selected and streaming
    //   readStream(dst, n)   next n bytes of the stream; dst == NULL skips them.
    //                        Crosses block boundaries (token + CRC) for you.
    //   readStop()           finish the current block, CMD12, deselect.
    //
    // While a stream is open the card holds the SPI bus: nothing else may
    // talk on SPI1 (no LCD flush) until readStop(). Any other card command
    // issued mid-stream stops the stream first, so misuse costs speed, not
    // data.
    //--------------------------------------------------------------------------
    uint8_t readStart(uint32_t block);
    uint8_t readStream(uint8_t* dst, uint32_t count);
    uint8_t readStop(void);

    /** Called by readBlocksPipelined() once per 512-byte block, in order. */
    typedef void (*BlockFn)(const uint8_t* block, void* user);

    /**
       CHGAME: read \a count consecutive blocks starting at \a block and hand
       each one to \a fn. Two 512-byte buffers ping-pong: while fn() works on
       block k in one buffer, DMA is already filling the other with block k+1,
       so whatever fn() does (copying into a framebuffer, diffing, decoding)
       costs no extra time unless it is slower than the wire (~170 us per
       block at 24 MHz). Without DMA it degrades to read-then-call.

       \return true on success. On failure the stream is stopped and the
       card deselected.
    */
    uint8_t readBlocksPipelined(uint32_t block, uint16_t count,
                                uint8_t* buf0, uint8_t* buf1,
                                BlockFn fn, void* user);

    /* CHGAME, TRIED AND REJECTED - "prefetch": send the next frame's CMD18
       early, deselect, let the LCD use the bus while the card fetches, then
       reselect and read. Would hide the ~580 us first-block latency. On the
       test card, about half the reads came back with errors, and the ones
       that worked still waited ~480 us, i.e. the card does not keep fetching
       while deselected. The SD spec requires CS low for the whole read
       transaction; this is why. Not worth card-dependent behaviour. */


    /** CHGAME: true while bulk transfers are going through DMA. Goes false
        for good if a DMA transfer ever fails to complete, after which the
        polled path carries on. */
    static uint8_t dmaEnabled(void);
    /** Return the card type: SD V1, SD V2 or SDHC */
    uint8_t type(void) const {
      return type_;
    }
    uint8_t writeBlock(uint32_t blockNumber, const uint8_t* src, uint8_t blocking = 1);
    uint8_t writeData(const uint8_t* src);
    uint8_t writeStart(uint32_t blockNumber, uint32_t eraseCount);
    uint8_t writeStop(void);
    uint8_t isBusy(void);
  private:
    uint32_t block_;
    uint8_t chipSelectPin_;
    uint8_t errorCode_;
    uint8_t inBlock_;
    uint16_t offset_;
    uint8_t partialBlockRead_;
    uint8_t status_;
    uint8_t type_;
    // CHGAME: stream state for readStart()/readStream()/readStop()
    uint8_t streaming_;     // CMD18 in progress, card selected
    uint8_t crcPending_;    // a block's data is fully read, its CRC is not
    uint16_t blockRemain_;  // data bytes left in the current block (0 = need token)
    uint8_t sckRateId_;     // rate ID applied after card identification
    // private functions
    uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
      cardCommand(CMD55, 0);
      return cardCommand(cmd, arg);
    }
    uint8_t cardCommand(uint8_t cmd, uint32_t arg);
    void error(uint8_t code) {
      errorCode_ = code;
    }
    uint8_t readRegister(uint8_t cmd, void* buf);
    uint8_t sendWriteCommand(uint32_t blockNumber, uint32_t eraseCount);
    void chipSelectHigh(void);
    void chipSelectLow(void);
    void type(uint8_t value) {
      type_ = value;
    }
    // CHGAME: give up on a CMD18 stream whose position is no longer known
    // (missing token, failed DMA): CMD12 without draining anything.
    void streamAbort(void);
    uint8_t waitNotBusy(unsigned int timeoutMillis);
    uint8_t writeData(uint8_t token, const uint8_t* src);
    uint8_t waitStartBlock(void);
};
//------------------------------------------------------------------------------
// CHGAME: optional phase profiler for readBlocksPipelined(). Build with
// -DSD_PROFILE to enable; it costs nothing otherwise. Times are CPU cycles
// (48 per microsecond), accumulated until the sketch reads and clears them.
#ifdef SD_PROFILE
struct SdProfile {
  uint32_t calls;       // readBlocksPipelined() calls
  uint32_t cmd;         // CMD18 sent until R1 (includes waiting out busy)
  uint32_t firstToken;  // R1 until the first block's start token (access latency)
  uint32_t nextTokens;  // waiting for the start tokens of blocks 2..n
  uint32_t dmaWait;     // spent waiting for block DMA after fn() returned
  uint32_t fn;          // time in the block callback (overlapped with DMA)
  uint32_t stop;        // CRC + CMD12 + deselect at the end
  uint32_t lastFn;      // the final, non-overlapped callback
  uint32_t total;       // whole call
};
extern SdProfile sdProf;
uint32_t sdCycles(void);
#endif
#endif  // Sd2Card_h
