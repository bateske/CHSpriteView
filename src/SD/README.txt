
** SD - a slightly more friendly wrapper for sdfatlib **

This library aims to expose a subset of SD card functionality in the
form of a higher level "wrapper" object.

License: GNU General Public License V3
         (Because sdfatlib is licensed with this.)

(C) Copyright 2010 SparkFun Electronics

Now better than ever with optimization, multiple file support, directory handling, etc - ladyada!


---------------------------------------------------------------------------
CHGAME VERSION
---------------------------------------------------------------------------
This copy is Arduino SD 1.3.0, vendored into CHSpriteView and optimised for
the CH32X035 / CHGame board. Every change is marked "CHGAME:" in the source.

  utility/Sd2Card.h/.cpp  register-level SPI1 + DMA transport, bus hand-off
                          with CHGfx, CMD18 streaming (readStart/readStream/
                          readStop/readBlocksPipelined), PB11 default CS
  utility/SdInfo.h        CMD12, CMD18
  utility/SdFile.cpp      File reads of 2+ whole blocks use one CMD18
  utility/Sd2PinMap.h     CH32 branch (stock SD has none and will not build)
  SD.h / SD.cpp           begin() mounts at 24 MHz with fall-back;
                          SD.rawCard(); File::contiguousRange()
  File.cpp                File::contiguousRange()

Define SD_CH32_DISABLE_FAST to fall back to the stock SPI-library transport.
