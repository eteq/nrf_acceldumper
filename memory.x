/* Linker script for the nRF52 assuming S140 v6.1.1 which is the adafruit standard bootloader for the nrf52840 itsybitsy */
/* Values based on https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/hathach-memory-map */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x26000, LENGTH = 796K
  /* This is only needed to be reserved if the SoftDevice is in use */
  RAM : ORIGIN = 0x20001628, LENGTH = 255448
  /* RAM : ORIGIN = 0x20000000, LENGTH = 255K */
  PANDUMP: ORIGIN = 0x2003FC00, LENGTH = 1K
}

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);
