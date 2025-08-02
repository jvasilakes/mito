// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries
// Adaptation: Matthieu Charbonnier


#include <SPI.h>
#include "Adafruit_SPIFlash.h"
#include "flash_devices.h"

// Built from the P25Q16H datasheet.
SPIFlash_Device_t const P25Q16H {
  .total_size = (1UL << 21), // 2MiB
  .start_up_time_us = 10000, // Don't know where to find that value
  
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,

  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02, // Datasheet p. 27
  .has_sector_protection = 1,   // Datasheet p. 27
  .supports_fast_read = 1,      // Datasheet p. 29
  .supports_qspi = 1,           // Obviously
  .supports_qspi_writes = 1,    // Datasheet p. 41
  .write_status_register_split = 1, // Datasheet p. 28
  .single_status_byte = 0,      // 2 bytes
  .is_fram = 0,                 // Flash Memory
};

// Use this constructor to tune the QSPI pins.
// Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
Adafruit_FlashTransport_QSPI flashTransport;

Adafruit_SPIFlash flash(&flashTransport);

#define BUFSIZE   4  // Because we're converting between int32_t and int8_t[]

// 4 byte aligned buffer has best result with nRF QSPI
uint8_t bufwrite[BUFSIZE] __attribute__ ((aligned(4)));
uint8_t bufread[BUFSIZE] __attribute__ ((aligned(4)));
uint8_t bufpos;

void initFlash(void) {
  flash.begin(&P25Q16H, 1);
}

void saveScaleParam(uint32_t param, uint8_t positive) {
  flash.eraseChip();
  flash.waitUntilReady();
  memcpy(bufwrite, &param, sizeof(param));

  flash.writeBuffer(0, bufwrite, sizeof(bufwrite));
  flash.writeBuffer(sizeof(bufwrite), &positive, sizeof(positive));
}

float readScaleParam() {
  flash.waitUntilReady();
  memset(bufread, 0, sizeof(bufread));
  memset(&bufpos, 0, sizeof(bufpos));
  flash.readBuffer(0, bufread, sizeof(bufread));
  flash.readBuffer(sizeof(bufread), &bufpos, sizeof(bufpos));

  uint32_t base_value;
  float value;
  memcpy(&base_value, &bufread, sizeof(bufread));
  value = float(base_value);
  if (bufpos == 0) {
    value *= -1;
  }
  return value;
}
