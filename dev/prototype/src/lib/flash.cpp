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

/* Size of the scale param. 4 because we're converting
 * between int32_t and int8_t[] */
#define BUFSIZE          4  
#define PARAM_ADDR       0  // Address of the scale param
#define PARAM_SIGN_ADDR  4  // PARAM_ADDR + BUFSIZE
#define DEVICE_CODE_ADDR 5  // PARAM_SIGN_ADDR + 1

// 4 byte aligned buffer has best result with nRF QSPI
uint8_t param_bufwrite[BUFSIZE] __attribute__ ((aligned(4)));
uint8_t param_bufread[BUFSIZE] __attribute__ ((aligned(4)));
uint8_t param_bufpos;
uint8_t device_code_buf;

void initFlash(void)
{
  flash.begin(&P25Q16H, 1);
}

void saveScaleParam(uint32_t param, uint8_t positive)
{
  memcpy(param_bufwrite, &param, sizeof(param));

  flash.waitUntilReady();
  flash.writeBuffer(PARAM_ADDR, param_bufwrite, sizeof(param_bufwrite));
  flash.writeBuffer(sizeof(param_bufwrite), &positive, sizeof(positive));
}

float readScaleParam(void)
{
  memset(param_bufread, 0, sizeof(param_bufread));
  memset(&param_bufpos, 0, sizeof(param_bufpos));

  flash.waitUntilReady();
  flash.readBuffer(PARAM_ADDR, param_bufread, sizeof(param_bufread));
  flash.readBuffer(PARAM_SIGN_ADDR, &param_bufpos, sizeof(param_bufpos));

  uint32_t base_value;
  float value;
  memcpy(&base_value, &param_bufread, sizeof(param_bufread));
  value = float(base_value);
  if (param_bufpos == 0) {
    value *= -1;
  }
  return value;
}

void saveDefaultDevice(uint8_t device_code)
{
  flash.waitUntilReady();
  flash.writeBuffer(DEVICE_CODE_ADDR, &device_code, sizeof(device_code));
}

uint8_t readDefaultDevice(void)
{
  memset(&device_code_buf, 0, sizeof(device_code_buf));
  flash.waitUntilReady();
  flash.readBuffer(DEVICE_CODE_ADDR, &device_code_buf, sizeof(device_code_buf));
  // TODO: Return WH06 (code 1) if no device currently saved in memory.
  return device_code_buf;
}
