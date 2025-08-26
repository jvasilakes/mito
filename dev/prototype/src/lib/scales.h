#ifndef __SCALE_DATA_H
#define __SCALE_DATA_H

#include <Arduino.h>

// TODO: Make this a class
namespace WH06
{
  // Indices in the advertised data for things we plan to change.
  const int SCALE_DATA_LEN = 19;   // See scale_data init below.
  const int SCALE_DATA_WEIGHT_INT = 12;   // The integer part
  const int SCALE_DATA_WEIGHT_FRAC = 13;   // The fractional part
  const int SCALE_DATA_TIMESTAMP_MSB = 17;
  const int SCALE_DATA_TIMESTAMP_LSB =  18;
  const char DEVICE_NAME[6] = "IF_B7";
  
  uint8_t scale_data[SCALE_DATA_LEN] = {
    0x00,0x01,  // 0, 1: TomTom industries lol
    0x02,0x03,0x11,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // 2-10: idk but seems to the be MAC?
    0x01,  // 11: 01 for kg, 00 for lbs
    0x00,0x00,  // 12, 13: weight in g/10 as uint16_t
    0x01,0xF4,  // 14, 15: idk it never changes.
    0x01,       // 16: 01 for kg, 00 for lbs, A1 for kg (hold), A0 for lbs (hold)
    0x99,0x90   // 17, 18: timestamp as uint16_t
  };
  
  void updateWeight(uint32_t weight)
  {
    uint8_t msb = ((weight * 10) & 0xFF00U) >> 8U;
    uint8_t lsb = ((weight * 10) & 0x00FFU);
    scale_data[SCALE_DATA_WEIGHT_INT] = msb;
    scale_data[SCALE_DATA_WEIGHT_FRAC] = lsb;
  }
  
  void updateTimestamp(uint16_t time)
  {
    // Update the scale_data with the current timestamp.
    uint8_t msb = (time & 0xFF00U) >> 8U;
    uint8_t lsb = (time & 0x00FFU);
    scale_data[SCALE_DATA_TIMESTAMP_MSB] = msb;
    scale_data[SCALE_DATA_TIMESTAMP_LSB] = lsb;
  }
} // WH06

#endif
