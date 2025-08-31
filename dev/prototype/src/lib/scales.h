#ifndef SCALES_h
#define SCALES_h

#include <Arduino.h>
#include <bluefruit.h>


class WH06
{
  private:
    // Indices in the advertised data for things we plan to change.
    static const int SCALE_DATA_WEIGHT_INT = 12;      // The integer part
    static const int SCALE_DATA_WEIGHT_FRAC = 13;     // The fractional part
    static const int SCALE_DATA_TIMESTAMP_MSB = 17;   // Most significant bit
    static const int SCALE_DATA_TIMESTAMP_LSB =  18;  // Least significant bit

  public:
    static const int SCALE_DATA_LEN = 19;   // See scale_data init below.
    const char DEVICE_NAME[6] = "IF_B7";       // All WH06 devices have this name.    
    uint8_t scale_data[SCALE_DATA_LEN] = {
      0x00,0x01,  // 0, 1: TomTom industries lol
      0x02,0x03,0x11,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // 2-10: idk but seems to the be MAC?
      0x01,  // 11: 01 for kg, 00 for lbs
      0x00,0x00,  // 12, 13: weight in g/10 as uint16_t
      0x01,0xF4,  // 14, 15: idk it never changes.
      0x01,       // 16: 01 for kg, 00 for lbs, A1 for kg (hold), A0 for lbs (hold)
      0x99,0x90   // 17, 18: timestamp as uint16_t
    };

  // Update the scale_data with the current weight reading.
  void updateWeight(uint32_t weight);

  // Update the scale_data timestamp.
  void updateTimestamp(uint16_t time);

  // Setup and start BLE advertising.
  void begin(void);
  void advertiseData(void);
  void updateAdvData(void);
};


class Tindeq
{
  private:
    const uint8_t progressor_service_uuid128[16] = {
      0x57, 0xad, 0x5e, 0x4f, 0xd3, 0x13, 0xcc, 0x9d,
      0x40, 0xc9, 0x1e, 0xa6, 0x7e, 0x4e, 0x17, 0x01
    };
    BLEService progressor = BLEService(progressor_service_uuid128);

    const uint8_t datapoint_characteristic_uuid128[16] = {
      0x57, 0xad, 0x5e, 0x4f, 0xd3, 0x13, 0xcc, 0x9d,
      0x40, 0xc9, 0x1e, 0xa6, 0x7e, 0x4e, 0x17, 0x02
    };
    BLECharacteristic datapoint = BLECharacteristic(datapoint_characteristic_uuid128);

    uint8_t scale_data[10] = {
      0x01, // Response code: weight measurement
      0x08,                   // length
      0x00, 0x00, 0x00, 0x10, // weight
      0x00, 0x00, 0x00, 0x00  // timestamp
    };

  public:
    const char DEVICE_NAME[11] = "Progressor";

  // Update the scale_data with the current weight reading.
  void updateWeight(uint32_t weight);

  // Update the scale_data timestamp.
  void updateTimestamp(uint32_t time);

  // Setup and start BLE advertising.
  void begin(void);
  void advertiseData(void);
  void updateAdvData(void);
};

#endif  /* SCALES_h */
