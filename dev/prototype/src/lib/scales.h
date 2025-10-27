#ifndef SCALES_h
#define SCALES_h

#pragma once

#include <Arduino.h>
#include <bluefruit.h>
#include "HX711.h"
#include "flash.h"


class Device
{
  public:
    virtual void begin(void);
    virtual void updateWeight(uint32_t weight);
    virtual void updateTimestamp(uint32_t time);
    virtual void advertiseData(void);
    virtual void updateAdvData(void);
    virtual ~Device() {};
};


class WH06 : public Device
{
  private:
    // Indices in the advertised data for things we plan to change.
    static const int SCALE_DATA_WEIGHT_INT = 12;      // The integer part
    static const int SCALE_DATA_WEIGHT_FRAC = 13;     // The fractional part
    static const int SCALE_DATA_TIMESTAMP_MSB = 17;   // Most significant bit
    static const int SCALE_DATA_TIMESTAMP_LSB =  18;  // Least significant bit
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

  public:
    // Update the scale_data with the current weight reading.
    void updateWeight(uint32_t weight) override;
    // Update the scale_data timestamp.
    void updateTimestamp(uint32_t time) override;
    // Setup and start BLE advertising.
    void begin(void) override;
    void advertiseData(void) override;
    void updateAdvData(void) override;
};


class Tindeq : public Device
{
  private:
    const uint8_t progressor_service_uuid128[16] = {
      0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d,
      0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17, 0x4e, 0x7e
    };
    BLEService progressor = BLEService(progressor_service_uuid128);

    const uint8_t datapoint_characteristic_uuid128[16] = {
      0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d,
      0xc9, 0x40, 0xa6, 0x1e, 0x02, 0x17, 0x4e, 0x7e
    };
    BLECharacteristic datapoint = BLECharacteristic(datapoint_characteristic_uuid128);

  public:
    uint8_t scale_data[10] = {
      0x01, // Response code: weight measurement
      0x08,                   // length
      0x00, 0x00, 0x00, 0x00, // weight float32
      0x00, 0x00, 0x00, 0x00  // timestamp uint32_t
    };

    uint8_t txValue = 0;

    const char DEVICE_NAME[11] = "Progressor";

    // Update the scale_data with the current weight reading.
    void updateWeight(uint32_t weight) override;

    // Update the scale_data timestamp.
    void updateTimestamp(uint32_t time) override;

    // Setup and start BLE advertising.
    void begin(void) override;
    void advertiseData(void) override;
    void updateAdvData(void) override;
};

class Mito : public Device
{
  private:
    // Primary service
    // a9996d01-16e7-49e1-a66f-f9c4ecff3681
    const uint8_t primary_service_uuid128[16] = {
      0xa9, 0x99, 0x6d, 0x01, 0x16, 0xe7, 0x49, 0xe1,
      0xa6, 0x6f, 0xf9, 0xc4, 0xec, 0xff, 0x36, 0x81
    };
    BLEService mito = BLEService(primary_service_uuid128);

    // Data point
    // a9996d02-16e7-49e1-a66f-f9c4ecff3681
    const uint8_t datapoint_characteristic_uuid128[16] = {
      0xa9, 0x99, 0x6d, 0x02, 0x16, 0xe7, 0x49, 0xe1,
      0xa6, 0x6f, 0xf9, 0xc4, 0xec, 0xff, 0x36, 0x81
    };
    BLECharacteristic datapoint = BLECharacteristic(datapoint_characteristic_uuid128);

    // Control point
    // a9996d03-16e7-49e1-a66f-f9c4ecff3681
    const uint8_t control_characteristic_uuid128[16] = {
      0xa9, 0x99, 0x6d, 0x03, 0x16, 0xe7, 0x49, 0xe1,
      0xa6, 0x6f, 0xf9, 0xc4, 0xec, 0xff, 0x36, 0x81
    };
    BLECharacteristic control_point = BLECharacteristic(control_characteristic_uuid128);

    // For serial communication.
    BLEUart bleuart;

  public:
    static Mito* instance;  // So the static callback can access bleuart.
    uint8_t scale_data[10] = {
      0x01, // Response code: weight measurement
      0x08,                   // length
      0x00, 0x00, 0x00, 0x00, // weight float32
      0x00, 0x00, 0x00, 0x00  // timestamp uint32_t
    };

    const char DEVICE_NAME[11] = "Mito";

    // Update the scale_data with the current weight reading.
    void updateWeight(uint32_t weight) override;

    // Update the scale_data timestamp.
    void updateTimestamp(uint32_t time) override;

    // Setup and start BLE advertising.
    void begin(void) override;
    void advertiseData(void) override;
    void updateAdvData(void) override;

    void setupCalibrate(void);
    void calibrate(HX711 scale);
};

#endif  /* SCALES_h */
