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
    virtual void updateBatteryLevel(uint32_t mv) {};
    virtual void updateBatteryAdv(void) {};
    virtual void advertiseData(void);
    virtual bool updateAdvData(void);
    virtual char getCommand(void) {return 0x65;};  // Start measurement
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
    bool updateAdvData(void) override;
};


class Tindeq : public Device
{
  private:
    BLEService progressor = BLEService("7e4e1701-1ea6-40c9-9dcc-13d34ffead57");
    BLECharacteristic datapoint = BLECharacteristic("7e4e1702-1ea6-40c9-9dcc-13d34ffead57");
    BLECharacteristic controlpoint = BLECharacteristic("7e4e1703-1ea6-40c9-9dcc-13d34ffead57");

  public:
    uint8_t lastCommand[20];
    uint8_t scale_data[10] = {
      0x01, // Response code: weight measurement
      0x08,                   // length
      0x00, 0x00, 0x00, 0x00, // weight float32
      0x00, 0x00, 0x00, 0x00  // timestamp uint32_t
    };
    uint8_t batt_data[6] = {
      0x00,  // Response code: battery level
      0x04,  // length
      0x00, 0x00, 0x00, 0x00  // battery millivolts
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
    bool updateAdvData(void) override;
    void updateBatteryLevel(uint32_t mv) override;
    void updateBatteryAdv(void) override;

    // Control point
    char getCommand(void) override;
};

// See https://github.com/Stevie-Ray/hangtime-grip-connect/blob/main/packages/core/src/models/device/forceboard.model.ts
class Forceboard : public Device
{
  private:
    // Forceboard service
    // 9a88d67f-8df2-4afe-9e0d-c2bbbe773dd0 
    const uint8_t primary_service_uuid128[16] = {
      0xd0, 0x3d, 0x77, 0xbe, 0xbb, 0xc2, 0x0d, 0x9e,
      0xfe, 0x4a, 0xf2, 0x8d, 0x7f, 0xd6, 0x88, 0x9a
    };
    //BLEService forceboard = BLEService(primary_service_uuid128);

    // Read
    // 9a88d685-8df2-4afe-9e0d-c2bbbe773dd0
    const uint8_t datapoint_characteristic_uuid128[16] = {
      0xd0, 0x3d, 0x77, 0xbe, 0xbb, 0xc2, 0x0d, 0x9e,
      0xfe, 0x4a, 0xf2, 0x8d, 0x85, 0xd6, 0x88, 0x9a
    };
    //BLECharacteristic datapoint = BLECharacteristic(datapoint_characteristic_uuid128);

    // Weight service
    // 467a8516-6e39-11eb-9439-0242ac130002
    const uint8_t weight_service_uuid128[16] = {
      0x02, 0x00, 0x13, 0xac, 0x42, 0x02, 0x39, 0x94,
      0xeb, 0x11, 0x39, 0x6e, 0x16, 0x85, 0x7a, 0x46
    };
    BLEService forceboard = BLEService(weight_service_uuid128);

    // Read + Write
    // 467a8518-6e39-11eb-9439-0242ac130002
    const uint8_t weight_characteristic_uuid128[16] = {
      0x02, 0x00, 0x13, 0xac, 0x42, 0x02, 0x39, 0x94,
      0xeb, 0x11, 0x39, 0x6e, 0x18, 0x85, 0x7a, 0x46
    };
    BLECharacteristic datapoint = BLECharacteristic(weight_characteristic_uuid128);

    // Unknown service
    // f3641400-00b0-4240-ba50-05ca45bf8abc
    const uint8_t unk_service_uuid128[16] = {
      0xbc, 0x8a, 0xbf, 0x45, 0xca, 0x05, 0x50, 0xba,
      0x40, 0x42, 0xb0, 0x00, 0x00, 0x14, 0x64, 0xf3
    };
    //BLEService forceboard = BLEService(unk_service_uuid128);

    // Read + Indicate
    // f3641401-00b0-4240-ba50-05ca45bf8abc
    const uint8_t unk_characteristic_uuid128[16] = {
      0xbc, 0x8a, 0xbf, 0x45, 0xca, 0x05, 0x50, 0xba,
      0x40, 0x42, 0xb0, 0x00, 0x01, 0x14, 0x64, 0xf3
    };
    //BLECharacteristic datapoint = BLECharacteristic(unk_characteristic_uuid128);

  public:
    uint8_t scale_data[5] = {
      0x00, 0x00,  // Number of samples in the packet
      0x00, 0x00, 0x00,  // Weight measurement in lbs. Bytes dot-product [32768, 256, 1]
    };
    uint8_t txValue = 0;

    const char DEVICE_NAME[12] = "Force Board";

    // Update the scale_data with the current weight reading.
    void updateWeight(uint32_t weight) override;

    // Update the scale_data timestamp.
    void updateTimestamp(uint32_t time) override;

    // Setup and start BLE advertising.
    void begin(void) override;
    void advertiseData(void) override;
    bool updateAdvData(void) override;
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
    bool updateAdvData(void) override;

    void setupCalibrate(void);
    void calibrate(HX711 scale);
};

#endif  /* SCALES_h */
