#include <Arduino.h>
#include "scales.h"

/*******************************
 ************* WH06 ************
*******************************/

extern WH06 device;

void WH06::updateWeight(uint32_t weight)
{
  uint8_t msb = ((weight * 10) & 0xFF00U) >> 8U;
  uint8_t lsb = ((weight * 10) & 0x00FFU);
  scale_data[SCALE_DATA_WEIGHT_INT] = msb;
  scale_data[SCALE_DATA_WEIGHT_FRAC] = lsb;
}

void WH06::updateTimestamp(uint16_t time)
{
  // Update the scale_data with the current timestamp.
  uint8_t msb = (time & 0xFF00U) >> 8U;
  uint8_t lsb = (time & 0x00FFU);
  scale_data[SCALE_DATA_TIMESTAMP_MSB] = msb;
  scale_data[SCALE_DATA_TIMESTAMP_LSB] = lsb;
}

void WH06::advertiseData(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);
  // This is necessary, as the Frez app looks for this name to connect to.
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData(&scale_data, SCALE_DATA_LEN);

  /* Start Advertising */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);      // always advertise at 32.
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising.
}

void WH06::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData(&scale_data, SCALE_DATA_LEN);
}

/*********************************
uint8_t* WH06::getScaleData(void)
{
  return &scale_data;
}
*********************************/

void WH06::begin(void)
{
  Bluefruit.begin();
  advertiseData();
}


/*******************************
 ************ TINDEQ ***********
*******************************/
void Tindeq::advertiseData(void)
{
  progressor.begin();
  datapoint.setProperties(CHR_PROPS_NOTIFY);
  datapoint.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  datapoint.begin();
  datapoint.notify(&scale_data, 10);  // timestamp

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(progressor);

  /* Start Advertising */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);      // always advertise at 32.
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising.
}

void Tindeq::updateWeight(uint32_t weight)
{
  float f_weight = static_cast<float>(weight);
  memcpy(&scale_data[2], &f_weight, sizeof(float));
}

void Tindeq::updateTimestamp(uint16_t time)
{
  uint32_t upcast_time = static_cast<uint32_t>(time);
  memcpy(&scale_data[6], &upcast_time, sizeof(uint32_t));
}

void Tindeq::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addName();
  datapoint.notify(&scale_data, 10);
}

void Tindeq::begin(void)
{
  Bluefruit.begin();
  advertiseData();
}

