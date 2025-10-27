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
  // Because the int weight is encoded as grams.
  f_weight = f_weight / 10;
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

/*******************************
 ************ MITO ***********
*******************************/
Mito* Mito::instance = nullptr;

void Mito::advertiseData(void)
{
  mito.begin();
  datapoint.setProperties(CHR_PROPS_NOTIFY);
  datapoint.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  datapoint.begin();
  datapoint.notify(&scale_data, 10);  // timestamp

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(mito);

  /* Start Advertising */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);      // always advertise at 32.
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising.
}

void Mito::updateWeight(uint32_t weight)
{
  float f_weight = static_cast<float>(weight);
  // Because the int weight is encoded as grams.
  f_weight = f_weight / 10;
  memcpy(&scale_data[2], &f_weight, sizeof(float));
}

void Mito::updateTimestamp(uint16_t time)
{
  uint32_t upcast_time = static_cast<uint32_t>(time);
  memcpy(&scale_data[6], &upcast_time, sizeof(uint32_t));
}

void Mito::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addName();
  datapoint.notify(&scale_data, 10);
}

void Mito::begin(void)
{
  instance = this;  // so the static callback can access it.
  Bluefruit.begin();
  advertiseData();
}

void Mito::setupCalibrate()
{
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  
  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);      // always advertise at 32.
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising.
}

void Mito::calibrate(HX711 scale)
{
  setupCalibrate();
  while (!bleuart.available()) delay(10);
  bleuart.println("Mito Scale Calibration");
  bleuart.println("=========================");

  scale.set_scale();
  scale.tare();
  bleuart.println("Tared.");
  bleuart.println("Enter weight.");
  bleuart.println("grams: ");
  bleuart.flush();

  char inByte;
  uint8_t bufsize = 128;
  char inputBuffer[bufsize];
  int bufPtr = 0;
  while (inByte != '\n' && inByte != '\r') {
    if (bleuart.available() > 0) {
      inByte = bleuart.read();
      if (inByte == ' ') {  // Skip empty space.
        continue;
      } else {
        if (bufPtr < (bufsize - 1)) {
          inputBuffer[bufPtr++] = inByte;
        }
      }
      bleuart.print(inByte);
    }
  }
  inputBuffer[bufPtr++] = '\0';
  bufPtr = 0;

  float grams = strtod(inputBuffer, NULL);
  float hectograms = grams / 100;
  
  bleuart.println("\nCalibrating...");

  float sum = 0.0f;
  float total_samples = 0.0f;
  for (int i=0; i<10; i++) {
    //for (int j=0; j<5; j++) {
    for (int j=0; j<1; j++) {
      float reading = scale.get_units(10);
      float scale_param = reading / hectograms;
      sum += scale_param;
      total_samples += 1.0f;
      bleuart.println(reading);
      bleuart.println(scale_param);
      delay(250);
    }
  }

  float mean_param = sum / total_samples;
  bleuart.print("Estimated parameter: ");
  bleuart.println(mean_param);
  int positive = 1;  // 1 for positive param, 0 for negative.
  if (mean_param < 0) {
    positive = 0;
    mean_param *= -1;
  }
  bleuart.println("Saving...");
  initFlash();
  saveScaleParam(uint32_t(mean_param), positive);
  bleuart.print("Validating...");
  float read_param = readScaleParam();
  bleuart.println(read_param);
}
