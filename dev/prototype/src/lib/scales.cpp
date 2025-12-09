#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "scales.h"

/*******************************
 ************* WH06 ************
*******************************/

extern WH06 device;

void WH06::updateWeight(uint32_t weight)
{
  // WH06 works with hectograms for some reason.
  uint8_t msb = ((weight / 10) & 0xFF00U) >> 8U;
  uint8_t lsb = ((weight / 10) & 0x00FFU);
  scale_data[SCALE_DATA_WEIGHT_INT] = msb;
  scale_data[SCALE_DATA_WEIGHT_FRAC] = lsb;
}

//void WH06::updateTimestamp(uint16_t time)
void WH06::updateTimestamp(uint32_t time)
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
 ******** ForceBoard ***********
*******************************/
void Forceboard::advertiseData(void)
{
  Bluefruit.Periph.setConnIntervalMS(8, 16);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // Start the service
  forceboard.begin();
  datapoint.setProperties(CHR_PROPS_NOTIFY);
  datapoint.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  datapoint.setMaxLen(sizeof(scale_data));
  datapoint.begin();
  datapoint.notify(&scale_data, sizeof(scale_data));  // timestamp

  //Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);

  const uint8_t data1[2] = { 0x80, 0x0C };
  Bluefruit.Advertising.addData(0x19, &data1, sizeof(data1));

  const uint8_t data[12] = {
    0x7f, 0xd6, 0x50, 0x69, 0x74, 0x63,
    0x68, 0x20, 0x53, 0x69, 0x78, 0x00
  };
  Bluefruit.Advertising.addData(0xFF, &data, sizeof(data));

  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.addService(forceboard);

  /* Start Advertising */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 128);     // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);        // always advertise at 32.
  Bluefruit.Advertising.start(0);                 // 0 = Don't stop advertising.
}

void Forceboard::updateWeight(uint32_t weight)
{
  // Convert from grams to lbs.
  uint32_t lbs = weight * 0.002204623;

  // Encode in their weird scheme
  uint32_t remain = lbs;
  uint32_t divs[3] = { 32768, 256, 1 };
  uint8_t encoding[3] = { 0 };
  for (int i=0; i<3; i++) {
      int t = min(remain / divs[i], 255);
      encoding[i] = t;
      remain -= divs[i] * t;
  }
  uint16_t num_samples = 1;
  memcpy(&scale_data[0], &num_samples, sizeof(num_samples));
  memcpy(&scale_data[2], &encoding, sizeof(encoding));
}

void Forceboard::updateTimestamp(uint32_t time)
{
  return;
}

void Forceboard::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  datapoint.notify(&scale_data, sizeof(scale_data));
}

void Forceboard::begin(void)
{
  Bluefruit.begin();
  advertiseData();
}

/*******************************
 ************ TINDEQ ***********
*******************************/
void Tindeq::advertiseData(void)
{
  //Bluefruit.Periph.setConnIntervalMS(8, 16);
  //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // Start the service
  progressor.begin();
  datapoint.setProperties(CHR_PROPS_NOTIFY);
  datapoint.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  datapoint.setMaxLen(sizeof(scale_data));
  datapoint.begin();

  controlpoint.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  controlpoint.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  controlpoint.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(progressor);

  /* Start Advertising */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 128);     // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);        // always advertise at 32.
  Bluefruit.Advertising.start(0);                 // 0 = Don't stop advertising.
}

void Tindeq::updateWeight(uint32_t weight)
{
  float f_weight = static_cast<float>(weight);
  // Because the int weight is encoded as grams.
  f_weight = f_weight / 1000;
  memcpy(&scale_data[2], &f_weight, sizeof(float));
}

void Tindeq::updateTimestamp(uint32_t time)
{
  memcpy(&scale_data[6], &time, sizeof(uint32_t));
}

void Tindeq::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  datapoint.notify(&scale_data, sizeof(scale_data));
}


void Tindeq::updateBatteryLevel(uint32_t mv)
{
  memcpy(&batt_data[2], &mv, sizeof(uint32_t));
}

void Tindeq::updateBatteryAdv(void)
{
  datapoint.notify(&batt_data, sizeof(batt_data));
}

char Tindeq::getCommand(void)
{
  uint8_t buffer[20];
  int len = controlpoint.read(buffer, sizeof(buffer));

  if (len > 0 && memcmp(buffer, lastCommand, len) != 0) {
    Serial.print("Command: ");
    Serial.println(buffer[0], HEX);
    memcpy(lastCommand, buffer, len);
    return buffer[0];
  }
  return 0;
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
  datapoint.notify(&scale_data, sizeof(scale_data));  // timestamp

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
  memcpy(&scale_data[2], &f_weight, sizeof(float));
}

void Mito::updateTimestamp(uint32_t time)
{
  memcpy(&scale_data[6], &time, sizeof(uint32_t));
}

void Mito::updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  datapoint.notify(&scale_data, sizeof(scale_data));
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
  
  bleuart.println("\nCalibrating...");

  float sum = 0.0f;
  float total_samples = 0.0f;
  for (int i=0; i<10; i++) {
    //float reading = scale.get_units(10);
    float reading = scale.read_average(10) - scale.OFFSET;
    float scale_param = reading / grams;
    sum += scale_param;
    total_samples += 1.0f;
    bleuart.println(reading);
    bleuart.println(scale_param);
    delay(250);
  }

  float mean_param = sum / total_samples;
  bleuart.print("Estimated parameter: ");
  bleuart.println(mean_param);
  bleuart.println("Saving...");
  initFlash();
  saveScaleParam(mean_param);
  bleuart.print("Validating...");
  float read_param = readScaleParam();
  bleuart.println(read_param);
}
