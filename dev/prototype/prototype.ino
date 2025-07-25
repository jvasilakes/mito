#include <string.h>
#include <bluefruit.h>
#include "Adafruit_TinyUSB.h"
#include "HX711.h"
#include "flash.h"

// Indices in the advertised data for things we plan to change.
#define SCALE_DATA_LEN           19   // See scale_data init below.
#define SCALE_DATA_WEIGHT_INT    12   // The integer part
#define SCALE_DATA_WEIGHT_FRAC   13   // The fractional part
#define SCALE_DATA_TIMESTAMP_MSB 17
#define SCALE_DATA_TIMESTAMP_LSB 18

int WH06_MODE = 1;

// Alternative program states.
// set to 1 in setup() if tare button is pressed
int doCalibrate = 0;  // 1 time during startup.
int debug = 1;        // 3 times during startup.

// Tare button
const int tarePin = 10;  // the tare button

// RGB LED
const int redPin = 6;    // the number of the LED pin
const int greenPin = 5;    // the number of the LED pin
const int bluePin = 4;    // the number of the LED pin

// Load Cell
const uint8_t LOADCELL_DOUT_PIN = 9;
const uint8_t LOADCELL_SCK_PIN = 8;
const int LOADCELL_GAIN = 128;

int tareState = 0;  // tare button push state.

// Current LED color
uint8_t color[3] = {0,0,0};

// Data to be advertised by the scale.
// Currently spoofing the data put out by the WH-06.
uint8_t scale_data[SCALE_DATA_LEN] = {
  0x00,0x01,  // 0, 1: TomTom industries lol
  0x02,0x03,0x11,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // 2-10: idk but seems to the be MAC?
  0x01,  // 11: 01 for kg, 00 for lbs
  0x00,0x00,  // 12, 13: weight in g/10 as uint16_t
  0x01,0xF4,  // 14, 15: idk it never changes.
  0x01,       // 16: 01 for kg, 00 for lbs, A1 for kg (hold), A0 for lbs (hold)
  0x99,0x90   // 17, 18: timestamp as uint16_t
};

// Parameter for Exponential Moving Average
uint32_t EMA_ALPHA = 30;
long reading = 0;  // For computing exponential moving average.
long smoothed_reading = 0;  // For computing exponential moving average.
long weight = 0;  // Current weight reading in hectograms.
uint16_t prev_time = 0;  // To measure increments without delay() 
uint16_t curr_time = 0;  // Current time stamp

int num_samples = 0;
int hz = 0;  // For measuring HX711 speed.
HX711 scale;


void setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  color[0] = red;
  color[1] = green;
  color[2] = blue;
}

void setLedColor(void)
{
  // Set the LED color based on RGB values.
  // Assumes common anode RGB LED
  digitalWrite(redPin, 255 - color[0]);
  digitalWrite(greenPin, 255 - color[1]);
  digitalWrite(bluePin, 255 - color[2]);
}

void turnOff(void)
{
  // Turn off the LED.
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
}

void flashLED(void)
{
  // Flash the LED on and off with the specified color.
  // Used to indicate tare.
  for (int i=0; i<3; i++) {
    turnOff();
    delay(100);
    setLedColor();
    delay(100);
  }
}

void advertiseData(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // This is necessary, as the Frez app looks for this name to connect to.
  Bluefruit.setName("IF_B7");
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData(&scale_data, SCALE_DATA_LEN);

  /* Start Advertising
   * We go fast since the frez app takes all data from 
   * the advertisement. */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms, so 32=20ms
  Bluefruit.Advertising.setFastTimeout(0);      // always advertise at 32.
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising.
}

void tare(void)
{
  debugPrintln("Taring...");
  scale.tare();
  weight = 0;
  flashLED();
}

int getWeight(void)
{
  int maxr;
  int minr;
  int midr;
  int rval = 0;
  // Take a reading from the HX711.
  // Remove the upper and lower outlier
  for (int i=0; i<3; i++) {
    if (scale.is_ready()) {
      reading = scale.read_average(1);
      rval = 1;
    }
    if (i == 0) {
      maxr = reading;
      minr = reading;
      midr = reading;
    } else {
      if (reading > maxr) {
        maxr = reading;
      } else if (reading < minr) {
        minr = reading;
      } else {
        midr = reading;
      }
    }
  }
  if (smoothed_reading == 0) {
    smoothed_reading = midr;
  }
  // Exponential moving average filter.
  debugPrint(smoothed_reading);
  debugPrint(",");
  smoothed_reading = ((EMA_ALPHA * midr) + ((100 - EMA_ALPHA) * smoothed_reading))/100;
  debugPrint(scale.OFFSET);
  debugPrint(",");
  debugPrint(scale.SCALE);
  debugPrint(",");
  weight = (smoothed_reading - scale.OFFSET) / scale.SCALE;
  debugPrint(midr);
  debugPrint(",");
  debugPrint(smoothed_reading);
  debugPrint(",");
  debugPrint(weight);
  debugPrint(",");
  debugPrintln(hz);
  //delay(1000);
  return rval;
}

void updateWeight(uint32_t grams)
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

void updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData(&scale_data, SCALE_DATA_LEN);
}

template <typename T>
void debugPrint(T msg)
{
  if (debug == true) {
    Serial.print(msg);
  }
}

template <typename T>
void debugPrintln(T msg)
{
  if (debug == true) {
    Serial.println(msg);
  }
}

void calibrate(void) {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Scale calibration");
  Serial.println("=========================\n");

  scale.set_scale();
  scale.tare();
  Serial.println("Place a known weight on the scale, then enter its weight in grams.");
  Serial.print("grams: ");

  char inByte;
  uint8_t bufsize = 128;
  char inputBuffer[bufsize];
  int bufPtr = 0;
  while (inByte != '\n' && inByte != '\r') {
    if (Serial.available() > 0) {
      inByte = Serial.read();
      if (inByte == ' ') {  // Skip empty space.
        continue;
      } else {
        if (bufPtr < (bufsize - 1)) {
          inputBuffer[bufPtr++] = inByte;
        }
      }
      Serial.print(inByte);
    }
  }
  inputBuffer[bufPtr++] = '\0';
  bufPtr = 0;

  float grams = strtod(inputBuffer, NULL);
  float hectograms = grams / 100;
  
  Serial.println();
  Serial.println("Running calibration...");

  float sum = 0.0f;
  float total_samples = 0.0f;
  for (int i=0; i<10; i++) {
    //for (int j=0; j<5; j++) {
    for (int j=0; j<1; j++) {
      float reading = scale.get_units(10);
      float scale_param = reading / hectograms;
      sum += scale_param;
      total_samples += 1.0f;
      Serial.println(reading);
      Serial.print("Estimated scale parameter: ");
      Serial.println(scale_param);
      delay(250);
    }
    //Serial.println("Take the weight off and press ENTER to tare.");
    //inByte = '\0';
    //while (inByte != '\n' && inByte != '\r') {
    //  if (Serial.available() > 0) {
    //    inByte = Serial.read();
    //  }
    //}
    //scale.set_scale();
    //scale.tare();
    //Serial.println("Put the same weight back on and press ENTER.");
    //inByte = '\0';
    //while (inByte != '\n' && inByte != '\r') {
    //  if (Serial.available() > 0) {
    //    inByte = Serial.read();
    //  }
    //}
  }

  float mean_param = sum / total_samples;
  Serial.print("Average scale parameter: ");
  Serial.println(mean_param);
  int positive = 1;
  if (mean_param < 0) {
    positive = 0;
    mean_param *= -1;
  }
  Serial.println("Saving to internal memory");
  initFlash();
  saveScaleParam(uint32_t(mean_param), positive);
  Serial.print("Validating saved parameter");
  float read_param = readScaleParam();
  Serial.print(" = ");
  Serial.println(read_param);
}

int countTarePresses() {
  curr_time = millis();
  prev_time = millis();
  int count = 0;
  int newState = LOW;
  int oldState = LOW;
  while (curr_time - prev_time < 1000) {
    newState = digitalRead(tarePin);
    if (newState != oldState) {
      if (newState == LOW) {
        // oldState is HIGH, so the button was pressed then released.
        count += 1;
      }
      oldState = newState;
    }
    curr_time = millis();
    delay(50);  // Debounce the button.
  }
  curr_time = millis();
  prev_time = millis();

  return count;
}

void setup() {
  // initialize the tare button.
  pinMode(tarePin, INPUT);
  // initialize the LED.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // If tare held at startup, enter calibration mode.
  tareState = digitalRead(tarePin);
  if (tareState == HIGH) {
    doCalibrate = 1;
    // white
    setColor(255, 255, 255);
    setLedColor();
  } else {
    // At startup show green for 1s
    // allowing user to press tare to enter different modes.
    setColor(0, 255, 0);
    setLedColor();
    int num_presses = countTarePresses();

    switch (num_presses) {
      case 0:
        WH06_MODE = 1;
        break;
      case 2:
        Serial.println("TINDEQ_MODE not supported yet.");
        Serial.println("Falling back to WH06_MODE.");
        // TINDEQ_MODE = 1;
        WH06_MODE = 1;
        break;
    }
    // light blue
    setColor(0, 175, 255);
    setLedColor();
  }


  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println("Starting.");
  // Initialize the scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, LOADCELL_GAIN);
  delay(250);
  scale.power_up();
  delay(250);
  if (scale.wait_ready_timeout(1000)) {
    debugPrintln("HX711 found");
  }
  scale.set_gain(64);

  if (doCalibrate == 1) {
    calibrate();
    // light blue
    setColor(0, 175, 255);
    setLedColor();
  }

  initFlash();
  float scale_param = readScaleParam();
  debugPrint("Scale set to ");
  debugPrintln(scale_param);
  scale.set_scale(scale_param);
  //scale.set_scale();
  tare();

  // Start BLE
  Bluefruit.begin();
  //Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  advertiseData();
  debugPrintln("BLE Advertising");
}

void loop() {
  tareState = digitalRead(tarePin);
  // Tare when button pressed.
  if (tareState == HIGH) {
    tare();
  }

  // Update the advertisement every time we get a new weight.
  // This is 10Hz by default on the HX711, but can be increased
  // to 80Hz via the RATE pin.
  int got_weight = getWeight();
  //int got_weight = getWeightFiltered();
  if (got_weight == 1) {
    curr_time = millis();
    updateWeight(weight);
    updateTimestamp(curr_time);
    updateAdvData();
    uint8_t wmsb = scale_data[SCALE_DATA_WEIGHT_INT];
    uint8_t wlsb = scale_data[SCALE_DATA_WEIGHT_FRAC];
    uint8_t tmsb = scale_data[SCALE_DATA_TIMESTAMP_MSB];
    uint8_t tlsb = scale_data[SCALE_DATA_TIMESTAMP_LSB];
    num_samples += 1;
  }

  // In case of overflow.
  if (curr_time < prev_time) {
    curr_time = prev_time = millis();
  }

  // Measure sampling rate.
  if ((curr_time - prev_time) >= 1000) {
    hz = num_samples;
    curr_time = prev_time = millis();
    num_samples = 0;
  }
}
