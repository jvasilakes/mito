#include <string.h>
#include "Adafruit_TinyUSB.h"
#include "src/lib/HX711.h"
#include "src/lib/flash.h"
#include "src/lib/scales.h"


/* Set in setup() */
uint8_t DEVICE_CODE;
// int DEVICE_CODE = 0;  // WH06
// int DEVICE_CODE = 1;  // Tindeq
int NUM_DEVICES = 2;

bool doCalibrate = 0;  // Tare button pressed at startup.

// Tare button
const uint8_t tarePin = 7;  // the tare button
bool tareState = 0;  // tare button push state.

// External button
const uint8_t buttonPin = 0;
bool buttonState = 0;

// Debug pin. If grounded enter debug mode.
const uint8_t debugPin = 3;
bool debug = 0;

// RGB LED pins
const uint8_t redPin = 4;    // the number of the LED pin
const uint8_t greenPin = 5;    // the number of the LED pin
const uint8_t bluePin = 6;    // the number of the LED pin
// Current LED color
uint8_t color[3] = {0,0,0};

// Load Cell parameters
const uint8_t LOADCELL_DOUT_PIN = 9;
const uint8_t LOADCELL_SCK_PIN = 10;
const uint8_t LOADCELL_GAIN = 128;

// Parameter for Exponential Moving Average
uint32_t EMA_ALPHA = 30;

// Global Variables
long reading = 0;  // For computing exponential moving average.
uint32_t smoothed_reading = 0;  // For computing exponential moving average.
uint32_t weight = 0;  // Current weight reading in hectograms.
uint16_t prev_time = 0;  // To measure increments without delay() 
uint16_t curr_time = 0;  // Current time stamp
uint16_t num_samples = 0;
uint16_t hz = 0;  // For measuring HX711 speed.
HX711 scale;  // The ADC
Device* device = nullptr;  // The overall device: WH06, Tindeq


void setLEDColor(uint8_t red, uint8_t green, uint8_t blue)
{
  // Set the LED color based on RGB values.
  // Assumes common anode RGB LED
  digitalWrite(redPin, 255 - red);
  digitalWrite(greenPin, 255 - green);
  digitalWrite(bluePin, 255 - blue);
}


uint8_t* getLEDColor(void)
{
  uint8_t* rgb = new uint8_t[3];
  rgb[0] = 255 - digitalRead(redPin); 
  rgb[1] = 255 - digitalRead(greenPin); 
  rgb[2] = 255 - digitalRead(bluePin); 
  return rgb;
}

void turnOffLED(void)
{
  // Turn off the LED.
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
}

void flashLED(void)
{
  // Flash the LED on and off with the current color.
  // Used to indicate tare.
  uint8_t* rgb = getLEDColor();
  for (int i=0; i<3; i++) {
    turnOffLED();
    delay(100);
    setLEDColor(rgb[0], rgb[1], rgb[2]);
    delay(100);
  }
  delete[] rgb;
}

void tare(void)
{
  debugPrintln("Taring...");
  scale.tare();
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
  int numerator; 
  if (smoothed_reading < scale.OFFSET) {
    numerator = 0;
  } else {
    numerator = smoothed_reading - scale.OFFSET;
  }
  weight = numerator / scale.SCALE;
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

template <typename T>
void debugPrint(T msg)
{
  if (debug == 1) {
    Serial.print(msg);
  }
}

template <typename T>
void debugPrintln(T msg)
{
  if (debug == 1) {
    Serial.println(msg);
  }
}

void calibrate(void)
{
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
  int positive = 1;  // 1 for positive param, 0 for negative.
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

/* Count the number of times the tare button
   is pressed within a millisecond window */
int countTarePresses(int window)
{
  curr_time = millis();
  prev_time = millis();
  int count = 0;
  int newState = LOW;
  int oldState = LOW;
  while (curr_time - prev_time < window) {
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

void setup()
{
  // initialize the tare button.
  pinMode(tarePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(debugPin, INPUT_PULLUP);
  // initialize the LED.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  bool debugState = digitalRead(debugPin);
  if (debugState == LOW) {
    debug = 1;
  }

  // If tare held at startup, enter calibration mode.
  tareState = digitalRead(tarePin);
  if (tareState == HIGH) {
    doCalibrate = 1;
    setLEDColor(255, 255, 255);  // white
  } else {
    initFlash();
    DEVICE_CODE = readDefaultDevice();
    // At startup/after calibration, show green for 1s
    // allowing user to press tare to enter different modes.
    setLEDColor(0, 255, 0);  // green
    int num_presses = countTarePresses(1000);
    if (num_presses > 0) {
      /* If tare pressed, enter device selection */
      flashLED();
      uint8_t curr_device = 0;  // WH06
      while (1) {
        switch (curr_device) {
          case 0:  // WH06
            setLEDColor(0, 200, 255);  // light blue
            break;
          case 1:  // Tindeq
            setLEDColor(255, 255, 0);  // yellow
            break;
        }
        num_presses = countTarePresses(1000);
        if (num_presses == 1) {
          // Go to the next device.
          curr_device = (curr_device + 1) % NUM_DEVICES;
        } else if (num_presses == 2) {
          // Save the current device as default and exit the loop.
          DEVICE_CODE = curr_device;
          flashLED();
          saveDefaultDevice(DEVICE_CODE);
          break;
        }
      }
    }
  }

  // Initialize the device.
  switch (DEVICE_CODE) {
    case 0:
      setLEDColor(0, 200, 255);  // light blue
      device = new WH06();
      break;
    case 1:
      setLEDColor(255, 255, 0);  // yellow
      device = new Tindeq();
      break;
  }

  if (debug == 1) {
    Serial.begin(115200);
    while ( !Serial ) delay(10);   // for nrf52840 with native usb
    Serial.println("Starting.");
  }
  // Initialize the scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, LOADCELL_GAIN);
  delay(250);
  scale.power_up();
  delay(250);
  if (scale.wait_ready_timeout(1000)) {
    debugPrintln("HX711 found");
  }
  scale.set_gain(LOADCELL_GAIN);

  if (doCalibrate == 1) {
    uint8_t* rgb = getLEDColor();
    calibrate();
    setLEDColor(rgb[0], rgb[1], rgb[2]);
    delete[] rgb;
  }

  // Set the SCALE.
  float scale_param = readScaleParam();
  debugPrint("Scale set to ");
  debugPrintln(scale_param);
  scale.set_scale(scale_param);
  //scale.set_scale();
  tare();

  // Start BLE
  device->begin();
  debugPrintln("BLE Advertising");
}

void loop() {
  tareState = digitalRead(tarePin);
  // Tare when button pressed.
  if (tareState == HIGH) {
    tare();
  }

  //buttonState = digitalRead(buttonPin);
  //if (buttonState == HIGH) {
  //  weight += 1;
  //} else {
  //  if (weight > 0) {
  //    weight -= 1;
  //  }
  //}
  //debugPrint(weight);
  //debugPrint(",");
  //debugPrint(buttonState);
  //debugPrint(",");
  //for (int i=2; i<6; i++) {
  //  debugPrint(device->getScaleData()[i]);
  //}
  //debugPrintln("");

  // Update the advertisement every time we get a new weight.
  // This is 10Hz by default on the HX711, but can be increased
  // to 80Hz via the RATE pin.
  int got_weight = getWeight();
  //int got_weight = getWeightFiltered();
  if (got_weight == 1) {
    curr_time = millis();
    device->updateWeight(weight);
    device->updateTimestamp(curr_time);
    device->updateAdvData();
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
