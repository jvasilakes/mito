#include <iostream>
#include <memory>
#include <string.h>
#include "Adafruit_TinyUSB.h"
#include "src/lib/HX711.h"
#include "src/lib/flash.h"
#include "src/lib/scales.h"


#define SLEEP_TIMEOUT 60000


/* Set in setup() */
uint8_t DEVICE_CODE;
// int DEVICE_CODE = 0;  // WH06
// int DEVICE_CODE = 1;  // Tindeq
int NUM_DEVICES = 2;

// Tare button
const uint8_t tarePin = 7;  // the tare button
bool tareState = 0;  // tare button push state.

// External button
const uint8_t buttonPin = 0;
bool buttonState = 0;

// Debug pin. If grounded enter debug mode.
bool debug = 0;
bool doCalibrate = 0;

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
uint32_t weight = 0;  // Current weight reading in grams.
uint32_t prev_weight = 0;  // Previous weight reading in grams.
bool weight_changed = 0;  // To keep track of sleep timeout.
uint32_t prev_time = 0;  // To measure increments without delay() 
uint32_t curr_time = 0;  // Current time stamp
uint16_t num_samples = 0;
uint16_t hz = 0;  // For measuring HX711 speed.
uint32_t sleepTimeoutStart = 0;
bool is_charging = 0;
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
  smoothed_reading = ((EMA_ALPHA * midr) + ((100 - EMA_ALPHA) * smoothed_reading))/100;
  int numerator; 
  if (smoothed_reading < scale.OFFSET) {
    numerator = 0;
  } else {
    numerator = smoothed_reading - scale.OFFSET;
  }
  weight = numerator / scale.SCALE;

  debugPrint(reading);
  debugPrint(",");
  debugPrint(smoothed_reading);
  debugPrint(",");
  debugPrint(scale.OFFSET);
  debugPrint(",");
  debugPrint(scale.SCALE);
  debugPrint(",");
  debugPrint(midr);
  debugPrint(",");
  debugPrint(weight);
  debugPrint(",");
  debugPrintln(hz);
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

/* Count the number of times the tare button
   is pressed within a millisecond window */
int countTarePresses(int window)
{
  window *= 1000;  // since curr/prev_time are microseconds.
  curr_time = micros();
  prev_time = micros();
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
    curr_time = micros();
    delay(50);  // Debounce the button.
  }
  curr_time = micros();
  prev_time = micros();

  return count;
}

void enterDeepSleep() {
  flashLED();
  if (is_charging) {
    // set LED to a color indicating charging
  } else {
    turnOffLED();
  }
  uint8_t wakePin = g_ADigitalPinMap[tarePin];  // Map Arduino pin to nRF pin
  nrf_gpio_cfg_sense_input(wakePin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  NRF_POWER->SYSTEMOFF = 1;
}

void setup()
{
  // initialize the tare button.
  pinMode(tarePin, INPUT);
  pinMode(buttonPin, INPUT);
  // initialize the LED.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  initFlash();
  DEVICE_CODE = readDefaultDevice();

  // Initialize the scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, LOADCELL_GAIN);
  delay(250);
  scale.power_up();

  // If tare held at startup, enter debug or calibrate mode
  tareState = digitalRead(tarePin);
  if (tareState == HIGH) {
    int num_presses;
    int interrupt_mode = 0;
    while (1) {
      switch (interrupt_mode) {
        case 0:  // debug mode
          setLEDColor(255, 180, 0);  // orange
          debug = 1;
          doCalibrate = 0;
          break;
        case 1:  // run calibrate
          setLEDColor(255, 255, 255);  // white
          debug = 0;
          doCalibrate = 1;
          break;
      } // end switch interrupt_mode
      num_presses = countTarePresses(1000);
      if (num_presses == 1) {
        interrupt_mode = (interrupt_mode + 1) % 2;
      } else if (num_presses == 2) {
        flashLED();
        if (doCalibrate == 1) {
          device = new Mito();
          // Downcast to Mito to access calibrate()
          if (Mito* mito = static_cast<Mito*>(device)) {
            mito->calibrate(scale);
            NVIC_SystemReset();
          } else {
            // Something went terribly wrong.
            setLEDColor(255, 0, 0);
            while (1) {flashLED();}
          }  // end if downcast
        }  // end if doCalibrate 
        break;
      } else { continue; }
    }  // end while
  // Otherwise, show green for 1s at startup
  // allowing user to press tare to enter different modes.
  } else {
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

  if (debug == 1) {
    Serial.begin(115200);
    while ( !Serial ) delay(10);   // for nrf52840 with native usb
    Serial.println("Starting.");
  }

  if (scale.wait_ready_timeout(1000)) {
    debugPrintln("HX711 ready.");
  }

  // Initialize the device.
  switch (DEVICE_CODE) {
    case 0:
      setLEDColor(0, 200, 255);  // light blue
      device = new WH06();
      debugPrintln("Device: WH06");
      break;
    case 1:
      setLEDColor(255, 255, 0);  // yellow
      device = new Tindeq();
      debugPrintln("Device: Tindeq");
      break;
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

  if (sleepTimeoutStart == 0 || (prev_weight != weight)) {
    sleepTimeoutStart = millis();
  }

  tareState = digitalRead(tarePin);
  // Tare when button pressed.
  if (tareState == HIGH) {
    tare();
  }

  // Update the advertisement every time we get a new weight.
  // This is 10Hz by default on the HX711, but can be increased
  // to 80Hz via the RATE pin.
  prev_weight = weight;
  int got_weight = getWeight();
  if (got_weight == 1) {
    curr_time = micros();
    device->updateWeight(weight);
    device->updateTimestamp(curr_time);
    device->updateAdvData();
    num_samples += 1;
  }

  if (sleepTimeoutStart > 0 && ((millis() - sleepTimeoutStart) > SLEEP_TIMEOUT)) {
    enterDeepSleep();
    // Will sleep until tare button pressed.
    sleepTimeoutStart = millis();
  }

  // In case of overflow.
  if (curr_time < prev_time) {
    curr_time = prev_time = micros();
  }

  // Measure sampling rate.
  if ((curr_time - prev_time) >= 1000000) {
    hz = num_samples;
    curr_time = prev_time = micros();
    num_samples = 0;
  }
}
