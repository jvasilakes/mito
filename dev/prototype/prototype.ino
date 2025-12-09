#include <iostream>
#include <memory>
#include <string.h>
#include <Adafruit_TinyUSB.h>
#include "src/lib/battery.h"
#include "src/lib/HX711.h"
#include "src/lib/flash.h"
#include "src/lib/scales.h"
#include "src/lib/debug.h"


#define LIGHT_SLEEP_TIMEOUT 60000  // 1 minute
#define DEEP_SLEEP_TIMEOUT 180000  // 3 minutes


/* Set in setup() */
uint8_t DEVICE_CODE;  // 0: WH06  1: Tindeq
int NUM_DEVICES = 2;

// Tare button
const uint8_t tarePin = 7;  // the tare button
bool tareState = 0;  // tare button push state.

bool debug = 0;  // Enter debug mode, which opens serial over usb.
bool doCalibrate = 0;  // Enter calibration mode.
bool enter_dfu = 0;  // Enter DFU mode and reset.

// RGB LED pins
const uint8_t redPin = 4;    // the number of the LED pin
const uint8_t greenPin = 5;    // the number of the LED pin
const uint8_t bluePin = 6;    // the number of the LED pin
// Current LED color
uint8_t color[3] = { 0, 0, 0 };

// Load Cell parameters
const uint8_t LOADCELL_DOUT_PIN = 9;
const uint8_t LOADCELL_SCK_PIN = 10;
const uint8_t LOADCELL_GAIN = 128;

// Parameter for Exponential Moving Average
int EMA_ALPHA = 30;

/* ========== Global Variables ========== */
long reading = 0;  // Raw ADC reading.
long smoothed_reading = 0;  // For computing exponential moving average.
uint32_t weight = 0;  // Current weight reading in grams.
uint32_t prev_weight = 0;  // Previous weight reading in grams.

uint32_t curr_time = 0;  // Current time stamp
uint32_t prev_time = 0;  // To measure increments without delay() 

uint32_t measure_time = 0;  // Microseconds since measurement started
uint32_t start_measure_time = 0;  // Microseconds since measurement started
bool currently_measuring = 0;

uint32_t sleepTimeoutStart = 0;

uint16_t num_samples = 0;  // Number of samples obtained from the ADC
uint16_t hz = 0;  // For measuring HX711 speed

float avg_battery_voltage = 0.0;
float prev_vbat = -1.0;

/* ========= The device =========== */
HX711 scale;  // The ADC
Device* device = nullptr;  // The overall device: WH06, Tindeq
Xiao battery;

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
  // Round weight to nearest 50 grams
  weight = weight - (weight % 50);

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
  debugPrint(hz);
  debugPrint(",");
  debugPrint(curr_time);
  debugPrint(",");
  debugPrint(battery.IsChargingBattery());
  debugPrint(",");
  debugPrint(avg_battery_voltage);
  debugPrint(",");
  debugPrintln(getBatteryPercentage());
  return rval;
}

float getBatteryPercentage() {
  float vbat = battery.GetBatteryVoltage();
  // There is some noise in the battery charging ADC
  // If vbat is much greater than the previous reading
  // Just keep the lower reading.
  if (prev_vbat == -1.0) { prev_vbat = vbat; }
  if (vbat - prev_vbat > 0.1) {
    vbat = prev_vbat;
  } else {
    prev_vbat = vbat;
  }
  avg_battery_voltage = ((EMA_ALPHA * vbat) + ((100 - EMA_ALPHA) * avg_battery_voltage))/100;
  if (avg_battery_voltage <= 3.3) {
    return 0.0;
  } else if (avg_battery_voltage >= 4.2) {
    return 1.0;
  } else {
    return (avg_battery_voltage - 3.3) / (4.2 - 3.3);
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
  if (battery.IsChargingBattery()) {
    // set LED to a color indicating charging
  } else {
    turnOffLED();
  }
  uint8_t wakePin = g_ADigitalPinMap[tarePin];  // Map Arduino pin to nRF pin
  nrf_gpio_cfg_sense_input(wakePin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  NRF_POWER->SYSTEMOFF = 1;
}

void lightSleep() {
  uint8_t* rgb = getLEDColor();
  flashLED();
  scale.power_down();
  turnOffLED();
  int sleep_start_time = millis();
  while (1) {
    tareState = digitalRead(tarePin);
    if (tareState == HIGH) {
      break;
    }
    if (sleepTimeoutStart > 0 && ((millis() - sleepTimeoutStart) > DEEP_SLEEP_TIMEOUT)) {
      if (DEVICE_CODE != 1) {
        // Don't enter deep sleep when Tindeq, as this will end the training
        //   session in the app without saving progress. Annoying if you've programmed
        //   in rests longer than 3 minutes.
        // If we enter deep sleep, a full system reset will occur on exit.
        enterDeepSleep();
      }
    }
    int blink_time = 100;
    uint8_t blink_rgb[3] = { 0 };
    if (battery.IsChargingBattery()) {
      float batt_level = getBatteryPercentage();
      debugPrintln(batt_level);
      blink_time = 1500;
      // Pulse LED with charge status color
      // Green, yellow, red.
      if (batt_level >= 0.75) {
        blink_rgb[1] = 255;  // Green
      } else if (batt_level >= 0.25) {
        blink_rgb[0] = 255;
        blink_rgb[1] = 255;  // Yellow
      } else {
        blink_rgb[0] = 255;  // Red
      }
    } else {
      blink_rgb[0] = rgb[0];
      blink_rgb[1] = rgb[1];
      blink_rgb[2] = rgb[2];
    }
    if (millis() - sleep_start_time >= 2000) {
      setLEDColor(blink_rgb[0], blink_rgb[1], blink_rgb[2]);
      delay(blink_time);
      turnOffLED();
      sleep_start_time = millis();
    }
  }
  scale.power_up();
  setLEDColor(rgb[0], rgb[1], rgb[2]);
  flashLED();
}

void setup()
{
  // initialize the tare button.
  pinMode(tarePin, INPUT);
  // initialize the LED.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize with the current battery voltage.
  avg_battery_voltage = battery.GetBatteryVoltage();

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
          enter_dfu = 0;
          break;
        case 1:  // run calibrate
          setLEDColor(255, 255, 255);  // white
          debug = 0;
          doCalibrate = 1;
          enter_dfu = 0;
          break;
        case 2:  // DFU
          setLEDColor(225, 0, 255);  // purple
          debug = 0;
          doCalibrate = 0;
          enter_dfu = 1;
      } // end switch interrupt_mode
      num_presses = countTarePresses(1000);
      if (num_presses == 1) {
        interrupt_mode = (interrupt_mode + 1) % 3;
      } else if (num_presses == 2) {
        flashLED();
        if (enter_dfu == 1) {
          // See https://forums.adafruit.com/viewtopic.php?t=218553
          NRF_POWER->GPREGRET = 0x57;  // DFU_MAGIC_UF2_RESET
          NVIC_SystemReset();
        } else if (doCalibrate == 1) {
          device = new Mito();
          // Downcast to Mito to access calibrate()
          if (Mito* mito = static_cast<Mito*>(device)) {
            mito->calibrate(scale);
            NVIC_SystemReset();
          } else {
            // Something went terribly wrong.
            setLEDColor(255, 0, 0);
            while (1) { flashLED(); }
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
      // device = new Forceboard();
      debugPrintln("Device: WH06");
      // debugPrintln("Device: Forceboard");
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
  tare();

  // Start BLE
  device->begin();
  debugPrintln("BLE Advertising");
}

void loop() {

  int weight_diff = prev_weight - weight;
  if (weight_diff < 0) {
    weight_diff *= -1;
  }
  // Detect differences of 100g or more only.
  if (sleepTimeoutStart == 0 || weight_diff >= 100) {
    sleepTimeoutStart = millis();
  }

  tareState = digitalRead(tarePin);
  // Tare when button pressed.
  if (tareState == HIGH) {
    tare();
  }

  char cmd = device->getCommand();
  switch (cmd) {
    case 0x64:  // Tare
      tare();
      break;
    case 0x65:  // Start measurement
      if (currently_measuring == 0) {
        measure_time = start_measure_time = micros();
      }
      currently_measuring = 1;
      break;
    case 0x66:  // Stop measurement
      currently_measuring = 0;
      break;
    case 0x6f:  // battery
      float volts = battery.GetBatteryVoltage();
      uint32_t mv = volts * 1000.0;
      device->updateBatteryLevel(mv);
      device->updateBatteryAdv();
  }

  // Update the advertisement every time we get a new weight.
  // This is 10Hz by default on the HX711, but can be increased
  // to 80Hz via the RATE pin.
  if (currently_measuring == 1) {
    prev_weight = weight;
    int got_weight = getWeight();
    if (got_weight == 1) {
      measure_time = micros();
      device->updateWeight(weight);
      device->updateTimestamp(measure_time - start_measure_time);
      device->updateAdvData();
      num_samples += 1;
    }
  }

  if (sleepTimeoutStart > 0 && ((millis() - sleepTimeoutStart) > LIGHT_SLEEP_TIMEOUT)) {
    lightSleep();
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

  // Poll the battery
  // If battery level is low, change LED to red.
  float batt_level = getBatteryPercentage();
  if (batt_level <= 0.1) {
    setLEDColor(255, 0, 0);
  }
}
