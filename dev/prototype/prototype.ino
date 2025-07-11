#include "Adafruit_TinyUSB.h"
#include "HX711.h"
#include <bluefruit.h>

// Indices in the advertised data for things we plan to change.
#define SCALE_DATA_LEN           19   // See scale_data init below.
#define SCALE_DATA_WEIGHT_INT    12   // The integer part
#define SCALE_DATA_WEIGHT_FRAC   13   // The fractional part
#define SCALE_DATA_TIMESTAMP_MSB 17
#define SCALE_DATA_TIMESTAMP_LSB 18

// Tare button
const int tarePin = 10;  // the tare button
const int buttonPin = 9; // For pretending to get a weight
// RGB LED
const int redPin = 6;    // the number of the LED pin
const int greenPin = 5;    // the number of the LED pin
const int bluePin = 4;    // the number of the LED pin

// Load Cell
const uint8_t LOADCELL_DOUT_PIN = 9;
const uint8_t LOADCELL_SCK_PIN = 8;
const int LOADCELL_GAIN = 128;

int tareState = 0;  // tare button push state.
int buttonState = 0;

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

uint32_t weight = 0;  // Current weight reading
uint16_t curr_time = 0;  // Current time stamp
uint16_t prev_time = 0;  // To measure increments without delay() 

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

void tare(void)
{
  Serial.println("Taring...");
  scale.tare();
  weight = 0;
  flashLED();
}

void tareFake(void)
{
  weight = 0;
  flashLED();
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

uint32_t getWeight(void)
{
  // Take a reading from the HX711.
  if (scale.wait_ready_retry(10)) {
    long reading = scale.read();
    Serial.print("Reading: ");
    Serial.println(reading);
    return reading;
  } else {
    Serial.println("HX711 not found.");
    setColor(255, 0, 0); // Red
  }
  return 0;
}

uint32_t getWeightFake(void)
{
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    weight += 100;
  }
  return weight;
}


void updateWeight(uint32_t weight)
{
  // Update the scale_data with a weight reading.
  uint8_t msb = ((weight / 10) & 0xFF00U) >> 8U;
  uint8_t lsb = ((weight / 10) & 0x00FFU);
  scale_data[SCALE_DATA_WEIGHT_INT] = msb;
  scale_data[SCALE_DATA_WEIGHT_FRAC] = lsb;
  char sbuf[30];
  sprintf(sbuf, "Weight: %02X %02X %d", msb, lsb, weight);
  Serial.println(sbuf);
}

void updateTimestamp(uint16_t time)
{
  // Update the scale_data with the current timestamp.
  uint8_t msb = (time & 0xFF00U) >> 8U;
  uint8_t lsb = (time & 0x00FFU);
  scale_data[SCALE_DATA_TIMESTAMP_MSB] = msb;
  scale_data[SCALE_DATA_TIMESTAMP_LSB] = lsb;
  char sbuf[30];
  sprintf(sbuf, "Time: %02X %02X %d", msb, lsb, time);
  Serial.println(sbuf);
}

void updateAdvData(void)
{
  // Update the advertisement with the current scale data.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData(&scale_data, SCALE_DATA_LEN);
}

void setup() {
  // initialize the pushbutton.
  pinMode(tarePin, INPUT);
  pinMode(buttonPin, INPUT);
  // initialize the LED.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  // Set the color to a nice light blue.
  setColor(0, 175, 255);
  setLedColor();

  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Initialize the scale
  /*
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, LOADCELL_GAIN);
  delay(250);
  scale.power_up();
  delay(250);
  if (scale.wait_ready_timeout(1000)) {
    Serial.println("HX711 found");
  } else {
    Serial.println("HX711 not found.");
  }
  scale.set_gain(128);
  //scale.set_scale(436400.f);
  scale.set_scale();
  tare();
  */

  // Start BLE
  Bluefruit.begin();
  //Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  advertiseData();
  Serial.println(F("Advertising..."));
  Serial.println();  
}

void loop() {
  tareState = digitalRead(tarePin);
  // Tare when button pressed.
  if (tareState == HIGH) {
    //tare();
    tareFake();
  }

  curr_time = millis();
  if (curr_time - prev_time >= 20) {
    // Poll the HX711 and advertise the new reading every 20ms.
    prev_time = curr_time;
    //weight = getWeight();
    weight = getWeightFake();
    updateWeight(weight);
    updateTimestamp(curr_time);
    updateAdvData();
  } else if (curr_time < prev_time ) {  // Overflow so we reset.
    curr_time = 0;
    prev_time = 0;
  }
}
