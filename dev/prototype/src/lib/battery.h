#include <Arduino.h>


#define VBAT_MV_PER_LSB   (0.5859375F)    // 2.4V ADC range and 12-bit ADC resolution = 2400/4096
#define VBAT_DIVIDER_COMP (2.96078F)      // (1000+510)/510  Voltage divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


void initBattery(void) {
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_2_4);
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  pinMode(PIN_VBAT, INPUT);
}


float readVBAT(void) {
  float raw;
  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(PIN_VBAT);
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..2400 and resolution is 12-bit (0..4095)
  //return raw * REAL_VBAT_MV_PER_LSB;
  return raw;
}

uint8_t mvToPercent(float mvolts) {
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}
