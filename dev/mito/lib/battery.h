/* Adapted from 
 * https://github.com/honvl/Seeed-Xiao-NRF52840-Battery */


#include <Arduino.h>
#include <bluefruit.h>

#define BAT_HIGH_CHARGE 22  // HIGH for 50mA, LOW for 100mA
#define BAT_CHARGE_STATE 23 // LOW for charging, HIGH not charging
#define VBAT_MV_PER_LBS (0.003395996F)


class XiaoBattery {
public:
  XiaoBattery();
  float GetBatteryVoltage();
  bool IsChargingBattery();
};

XiaoBattery::XiaoBattery() {
  pinMode(VBAT_ENABLE, OUTPUT);
  pinMode(BAT_CHARGE_STATE, INPUT);

  digitalWrite(BAT_HIGH_CHARGE, LOW); // charge with 100mA
}

float XiaoBattery::GetBatteryVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);

  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_MV_PER_LBS;
  float vBat = adcVoltage * (1510.0 / 510.0);

  digitalWrite(VBAT_ENABLE, HIGH);

  return vBat;
}

bool XiaoBattery::IsChargingBattery() { return digitalRead(BAT_CHARGE_STATE) == LOW; }
