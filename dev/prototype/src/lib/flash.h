#ifndef FLASH_h
#define FLASH_h

void initFlash(void);
void saveScaleParam(float param);
float readScaleParam();
void saveDefaultDevice(uint8_t device_code);
uint8_t readDefaultDevice(void);

#endif /* FLASH_h */
