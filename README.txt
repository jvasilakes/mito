# Mito

A small, cheap, and completely open-source force gauge to rival the Tindeq and Forceboard.

![Mito PCB](https://github.com/jvasilakes/mito/tree/master/hardware/mito.png)


## Building the firmware

```
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
arduino-cli board list  # Get /dev/ttyACM number
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
```
