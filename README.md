# Mito

A tiny, cheap, hackable, completely open-source force gauge for isometric finger strength training.


![Mito](https://github.com/jvasilakes/mito/blob/main/pics/1000009354_cropped.jpg?raw=true)


## Hardware information

The Mito is based on the Seeed XIAO NRF52840 microcontroller, an HX711 ADC, and a custom load cell. It runs at approximately 80 samples per second on the hardware side, and sends approximately 60 samples per second over bluetooth. Initial tests show a standard error of about 100g. The load cell and two Peguet 3.5mm stainless steel maillon rapides provide a working load limit of 220kg and a breaking load of 1100kg. 

![Mito PCB](https://github.com/jvasilakes/mito/blob/main/hardware/mito.png?raw=true)


## User Guide

On one side of your device you'll see

 1. A hardware on/off switch
 2. An LED indicator
 3. A tare button

On the other there is a USB-C port that is used for battery charging and firmware updates.

Assuming you've obtained a prebuilt device, it should be ready to go out of the box. Flip the switch to turn it on, and you'll see a green light for 1 second before it changes to yellow. Download the Frez app on your phone, register/sign in to your account, then press the "connect" button on the main page. If the Mito is powered on, a scan of available devices should reveal a device called "Progressor". Connect to it, register it as "Mito" or whatever name you like, and start training!


### Automatic sleep mode

After 1 minute of inactivity (i.e., no weight changes) the Mito will enter sleep mode to save battery. This is indicated by a periodic flashing LED. To wake your Mito up, simply press the tare button and continue your training. After 10 minutes in light sleep, the Mito will enter deep sleep from which it can only be woken up by turning it off and on again.


### Charging the battery

The Mito should need charging very infrequently. The current estimate is at least a few weeks of regular usage before the battery will run low. To charge, plug the Mito into a power source using a USB-C cable and turn it on with the hardware switch. The Mito will enter sleep mode after 1 minute of inactivity and continue to charge, indicated by a longer blinking LED.


### Firmware updates

Firmware updates will be provided here on the Releases page, so keep an eye out. To update your device's firmware with a new version, follow these steps:

 1. Download the new firmware UF2 file onto your computer (e.g., `mito_v0.0.1.uf2`).
 2. Plug your Mito into your computer using the USB-C port while holding down the tare button. Release the button once you see the red light. You've now entered maintenance mode selection.
 3. Use single presses of the tare button to cycle through the maintenance modes until you see a blue light, which indicates firmware update mode. Double click the tare button to select it.
 4. The Mito should turn off and after a few moments a USB device called XIAO-SENSE should pop up on your computer. Simply drag and drop the new UF2 file onto the root folder of this device. Once the copy is complete, the Mito should reset.
 5. Unplug the USB cable and start training!


### Changing the default device mode

The Mito is compatible with the Frez app for visualizing and tracking your training sessions. By default the Mito is in Tindeq mode (yellow light), meaning that it pretends to be a Tindeq device. This mode currently provides the most stable and fastest bluetooth connection to the Frez app. The Tindeq app is currently not supported, and your experience may vary.

If for whatever reason you want your Mito to pretend to be a WH-06 (the original device around which Frez was built), follow these steps:

 1. Turn your Mito on with the hardware switch.
 2. While the initial green light is showing, press the tare button to enter device selection mode. The light should turn blue.
 3. A single press of the tare button will cycle through the available device modes. Blue indicates the WH-06 while yellow indicates the Tindeq.
 4. Press the tare button twice quickly to change the default device to the current selection. The Mito will start up and from now on will be in your chosen device mode every time you turn it on.

If you want to abort this and stay with your current default device, simply turn the Mito off and back on again.


## Known Issues


### Frez Connectivity

It can happen that if you connect your Mito to Frez or another app (e.g., Tindeq), close the app, and the try to reconnect, the initialization will hang and you'll get a device disconnected error. This can be fixed by simply turning the Mito off and on again, and restarting the app.


### Battery Level Indicator

Frez shows battery level next to the connected device icon in the lower right corner. Unfortunately, the battery level sampling on the XIAO microcontroller isn't the most accurate, so you may see inconsistnet values. Overall, however, a low battery indication here is a good sign its time to charge.



## Advanced Usage


### Calibration

All pre-built Mito's are already calibrated. If you need to redo it however, follow these steps:

 1. Download "Serial Bluetooth Terminal" app on your phone. 
 2. Turn on the Mito while holding down the tare button to enter maintenance mode.
 3. Use single presses of the tare button to cycle through modes until you see a white light. Press the tare button twice quickly to select this, which will enter calibration mode.
 4. Open the Serial Bluetooth Terminal app, scan for devices, and select the Mito.
 5. A terminal window should open. Hang the Mito on something solid, but keep the other anchor free. Once you see `Connecting to Mito... Connected` in the terminal window, send an empty command to start the calibration process. 
 6. After you see `Tared. Enter weight. grams: ` in the terminal, hang a known weight on the free anchor of the Mito. Make sure the weight is stationary and not swinging, twirling, etc.
 7. Enter the weight in grams (e.g., for a 20kg plate you would enter `20000`) and hit send.
 8. The calibration will now run for a few iterations. At the end it will print `Estimated parameter:  Saving...`. 
 9. Power cycle the Mito, connect to the Frez app, and validate that the reading is correct for the weight you used to calibrate. 


### Configuration

You can manually set the following variables in `prototype/src/lib/config.h`. For the changes to take effect, you will need to recompile and upload the firmware to your Mito (see below).

 * `LIGHT_SLEEP_SECONDS`: The number of seconds with no weight changes until the Mito enters light sleep. Default 60.
 * `DEEP_SLEEP_SECONDS`: The number of seconds in light sleep until the Mito enters deep sleep. Default 600.
 * `SCALE_FACTOR`: The calibration scale factor. Default set during calibration (see above). This can be used to fine-tune your precise calibration factor.


### Building the firmware

First, clone this repository or download the code for your chosen release. Then,

```
cd dev
arduino-cli compile --export-binaries --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
python ../utils/uf2conv.py prototype/build/Seeeduino.nrf52.xiaonRF52840Sense/prototype.ino.hex --family 0xADA52840 --convert --output prototype/releases/mito_vX.X.X.uf2 
```

Change the `X.X.X` to your desired version. If you've made changes to an official release, I recommend adding a suffix to indicate it (e.g., `mito_v0.0.4_dev.uf2`).

### Debug Mode

If you're doing development and want to access information over serial, plug your Mito into your computer while holding down the tare button. Use single presses of the tare button until you see a red light. Double click the tare button to enter debug mode. In this mode, the Mito will not start its initialization routine until you've connected to it with a serial monitor. 

Once the Mito has begun measuring, it will print data in the following format for each measurement taken from the ADC:

`adc_reading,smoothed_adc_reading,scale_offset,scale_calibration_factor,weight_grams,device_sample_rate_hz,ble_notify_rate_hz,battery_is_charging,battery_voltage,battery_percentage`

It will also print out any commands it receives from the central, and when the device is charging in sleep mode it will print out the current battery percentage.


### Entering firmware update mode with the hardware reset button

It might happen that a failed firmware update means that you can't enter firmware update mode using the method described above. If this happens you'll have to use the hardware reset button on the microcontroller. You can do this in two ways:

 1. Insert a small thin object into the hole below the 'S' on the CRUX TOOLS side of the Mito. a bobby pin works great for this. You'll have to fish around a bit, but the XIAO's reset button should be directly below this. Do your best to double click it. If you're successful, the Mito will turn off and you'll see the XIAO-SENSE USB device pop up on your computer. 
 2. If the above is simply too annoying, you can also open the case using a standard Philip's head screwdriver. If you do this, there is a chance you will have to re-calibrate after putting it together again, since the load cell can get shifted slightly.
