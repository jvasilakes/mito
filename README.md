# Mito

The smallest, cheapest, and only completely open-source force gauge to rival the Tindeq and Forceboard for isometric strength training.


![Mito](https://github.com/jvasilakes/mito/blob/master/pics/1000009354_cropped.jpg?raw=true)


## Hardware information

The Mito is based on the Seeed XIAO NRF52840 microcontroller, an HX711 ADC, and a custom load cell. It runs at approximately 80 samples per second on the hardware side, and sends approximately 60 samples per second over bluetooth. Initial tests show a standard error of about 100g, and I ran into logistical issues adding weight before I could find the maximum supported weight.

![Mito PCB](https://github.com/jvasilakes/mito/blob/master/hardware/mito.png?raw=true)

The load cell is high quality stainless steel, and the anchor points are two Peguet 3.5mm stainless steel maillon rapides with a working load limit of 220kg and a breaking load of 1100kg. 


## User Guide

On one side of your device you'll see

 1. A hardware on/off switch
 2. An LED indicator
 3. A tare button

On the other there is a USB-C port that is used for battery charging and firmware updates.

Assuming you've obtained a prebuilt device, it should be ready to go out of the box. Flip the switch to turn it on, and you'll see a green light for 1 second before it changes to yellow, indicating that the Mito is in Tindeq mode. Download the Tindeq app on your phone, register/sign in to your account, then press the red button in the upper right corner to connect.


### Automatic sleep mode

After 1 minute of inactivity (i.e., no weight changes) the Mito will enter sleep mode to save battery. This is indicated by a periodic flashing green light. To wake your Mito up, simply press the tare button and continue your training.


### Charging the battery

The Mito should need charging very infrequently. The current estimate is at least a few weeks of regular usage before the battery will run low. To charge, plug the Mito into a power source using a USB-C cable and turn it on. The Mito will enter sleep mode after 1 minute of inactivity and continue to charge.


### Firmware updates

Firmware updates will be provided here on the Releases page, so keep an eye out. To update your device's firmware with a new version, follow these steps:

 1. Download the new firmware UF2 file onto your computer (e.g., `mito_v0.0.1.uf2`).
 2. Plug your Mito into your computer while holding down the tare button. Release the button once you see the red light. You've now entered maintenance mode selection.
 3. Use single presses of the tare button to cycle through the maintenance modes until you see a blue light, which indicates firmware update mode. Double click the tare button to select it.
 4. The Mito should turn off and after a few moments a USB device called XIAO-SENSE should pop up on your computer. Simply drag and drop the new UF2 file onto the root folder of this device. Once the copy is complete, the Mito should reset, and you're done!

If you run into errors with the drag and drop method in your computer's file explorer, it should work to copy over the new UF2 file on the command line. E.g., on my Linux machine I do

```
cp mito_v0.0.1.uf2 /media/jav/XIAO-SENSE/
```


### Changing the default device mode

The Mito is compatible with both the Tindeq and Frez apps for visualizing and tracking your training sessions. By default the Mito is in Tindeq mode, meaning that it pretends to be a Tindeq device. This mode currently provides the most stable and fast bluetooth connection to the Tindeq app, and the Frez app is compatible with the Tindeq as well, although it is a bit slow.

If for whatever reason you want your Mito to pretend to be a WH-06 (the original device around which Frez was built), follow these steps:

 1. Turn your Mito on with the hardware switch.
 2. While the initial green light is showing, press the tare button to enter device selection mode. The light should turn blue.
 3. A single press of the tare button will cycle through the available device modes. Blue indicates the WH-06 while yellow indicates the Tindeq.
 4. Press the tare button twice quickly to change the default device to the current selection. The Mito will start up and from now on will be in your chosen device mode every time you turn it on.

If you want to abort this and stay with your current default device, simply turn the Mito off and back on again.



## Advanced Usage


### Calibration

All pre-built Mito's are already calibrated. If you need to redo it however, follow these steps:

TODO


### Building the firmware

```
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
arduino-cli board list  # Get /dev/ttyACM number
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
```

### Debug Mode

If you're doing development and want to access information over serial, plug your Mito into your computer while holding down the tare button. Use single presses of the tare button until you see a red light. Double click the tare button to enter debug mode. In this mode, the Mito will not start its initialization routine until you've connected to it with a serial monitor. 


### Entering firmware update mode with the hardware reset button

It might happen that a failed firmware update means that you can't enter firmware update mode using the method described above. If this happens you'll have to use the hardware reset button on the microcontroller. You can do this in two ways:

 1. Insert a small thin object into the hole below the 'S' on the CRUX TOOLS side of the Mito. a bobby pin works great for this. You'll have to fish around a bit, but the XIAO's reset button should be directly below this. Do your best to double click it. If you're successful, the Mito will turn off and you'll see the XIAO-SENSE USB device pop up on your computer. 
 2. If the above is simply too annoying, you can also open the case using a standard Philip's head screwdriver.
