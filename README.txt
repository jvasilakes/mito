conda activate ble
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
arduino-cli board list  # Get /dev/ttyACM number
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:nrf52:xiaonRF52840Sense prototype
