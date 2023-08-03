"Extra" Tock Capsules
=====================

This crate contains miscellaneous capsules which do not fit into any other, more
specific category, and which do not require any external (non-vendored and
unvetted) dependencies.

For more information on capsules, see [the top-level README](../README.md).

The remainder of this document contains a list of capsules in this crate, along
with a short description.

Sensor and other IC Drivers
---------------------------

These implement a driver to setup and read various physical sensors.

- **[ADC Microphone](src/adc_microphone.rs)**: Single ADC pin microphone.
- **[Analog Sensors](src/analog_sensor.rs)**: Single ADC pin sensors.
- **[APDS9960](src/apds9960.rs)**: Proximity sensor.
- **[BME280](src/bme280.rs)**: Humidity and air pressure sensor.
- **[BMP280](src/bmp280.rs)**: Temperature (and air pressure) sensor.
- **[CCS811](src/ccs811.rs)**: VOC gas sensor.
- **[FXOS8700CQ](src/fxos8700cq.rs)**: Accelerometer and magnetometer.
- **[HTS221](src/hts221.rs)**: Temperature and humidity sensor.
- **[ISL29035](src/isl29035.rs)**: Light sensor.
- **[L3GD20](src/l3gd20.rs)**: MEMS 3 axys digital gyroscope and temperature
  sensor.
- **[LSM303xx Support](src/lsm303xx.rs)**: Shared files.
  - **[LSM303AGR](src/lsm303agr.rs)**: 3D accelerometer and 3D magnetometer
    sensor.
  - **[LSM303DLHC](src/lsm303dlhc.rs)**: 3D accelerometer and 3D magnetometer
    sensor.
- **[LSM6DSOXTR](src/lsm6dsoxtr.rs)**: 3D accelerometer and 3D magnetometer
    sensor.
- **[LPS25HB](src/lps25hb.rs)**: Pressure sensor.
- **[MLX90614](src/mlx90614.rs)**: Infrared temperature sensor.
- **[RP2040 Temperature](src/temperature_rp2040.rs)**: Analog RP2040 temperature
  sensor.
- **[SHT3x](src/sht3x.rs)**: SHT3x temperature and humidity sensor.
- **[SI7021](src/si7021.rs)**: Temperature and humidity sensor.
- **[STM32 Temperature](src/temperature_stm.rs)**: Analog STM32 temperature
  sensor.
- **[TSL2561](src/tsl2561.rs)**: Light sensor.

These drivers provide support for various ICs.

- **[FM25CL](src/fm25cl.rs)**: FRAM chip.
- **[FT6x06](src/ft6x06.rs)**: FT6x06 touch panel.
- **[HD44780 LCD](src/hd44780.rs)**: HD44780 LCD screen.
- **[LPM013M126](src/lpm013m126.rs)**: LPM013M126 LCD screen.
- **[LTC294X](src/ltc294x.rs)**: LTC294X series of coulomb counters.
- **[MAX17205](src/max17205.rs)**: Battery fuel gauge.
- **[MCP230xx](src/mcp230xx.rs)**: I2C GPIO extender.
- **[MX25r6435F](src/mx25r6435f.rs)**: SPI flash chip.
- **[PCA9544A](src/pca9544a.rs)**: Multiple port I2C selector.
- **[SD Card](src/sdcard.rs)**: Support for SD cards.
- **[Seven Segment Display](src/seven_segment.rs)**: Seven segment displays.
- **[ST77xx](src/st77xx.rs)**: ST77xx IPS screen.


Wireless
--------

Support for wireless radios.

- **[nRF51822 Serialization](src/nrf51822_serialization.rs)**: Kernel support
  for using the nRF51 serialization library.
- **[RF233](src/rf233.rs)**: Driver for RF233 radio.
- **[BLE Advertising](src/ble_advertising_driver.rs)**: Driver for sending BLE
  advertisements.
- **[LoRa Phy]**: Support for exposing Semtech devices to userspace
  See the lora_things_plus board for an example

Libraries
---------

Protocol stacks and other libraries.

- **[IEEE 802.15.4](src/ieee802154)**: 802.15.4 networking.
- **[Networking](src/net)**: Networking stack.
- **[USB](src/usb)**: USB 2.0.
- **[Segger RTT](src/segger_rtt.rs)**: Segger RTT support. Provides `hil::uart`
  interface.
- **[Symmetric Cryptography](src/symmetric_encryption)**: Symmetric
  encryption.
- **[Public Key Cryptography](src/public_key_crypto)**: Asymmetric
  encryption.


MCU Peripherals for Userspace
-----------------------------

These capsules provide a `Driver` interface for common MCU peripherals.

- **[Analog Comparator](src/analog_comparator.rs)**: Voltage comparison.
- **[CRC](src/crc.rs)**: CRC calculation.
- **[DAC](src/dac.rs)**: Digital to analog conversion.
- **[CAN](src/can.rs)**: CAN communication.


Helpful Userspace Capsules
--------------------------

These provide common and better abstractions for userspace.

- **[Air Quality](src/air_quality.rs)**: Query air quality sensors.
- **[Ambient Light](src/ambient_light.rs)**: Query light sensors.
- **[App Flash](src/app_flash_driver.rs)**: Allow applications to write their
  own flash.
- **[Buzzer](src/buzzer_driver.rs)**: Simple buzzer.
- **[CTAP](src/ctap.rs)**: Client to Authenticator Protocol (CTAP) support.
- **[Humidity](src/humidity.rs)**: Query humidity sensors.
- **[Key-Value Store](src/kv_driver.rs)**: Store key-value data.
- **[LED Matrix](src/led_matrix.rs)**: Control a 2D array of LEDs.
- **[Proximity](src/proximity.rs)**: Proximity sensors.
- **[Read Only State](src/read_only_state.rs)**: Read-only state sharing.
- **[Screen](src/screen.rs)**: Displays and screens.
- **[SHA](src/sha.rs)**: SHA hashes.
- **[Sound Pressure](src/sound_pressure.rs)**: Query sound pressure levels.
- **[Temperature](src/temperature.rs)**: Query temperature sensors.
- **[Text Screen](src/text_screen.rs)**: Text-based displays.
- **[Touch](src/touch.rs)**: User touch panels.


Virtualized Sensor Capsules for Userspace
-----------------------------------------

These provide virtualized (i.e. multiple applications can use them
simultaneously) support for generic sensor interfaces.

- **[Asynchronous GPIO](src/gpio_async.rs)**: GPIO pins accessed by split-phase
  calls.
- **[9DOF](src/ninedof.rs)**: 9DOF sensors (acceleration, magnetometer,
  gyroscope).
- **[Nonvolatile Storage](src/nonvolatile_storage_driver.rs)**: Persistent
  storage for userspace.


Utility Capsules
----------------

Other capsules that implement reusable logic.

- **[Nonvolatile to Pages](src/nonvolatile_to_pages.rs)**: Map arbitrary reads
  and writes to flash pages.
- **[HMAC](src/hmac.rs)**: Hash-based Message Authentication Code (HMAC) digest
  engine.
- **[Log Storage](src/log.rs)**: Log storage abstraction on top of flash
  devices.
- **[Bus Adapters](src/bus.rs)**: Generic abstraction for SPI/I2C/8080.
- **[TicKV](src/tickv.rs)**: Key-value storage.
- **[Key-Value Store](src/kv_store.rs)**: Key-value virtualized interface.
- **[SHA256](src/sha256.rs)**: SHA256 software hash.
- **[SipHash](src/sip_hash.rs)**: SipHash software hash.


Debugging Capsules
------------------

These are selectively included on a board to help with testing and debugging
various elements of Tock.

- **[Debug Process Restart](src/debug_process_restart.rs)**: Force all processes
  to enter a fault state when a button is pressed.
- **[Panic Button](src/panic_button.rs)**: Use a button to force a `panic!()`.
