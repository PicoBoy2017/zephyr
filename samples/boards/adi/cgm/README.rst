.. zephyr:code-sample:: adi cgm
   :name: CGM
   :relevant-api: Bluetooth

   A sample reference design for a Continuous Glucose Monitor (CGM) device.

Overview
********

The CGM demo combines multiple subsystems, threads, and drivers to create a CGM reference
software design. Use and extend this design to implement the desired device functionality
and customization.

The CGM Zephyr app monitors data from an AFE electrochemical sensor, calculates patient
glucose levels and transmits the determined glucose level to the central device.

All features of this demo are interactable through the CodeFusion Studio mobile app.

Main thread:

- Entry point for the application
- Spawns threads, initializes sensors and BLE services
- Defines the BLE connection and pairing logic

Electrochemical sensor monitor thread:

- Periodically fetches data from the connected AFE
- Processes the data to calculate glucose levels

BLE continuous glucose monitoring services:

- Periodically transmit glucose level to central device through BLE
- Manages database for stored records

LED indicator thread:

- Indicates the Bluetooth status of the device

  - Double blink: searching for connection
  - Solid: connected

Accelerometer service thread:

- Custom BLE service for real-time sensor streaming

Battery service monitor thread:

- Periodic thread to transmit battery level to central device

BLE shell backend:

- Allow for Zephyr shell commands to be sent over BLE


Requirements
************

Your board must:

#. Be BLE enabled
#. Have an electrochemical AFE sensor
#. Have an accelerometer sensor
#. Have a push-button
#. Have an LED connected via a GPIO pin (these are called "User LEDs" on many of
   Zephyr's :ref:`boards`).

Building and Running
********************

Build and flash Blinky as follows, changing ``reel_board`` for your board:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/adi/cgm
   :board: max32657evkit
   :goals: build flash
   :compact:

After flashing, the LED will 'double blink' to indicate the device is searching
for a connection. Once a connection is established, the LED will remain solid.

Tips:

- This sample only allows for a single bonded device, as defined in main.c. To
  clear bonded devices, press the push-button on the evkit.
- When sending commands over the Zephyr shell BLE backend, the shell will continue
  buffering input until a newline character is sent. Upon which, the buffered input
  is evaluated.
