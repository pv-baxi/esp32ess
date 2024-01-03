# ESP32ESS documentation

## Hardware

The hardware for this project was built using the following components:
* ESP32-DevKitC V4 board (ESP32-WROOM-32D)
* MAX485 converter (resistors R5, R6, R7 removed)
* SN65HVD230 CAN bus transceiver
* LC Display 4x20 HD44780
* Optical impulse receiver circuit

Find a photo of my control board implementation below:

![control board photo](esp32ess_control_board.jpg)

### ESP32-DevKitC V4

V4 DevKit was used due to the improved WiFi antenna. But other boards will surely work as well.

### MAX485 converter for VE.Bus

The Victron VE.Bus is basically a RS485 bus. To control it, for example a
MAX485 converter circuit can be used. However, in contrary to RS485, the bus
needs to be high impedance >100kOhm, otherwise the connected Victron device
will turn off. To achieve that, the following resistors need to be removed
from a convential RS485 transceiver circuit:
* remove the 120 Ohm resistor between A(D+) and B(D-) pins
* remove any resistor from pins A & B (D+/-) towards Vcc or Gnd

The MAX485 circuit is connected to the ESP32 using the following three pins: DI, DE+RE(connected), RO.

All three pins are routed to the ESP32 through bidirectional level translators. For current pin assignment, please see "Constants"-section of the source code.

### SN65HVD230 CAN bus transceiver

In my hardware I'm using the Texas Instruments SN65HVD230 CAN bus transceiver, which has officially to be powered with 3.3V, nicely matching the operating voltage of the ESP32.

However, it took me quite some time to find out, that my cheap SN65HVD230 was a fake IC. With 3.3V operating voltage it's already able to read/sniff CAN bus messages, but it's not able to acknowledge them. Without acknowledgment, our battery will not send the desired 0x355 message with the charge level.

For me it helped to power my fake SN65HVD230 with 5.0V instead and connect both CTX and CRX pins to the ESP32 using a bidirectional level translator circuit. Then it was "magically" starting to acknowledge messages and everything suddently worked as desired.

### LC Display 4x20 HD44780

As I like those ASCII displys from my past, and they are easily accessible from trashed electronics, I've used one in my hardware. It's connected to the ESP32 in parallel 4-bit mode via pins RS, EN, D4..D7. The pins RW and the contrast voltage I've hardwired to Vss (Gnd). The display needs 5V. But as it only works in receive-mode (RW=0) and is able to handle the 3.3V levels correctly, I didn't use level converters.

For current pin assignment, please see "Constants"-section of the source code.

### Optical impulse receiver circuit

For receiving the infrared 1/10000kWh impulses from my power meter, I build a small circuit based on the LM393 comperator IC. Note the the IR diode is actually a TRANSMITTER diode from an old remote control. All my available reveiver diodes didn't work due to the wrong wavelength of the grey filter plastics. But in my experiments I noticed that a transmitter diode (with shiny plastics) also works nicely as receiver.

Please find the circuit below (also avalable as [KiCad Schematic](optical_circuit.kicad_sch)):

![optical sensor schematic](optical_circuit.png)

Here a photo of my implementation mounted in front of my power meter:

![optical sensor photo](esp32ess_optical_sensor.jpg)
