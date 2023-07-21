# ESP32 BLE HID Explorer

This program scans for BLE HID devices. When a device is found, scanning stops.
Dump out all services and characteristics. For all subscrible characteristics,
subscribe to notifications or indications. For all readable characteristics,
read and dump values, While connected to the device, print
notification/indication notification/indication data in hex. When disconnected,
resume scanning for a new HID device.

The first time a BLE HID device is used with this program it must be paired.
This usually means holding down a button while turning the device. Consult the
device docs for details. For devices with one button, entering pairing mode
can usually by turning on the device then long pressing the same button.

For example, the Xbox One controller has a button next to the USB connector.
When the controller is off, press and hold this button then press and hold the
Xbox logo button until the logo lights up. Release both buttons. This program
should find the controller and dump out information. The next time it is not
necessary to pair. Just press the Xbox logo button until it lights up.

As an extra bonus, this program can decode the Xbox One binary data into X, Y
axes values, etc.

Note this a demo program so it is slow. It fetches and prints a lot of
information which is useful for understanding how BLE works but is not needed
in real world operation. For example, fetching the Device Information is
completely optional. Same for the Report Map which can be hundreds of bytes
long.

## Software

* Tested with Arduino IDE 1.8.19 but 2.x should work.
* NimBLE-Arduino Library installed using the IDE Library Manager.

## Hardware

This program should work on any ESP32 board supported by NimBLE.
