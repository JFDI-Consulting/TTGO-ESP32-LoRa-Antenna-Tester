# TTGO-ESP32-LoRa-Antenna-Tester

This firmware used RadioHead's RH_RF95 driver with RHEncryptedDriver and RHReliableDatagram to establish a connection between two ESP32 boards for the purposes of LoRa antenna testing. Yes, you should model your antenna and test candidate designs with a VNA, but your perfect antenna will perform differently in the real world, surrounded by circuit boards, casings and buildings. So there's no substitute for real world testing.

Min, max and mean are calculated for RSSI, SNR and frequency error. The firmware starts in receive mode. A press on the user button will send 10 test messages, time how long it takes to send and receive the ACK, and calculate RSSI etc on the received ACK messages. You can press the user button again within the 5 second timeout to send another set of 10 test messages and have their stats combined with those of the previous set. The stats can be zeroed by holding the user button down for 1.5 seconds. Alternatively you can just reset the device.

After sending a set of test messages, the device returns to receive mode, ready for the test to be performed in the opposite direction if required for symmetry testing purposes.

It's advisable to change only one small element at a time in between test runs. Keep the location and orientation of the two devices fixed and constant, changing only the antenna or its orientation.

We've written this for the TTGO/Heltec ESP32 LoRa OLED board v1 which uses the 433MHz band through the Semtech SX1278 chipset.

Inspired by, but otherwise unrelated to, Andreas Spiess's antenna tester firmware.