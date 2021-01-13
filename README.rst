Bluetooth: Multi-NUS Central
#######################

The Multi-NUS is a simple wireless UART network demo.

This application is an extension of the Central UART sample application in NCS which is a fork of the Zephyr Project. 
This version of the central UART is able to connect to multiple peripherals, up to 20, at the same time. 
This application was built using v1.4.1 of NCS.
 
It uses the NUS Client to send data back and forth between a UART connection and a Bluetooth LE connection, emulating a serial port over Bluetooth LE.

Please see my blog post on the DevZone for a complete and thorough explanation: 
https://devzone.nordicsemi.com/nordic/nrf-connect-sdk-guides/b/software/posts/enter-the-multi-nus-a-simple-wireless-uart-network

Also consult the documentation for original Nordic UART Service. There you'll find all you need to know about which devices to use, programming, debugging, and testing. 

http://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/samples/bluetooth/central_uart/README.html

This application runs on a central device and will connect to up to 20 peripheral devices running the peripheral_uart sample from NCS.
The peripheral application doesn't need to be modified. 

There is a simple protocol for routing messages on the network. 

All routed messages start with *, followed by a two digit ID for the peripheral. Whatever data you intend to transmit will follow. 
Any device can created a routed message by using this code and the message will be transmitted by the central.
Any device can broadcast a message by using the address 99.
