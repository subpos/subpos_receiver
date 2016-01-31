# SubPos Receiver

C++/Arduino libraries that wait for a scan result from an ESP Wi-Fi module and trilaterates a position from this scan.
Outputs NMEA position data via USB UART.

Note this just does a raw position calc from each scan and doesn't time out nodes or perform averaging (yet).

Tested on Arduino and the Teensy 3.2 and Energia and the Launchpad Tiva C (tm4c123). The sketch uses 127,316 bytes.

For use with the ESP Wi-Fi Scanner Firmware - https://github.com/subpos/esp_wifi_scanner

Connect ESP module to UART1 on Teensy module. 

Trilateration based on : https://github.com/Wayne82/Trilateration

This also requires the following libraries (add both the library zip files in the Arduino/Energia IDE):

Eigen C++ - http://eigen.tuxfamily.org/index.php?title=Main_Page

StandardCplusplus - https://github.com/maniacbug/StandardCplusplus

