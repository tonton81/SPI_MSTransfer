# SPI Master Slave Transfer library for Teensy (SPI_MSTransfer Library)

Work in progress for PJRC Teensy by tonton81 developing here : https://forum.pjrc.com/threads/50008-Project-SPI_MSTransfer

Tested connected two Teensy 3.x's :: https://forum.pjrc.com/threads/48450-uNav-AHRS?p=169644&viewfull=1#post169644

New library is a remake of the original spi_controller, but with more advanced functionality.

Code is still in a design/remake state, so the constructor might be modified and unnecessary code would be cleaned up while additional features are implemented. It's currently in a baseline state that currently offers following capability:

1) The master can send arrays to the slave, and the slave can send arrays back, the lengths do not matter, and CRC from both ends are validated. There is no cross-array transfers, the protocol is very similar to uart in that an array is sent first, processing is done, and an array is sent back, simple.

2) Fire & forget is implemented by design, adding functions can use that feature immediately.

3) Master supports 3x resends for invalidated packets before exiting the function. Bad packets could be unstable SPI frequencies, bad SPI lines, or your breadboard, so protection is implemented 

4) Slave supports circular-buffer-sending. Master is constantly pulsing the clock line after the sent receipt, thus allowing the slave sufficient time for responding when it's ready. This allows either an out-of-sync or bad CRC to recapture and revalidate from the slave.

5) Tests were done between 4MHz to 40 MHz. For example, at 24MHz, a confirmed 2-way acknowledgment with crc validation for digitalwrite(digitalread) toggling is 85micros, and pinmode is 28micros. Fire and forget (with acknowledgement of a confirmed receipt), a 100 byte array from a master to slave at 24MHz SPI came to 384 uS (this was with CRC validation) 

6) Library has been tested on various master/slave configurations
* a. T3.6 - T3.6
* b. T3.6 - T3.5
* c. T3.5 - T3.5
* d. T3.6 - T3.1
* e. T3.5 - T3.2

7) Master/Slave support hot plugging.

8) Additional features are planned and to be implemented

CONNECTIONS: For the SLAVE, SPI0 is the port of the slave. For the MASTER, you can use any SPI port you want

Slave's wiring:
T_3.5 :: T_3.6 > Connections
GND :: GND > GND
14 :: 14 > SCK
12 :: 11 > MOSI
11 :: 12 > MISO
02 :: 15 > CS

Pin 2 was chosen so that later on when I add Serial2 support pin10 would be available.  Pin 14 for clock so that Led could be used

NOTE: SPI communication between teensy master&slave, MISO GOES TO MOSI, and MOSI to MISO, crossed, just like uart.


Thanks goes out to defragster for making me improve my old code.

Enjoy current beta 
