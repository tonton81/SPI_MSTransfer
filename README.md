
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

## USAGE MASTER:

For the Master to successfully send to the slave you have to create a instance of SPI_MSTransfer defining SPI parameters:

Example (see the examples for layout)

    SPI_MSTransfer teensy_gpio = SPI_MSTransfer("Serial", SPI_MST_CS, &SPI_MST_BUS, SPI_SPEED ); // bad with default timeouts

where 
* teensy_gpio is the name to be used to access member functions of the library. This is user defined. You can call it anything. It's purpose is not just for the SPI_MSTransfer callback communication, but an object which has different attributes depending on the construction.

"teensy_gpio" has a special use case, is really assigned to "Serial", which is USBSerial of the slave.
Not only does it have events() rights, but gpio rights as well, pinmode, analog, etc, they are global between objects.

"Serial" - The user has the ability to control the ports of the slave Teenys.  In this case we defined as Serial.  But if you wanted to control to Serial5 you would use "Serial5". The could also access the slave's EEPROM by substituting EEPROM for "Serial". Future implementations will allow the user to access the slave's different I2C and SPI buses.

For instance if the you define:

```SPI_MSTransfer mySlaveSerial5 = SPI_MSTransfer("Serial5", SPI_MST_CS, &SPI_MST_BUS, SPI_SPEED );```

you would be able to:
```mySlaveSerial5.begin(115200);
mySlaveSerial5.write(0x55);
mySlaveSerial5.write(buffer,length);
mySlaveSerial5.read();
mySlaveSerial5.flush();
mySlaveSerial5.peek();
etc...
```

* SPI_MST_CS is the chip select pin number that you use to address the slave device

* SPI_MST_BUS is the SPI port that you want to use, typically this would be SPI for the default SPI bus but on the Teensy 3.5 it can be SPI1 or SPI2. Remember to add the "&" before the bus.

* SPI_SPEED is the SPI bus speed that you want to use, typically this is set to 30Mhz, 30000000.

In the sketch setup() there a couple of things that are mandatory for the master:
* teensy_gpio.onTransfer(myCallback)
where myCallback is a callback function to tell the sketch what to do when the master receives data back from the slave.  Yes you can do two way transfers.

Note: At this point you can also assign different pins for the master MOSI,MISO and Clock for the Teensy.

After Master SPI setup you will need to start the SPI bus.
* SPI_MST_BUS.begin()

Now for the loop()
The library is capabable of transfering uint8, uint16 and uint32's. To transfer uint16's. The basic command format would be:
```transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID, bool fire_and_forget = 0);```

If for instance you have a bunch of doubles that you want to send you would first need to define a buffer and put your doubles into an array:
```uint16_t *buf;                  // pointer to the buffer, buf
    double MST_Vals[12];            // your array
    buf = (uint16_t *)MST_Vals;     // set the buffer to point to the array
then do the transfer using
    teensy_gpio.transfer16((uint16_t *) MST_Vals, sizeof(MST_Vals)/2, 60, 1);
    where
    buffer -> MST_Vals
    length -> sizeof(MST_Vals)/2, have to divide by 2 since you are taking the size of doubles which are 32bits to get the size in 16bits?????
    60 -> packetID that you assign
     1 -> implies fire and forget
```
After this all you really need is to add:
```teensy_gpio.events();```
why, so the callback works if get something back from the slave.

Now, you have the option to add (real nice to know you are getting data) a command to blink the LED on the slave to let you know data was sent:
```teensy_gpio.pinToggle(LED_BUILTIN);```
If you are sending alot of data wrap it in a timer to send about every 1 second.
