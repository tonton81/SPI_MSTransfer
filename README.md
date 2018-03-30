
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

## SLAVE USAGE

Make sure you include the SPI_MSTransfer library
```#include <SPI_MSTransfer.h>```

Again, define an instance for the library, in this case we just call it "slave"
```SPI_MSTransfer slave = SPI_MSTransfer("SLAVE", "STANDALONE");```
Note for the slave all we are doing is telling the library that this is a slave device "SLAVE" and it is "STANDALONE".

In the setup section of the sketch, you will need to tell give the slave a begin and ontransfer of data use the callback function "myCallback" which is where all the fun stuff happens:
```
slave.begin( );
slave.onTransfer(myCallback);
```
Another command you can use is the debug command.  Use this sparingly:

```slave.debug(Serial); // SPI_MST Debug error tracking```

In the sketch's loop you also have the ability to send data back to the master from the slave. This code snippet demonstrates how you can do that.  The only command you need to send the data back is the onTransfer command which is the same as in the Master's usage.
```
  static uint32_t _timer = millis();
  //This section is optional and demonstrates how to send data from
  //slave back to the master
  if ( millis() - _timer > 5 ) {
    uint16_t *buf;
    double MST_PrintVals[12];
    buf = (uint16_t *)MST_PrintVals;
    int ii = 0;
    static uint16_t __count = 0;
    for ( uint32_t i = 0; i < sizeof(MST_PrintVals) / sizeof( MST_PrintVals[0]  ); i++ ) MST_PrintVals[i] = __count++;
    _timer = millis();
    slave.transfer16((uint16_t *)MST_PrintVals, sizeof(MST_PrintVals) / 2, 55, 1);
  }
```
At the end of the loop you need to add the event command again in the same way as you did the master:
```	slave.events();```

Now for the critical piece for the slave, the myCallback function.

Lets dissect some code to demonstrate how this works.

The callback function takes 3 parameters: 
- a pointer to buffer to hold the packet data when it is received. In this case it is defined as a uint16_t even though we are transferring doubles. 
- The length of the packet and 
- the info on the packet, "AsyncMST info".  Available in info are the packetID, error, and slave ID.
```
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
```
While not used in this example is that you can have multiple callbacks depending on the size of the variables being transfered, e.g.,
```void myCallback1(uint8_t *buffer, uint16_t length, AsyncMST info)```
or 
```void myCallback2(uint16_t *buffer, uint16_t length, AsyncMST info)```
to transfer floats as opposed to doubles in our example.

If we defined a packetID number in the MASTER as "55" we can test what to do with that packet by comparing it to the packetID received using info.packetID:

```	if ( 55 == info.packetID ) {```

Once you get the right packet you should test its length to make sure we have a complete packet.  Since we are using doubles in this example that equates to 4 bytes x 12 variables that we sent from the MASTER which equates to 48 for the lenght. If the wrong length the send message to serial monitor letting us know.  Same thing applies if there is an error which we test for using info.error.

```
		if ( 48 != length ) {
			Serial.print("Bad Length: "); Serial.println(length); // If this shows then the SPI Passed array size is nor wrong
		}
		else if ( 0 != info.error ) {
			Serial.println("\nBad CRC: ");
		}
```

If the packet passes out tests then you need to tell the sketch what to do with the packet.  In this case with are assigning the buffer to the variable MST_PrintVals and parsing back to string to send to serial port (USB since we are using Serial):

```
		else {
			double* MST_PrintVals;
			MST_PrintVals = (double *)buffer;
			{
				//trig conversions
				float rad2deg = 180.0f / PI;
				float deg2rad = PI / 180.0f;
				int  textLength = 12 * 31;
				char text[textLength];

				char utcText[30];
				char tsIMUText[30];
				char tsGPSText[30];

				// KF parameters
				char latText[30];
				char lonText[30];
				char altText[30];
				char pk1xText[30];
				char pk1yText[30];
				char pk1zText[30];
				char nuk1xText[30];
				char nuk1yText[30];
				char nuk1zText[30];

				int ii = 0;
				dtostrf( MST_PrintVals[ii] , 10, 6, utcText);
				ii++;
				dtostrf( MST_PrintVals[ii] * 0.000001f, 10, 4, tsIMUText);
				ii++;
				dtostrf( MST_PrintVals[ii] * 0.000001f, 10, 4, tsGPSText);
				ii++;
				dtostrf( MST_PrintVals[ii] *rad2deg, 10, 6, latText);
				ii++;
				dtostrf( MST_PrintVals[ii] *rad2deg, 10, 6, lonText);
				ii++;
				dtostrf( MST_PrintVals[ii] , 10, 4, altText);
				ii++;
				dtostrf(sqrt( MST_PrintVals[ii] ), 10, 4, pk1xText);
				ii++;
				dtostrf(sqrt( MST_PrintVals[ii] ), 10, 4, pk1yText);
				ii++;
				dtostrf(sqrt( MST_PrintVals[ii] ), 10, 4, pk1zText);
				ii++;
				dtostrf( MST_PrintVals[ii] , 10, 4, nuk1xText);
				ii++;
				dtostrf( MST_PrintVals[ii] , 10, 4, nuk1yText);
				ii++;
				dtostrf( MST_PrintVals[ii] , 10, 4, nuk1zText);

				// Create single text parameter and print it
				snprintf(text, textLength, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
				         utcText, tsIMUText, tsGPSText,
				         latText, lonText, altText,
				         pk1xText, pk1yText, pk1zText,
				         nuk1xText, nuk1yText, nuk1zText);
				Serial.println(text);
				//Serial.send_now();
			}
		}
```

If the the packetID is not covered we again send a message with the packet info that we received.

```
	}
	else {
        Serial.print("Packet Received with ");
		Serial.print("PacketID: ");
		Serial.println(info.packetID);
	}
}
```
