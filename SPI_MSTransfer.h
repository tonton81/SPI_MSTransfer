/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Contributors:
  Tim - https://github.com/Defragster
  Mike - https://github.com/mjs513

  Designed and tested for PJRC Teensy 3.2, 3.5, and 3.6 boards.

  Forum link : https : //forum.pjrc.com/threads/50008-Project-SPI_MSTransfer

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#ifndef _SPI_MSTransfer_H_
#define _SPI_MSTransfer_H_

#include "Stream.h"
#include <SPI.h>
#include <i2c_t3.h>
//#include <FastLED.h>
#include "circular_buffer.h"
#include <EEPROM.h>

#define DATA_BUFFER_MAX 200
#define QUEUE_SLOTS 16

struct AsyncMST {
  uint16_t packetID = 0;
  uint8_t error = 0, slave = -1; // slave identification added by linarism
};

typedef void (*_slave_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_master_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_slave_handler_ptr_uint8_t)(uint8_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_master_handler_ptr_uint8_t)(uint8_t* buffer, uint16_t length, AsyncMST info);


class SPI_MSTransfer : public Stream {

  public:
    SPI_MSTransfer(const char *data, uint8_t addr, SPIClass *SPIWire, uint32_t spi_bus_speed = 8000000);
    SPI_MSTransfer(const char *data, const char *mode);

    virtual void            digitalWrite(uint8_t pin, bool state);
    virtual void            digitalWriteFast(uint8_t pin, bool state);
    virtual bool            digitalRead(uint8_t pin);
    virtual bool            digitalReadFast(uint8_t pin);
    virtual void            pinMode(uint8_t pin, uint8_t state);
    virtual void            pinToggle(uint8_t pin);
    virtual uint8_t         transfer(uint8_t *buffer, uint16_t length, uint16_t packetID, bool fire_and_forget = 0);
    virtual uint16_t        transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID, bool fire_and_forget = 0);
    virtual uint16_t        events(uint32_t MinTime = 100);
    virtual void            onTransfer(_slave_handler_ptr handler);
    virtual void            onTransfer(_slave_handler_ptr_uint8_t handler);
    virtual void            begin(); // SLAVE // WIRE
    virtual void            begin(uint32_t baudrate);
    virtual int             read(); // SERIAL // WIRE
    virtual int             available(); // SERIAL // WIRE
    virtual int             peek(); // SERIAL // WIRE
    virtual size_t          write(uint8_t val) { return write(&val, 1); }  // SERIAL // WIRE
    virtual size_t          write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }  // SERIAL // WIRE
    virtual size_t          write(const uint8_t *buf, size_t size);  // SERIAL // WIRE
    virtual void            flush();
    virtual void            setTX(uint8_t pin, bool opendrain=false);
    virtual void            setRX(uint8_t pin);
    virtual bool            attachRts(uint8_t pin);
    virtual bool            attachCts(uint8_t pin);
    virtual void            debug(Stream &serial);
    virtual void            analogReadResolution(unsigned int bits);
    virtual uint32_t        analogWriteResolution(uint32_t bits);
    virtual int             analogRead(uint8_t pin);
    virtual void            analogWrite(uint8_t pin, int val);
    virtual void            watchdog(uint32_t value);
    virtual void            _detect();
    static                  Circular_Buffer<uint16_t, QUEUE_SLOTS, DATA_BUFFER_MAX> mtsca;
    static                  Circular_Buffer<uint16_t, QUEUE_SLOTS, DATA_BUFFER_MAX> stmca;
    virtual size_t          print(const char *p);
    virtual size_t          println(const char *p);
    virtual int             read(int addr); // EEPROM
    virtual void            write(int addr, uint8_t value); // EEPROM
    virtual void            update(int addr, uint8_t data); // EEPROM
    virtual uint16_t        length(); // EEPROM
    virtual uint32_t        crc(); // EEPROM
    Stream*                 debugSerial;
    virtual void            setSCL(uint8_t pin); // WIRE
    virtual void            setSDA(uint8_t pin); // WIRE
    virtual void            beginTransmission(uint8_t addr); // WIRE
    virtual uint8_t         endTransmission(uint8_t sendStop = 1); // WIRE
    virtual void            setClock(uint32_t frequency); // WIRE
    virtual void            send(uint8_t b) { write(b); } // WIRE
    virtual void            send(uint8_t *s, uint8_t n ) { write(s, n); } // WIRE
    virtual void            send(int n) { write((uint8_t)n); } // WIRE
    virtual uint8_t         receive(void) { int c = read(); if (c < 0) return 0; return c;	} // WIRE
    virtual uint8_t         requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
    virtual uint8_t         requestFrom(uint8_t address, uint8_t quantity) { return requestFrom(address, quantity, (uint8_t)1); }
    virtual uint8_t         requestFrom(int address, int quantity, int sendStop) { return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)(sendStop ? 1 : 0)); }
    virtual uint8_t         requestFrom(int address, int quantity) { return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)1); 	}


//  below here future implementation
    virtual void      beginTransaction(uint32_t baudrate, uint8_t msblsb, uint8_t dataMode); // SPI
    virtual void      endTransaction(); // SPI 
    virtual uint8_t   transfer(uint8_t data); // SPI 8bit
    virtual uint16_t  transfer16(uint16_t data); // SPI 16bit
    //virtual void      show(uint8_t pin, CRGB *array, uint16_t array_length); // Fastled
    virtual void      software_reset();
    virtual bool      online();

  private:
    SPIClass                *spi_port;
    static                  _slave_handler_ptr _slave_handler; 
    static                  _master_handler_ptr _master_handler; 
    static                  _slave_handler_ptr_uint8_t _slave_handler_uint8_t; 
    static                  _master_handler_ptr_uint8_t _master_handler_uint8_t; 
    static bool             watchdogEnabled;
    static uint32_t         watchdogFeedInterval;
    static uint32_t         watchdogTimeout;
    virtual  bool           command_ack_response(uint16_t *data, uint32_t len);
    volatile int8_t         _serial_port_identifier = -1;
    virtual  void           SPI_assert();
    virtual  void           SPI_deassert();
    volatile bool           _slave_access = 0;
    volatile bool           _master_access = 0;
    volatile uint32_t       _spi_bus_speed;
    volatile uint8_t        chip_select = -1;
    volatile int8_t         eeprom_support = -1;
    volatile int8_t         wire_port = -1;
    volatile uint8_t        _delay_before_deassertion = 25;
    volatile uint8_t        _transfer_slowdown_while_reading = 0;
    volatile bool           _slave_data_available = 0;


 //  below here future implementation / cleanup
   virtual  uint8_t   status_update();
    volatile uint8_t   spi_support = -1;
    volatile uint8_t   fastled_support = -1;
    volatile uint8_t   servo_support = -1;
    volatile uint8_t   serial_port = -1;
    volatile uint8_t   remote_spi_port = -1;

};
#endif
