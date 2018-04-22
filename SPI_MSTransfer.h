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

#ifndef SPI_MST_DATA_BUFFER_MAX
#define SPI_MST_DATA_BUFFER_MAX 250
#endif

#ifndef SPI_MST_QUEUE_SLOTS
#define SPI_MST_QUEUE_SLOTS 8
#endif

#include "Stream.h"
#include <SPI.h>
#include <i2c_t3.h>
#include "circular_buffer.h"
#include <EEPROM.h>
#include <functional>
#include <atomic>

struct AsyncMST {
  uint16_t packetID = 0;
  uint8_t error = 0, slave = -1, port = 0; // slave identification added by linarism
};

typedef void (*_slave_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_master_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_slave_handler_ptr_uint8_t)(uint8_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_master_handler_ptr_uint8_t)(uint8_t* buffer, uint16_t length, AsyncMST info);

typedef void (*_wire_onReceive)(size_t count, AsyncMST info); // wire onReceive

typedef std::function<void(AsyncMST info)> _detectPtr;

class SPI_MSTransfer : public Stream {

  public:
    SPI_MSTransfer(const char *data, uint8_t addr, SPIClass *SPIWire, uint32_t spi_bus_speed = 8000000);
    SPI_MSTransfer(const char *data, const char *mode);

    void            digitalWrite(uint8_t pin, bool state);
    void            digitalWriteFast(uint8_t pin, bool state);
    bool            digitalRead(uint8_t pin);
    bool            digitalReadFast(uint8_t pin);
    void            pinMode(uint8_t pin, uint8_t state);
    void            pinToggle(uint8_t pin);
    uint8_t         transfer(uint8_t *buffer, uint16_t length, uint16_t packetID, uint8_t fire_and_forget = 0);
    uint16_t        transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID, uint8_t fire_and_forget = 0);
    uint16_t        events(uint32_t MinTime = 500);
    void            onTransfer(_slave_handler_ptr handler);
    void            onTransfer(_slave_handler_ptr_uint8_t handler);
    void            begin(); // SLAVE // WIRE
    void            begin(uint32_t baudrate); // SERIAL // WIRE
    int             read(); // SERIAL // WIRE
    size_t          read(uint8_t* buffer, size_t size); // SERIAL // WIRE
    int             available(); // SERIAL // WIRE
    int             peek(); // SERIAL // WIRE
    size_t          write(uint8_t val) { return write(&val, 1); }  // SERIAL // WIRE
    size_t          write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }  // SERIAL // WIRE
    size_t          write(const uint8_t *buf, size_t size);  // SERIAL // WIRE
    void            flush();
    void            setTX(uint8_t pin, bool opendrain=false);
    void            setRX(uint8_t pin);
    bool            attachRts(uint8_t pin);
    bool            attachCts(uint8_t pin);
    void            debug(Stream &serial);
    void            analogReadResolution(unsigned int bits);
    uint32_t        analogWriteResolution(uint32_t bits);
    int             analogRead(uint8_t pin);
    void            analogWrite(uint8_t pin, int val);
    void            watchdog(uint32_t value);
    static          Circular_Buffer<uint16_t, SPI_MST_QUEUE_SLOTS, SPI_MST_DATA_BUFFER_MAX> mtsca;
    static          Circular_Buffer<uint16_t, SPI_MST_QUEUE_SLOTS, SPI_MST_DATA_BUFFER_MAX> stmca;
    size_t          print(const char *p);
    size_t          println(const char *p);
    int             read(int addr); // EEPROM
    void            write(int addr, uint8_t value); // EEPROM
    void            update(int addr, uint8_t data); // EEPROM
    uint16_t        length(); // EEPROM
    uint32_t        crc(); // EEPROM
    Stream*         debugSerial = nullptr;
    void            setSCL(uint8_t pin); // WIRE
    void            setSDA(uint8_t pin); // WIRE
    void            beginTransmission(uint8_t addr); // WIRE
    uint8_t         endTransmission(uint8_t sendStop = 1); // WIRE
    void            setClock(uint32_t frequency); // WIRE
    void            send(uint8_t b) { write(b); } // WIRE
    void            send(uint8_t *s, uint8_t n ) { write(s, n); } // WIRE
    void            send(int n) { write((uint8_t)n); } // WIRE
    uint8_t         receive(void) { int c = read(); if (c < 0) return 0; return c;	} // WIRE
    uint8_t         requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop); // WIRE
    uint8_t         requestFrom(uint8_t address, uint8_t quantity) { return requestFrom(address, quantity, (uint8_t)1); } // WIRE
    uint8_t         requestFrom(int address, int quantity, int sendStop) { return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)(sendStop ? 1 : 0)); } // WIRE
    uint8_t         requestFrom(int address, int quantity) { return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)1); 	} // WIRE
    bool           _slave_processing_busy = 0;
    uint32_t        _slave_processing_selftimer = 0;
    uint8_t         transfer(uint8_t spi_data); // SPI
    uint16_t        transfer16(uint16_t spi_data); // SPI
    uint32_t        transfer32(uint32_t spi_data); // SPI
    void            beginTransaction(uint32_t baudrate, uint8_t msblsb, uint8_t dataMode); // SPI
    void            autoCS(int8_t pin, bool asserted = 0); // SPI
    void            endTransaction(); // SPI 
    size_t          transfer16(const uint16_t *buf, size_t size); // SPI
    size_t          transfer(const uint8_t *buf, size_t size); // SPI
    uint8_t         setCS(uint8_t pin); // SPI
    void            setMOSI(uint8_t pin); // SPI
    void            setMISO(uint8_t pin); // SPI
    void            setSCK(uint8_t pin); // SPI
    void            begin_(i2c_mode mode, uint8_t address1, uint8_t address2, uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup, uint32_t rate, i2c_op_mode opMode); // I2C
    void            begin(i2c_mode mode, uint8_t address1, i2c_pins pins=I2C_PINS_DEFAULT, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) { begin_(mode, address1, 0, pins, pins, pullup, rate, opMode); } // I2C
    void            begin(i2c_mode mode, uint8_t address1, uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) { begin_(mode, address1, 0, pinSCL, pinSDA, pullup, rate, opMode); } // I2C
    void            begin(i2c_mode mode, uint8_t address1, uint8_t address2, i2c_pins pins=I2C_PINS_DEFAULT, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) { begin_(mode, address1, address2, pins, pins, pullup, rate, opMode); } // I2C
    void            begin(i2c_mode mode, uint8_t address1, uint8_t address2, uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) { begin_(mode, address1, address2, pinSCL, pinSDA, pullup, rate, opMode); } // I2C
    void            setRate_(uint32_t busFreq, uint32_t i2cFreq); // I2C
    void            setRate(uint32_t busFreq, uint32_t i2cFreq) { setRate_(busFreq, i2cFreq); } // I2C
    uint8_t         setOpMode(i2c_op_mode opMode); // I2C
    uint32_t        getClock(); // I2C
    void            resetBus(); // I2C
    void            onReceive(_wire_onReceive handler); // I2C
    static  void    receiveEvent0(size_t count); // I2C
    static  void    receiveEvent1(size_t count); // I2C
    static  void    receiveEvent2(size_t count); // I2C
    static  void    receiveEvent3(size_t count); // I2C
    void            sendTransmission(i2c_stop sendStop = I2C_STOP); // I2C
    void            sendRequest(uint8_t addr, size_t len, i2c_stop sendStop = I2C_STOP); // I2C
    void            setDefaultTimeout(uint32_t timeout); // I2C
    uint8_t         getSCL(); // I2C
    uint8_t         getSDA(); // I2C
    uint8_t         done(); // I2C
    uint8_t         finish(uint32_t timeout=0); // I2C
    void            onDetect(_detectPtr handler); 
    bool            _slave_was_reset;
    static std::atomic<bool> _slave_queue_active;
    static std::atomic<bool> _slave_dequeue_active;


//  below here future implementation
    //virtual void      show(uint8_t pin, CRGB *array, uint16_t array_length); // Fastled

  private:
    SPIClass                *spi_port;
    static                  _slave_handler_ptr _slave_handler; 
    static                  _master_handler_ptr _master_handler; 
    static                  _slave_handler_ptr_uint8_t _slave_handler_uint8_t; 
    static                  _master_handler_ptr_uint8_t _master_handler_uint8_t; 
    static bool             watchdogEnabled;
    static uint32_t         watchdogFeedInterval;
    static uint32_t         watchdogTimeout;
    bool           command_ack_response(uint16_t *data, uint32_t len);
    volatile int8_t         _serial_port_identifier = -1;
    void                    SPI_assert();
    void                    SPI_deassert();
    bool                    _slave_access = 0;
    bool                    _master_access = 0;
    uint32_t                _spi_bus_speed = 4000000;
    uint8_t                 chip_select = -1;
    volatile int8_t         eeprom_support = -1;
    volatile int8_t         wire_port = -1;
    uint8_t                 _delay_before_deassertion = 25;
    uint8_t                 _transfer_slowdown_while_reading = 0;
    static std::atomic<bool> _slave_data_available;
    volatile bool           _run_queue_flag = 0;
    volatile int8_t         remote_spi_port = -1;
    volatile int8_t         remote_spi_pin = -1;
    volatile bool           remote_spi_pin_assert = 0;
    static                  _wire_onReceive _wire_onReceivefunc;
    volatile int8_t         fastled_support = -1;
    void                    _onDetect();
    static                  _detectPtr _slaveDetectHandler;
    uint8_t                 graceful_detect = 0;
    static volatile bool    _wire_callback_active;
    static volatile uint8_t _wire_callback_readpos;

 //  below here future implementation / cleanup
    volatile uint8_t   servo_support = -1;

};

#endif
