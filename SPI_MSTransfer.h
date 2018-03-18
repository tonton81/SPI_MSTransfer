#ifndef _SPI_MSTransfer_H_
#define _SPI_MSTransfer_H_

#include "Stream.h"
#include <SPI.h>
//#include <i2c_t3.h>
//#include <FastLED.h>
#include "circular_buffer.h"


#define DATA_BUFFER_MAX 1024
#define _transfer_slowdown 0

struct AsyncMST {
  uint16_t packetID = 0;
  uint8_t error = 0;
};

typedef void (*_slave_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef void (*_master_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);


class SPI_MSTransfer : public Stream {

  public:
    SPI_MSTransfer(const char *data, uint8_t addr, SPIClass *SPIWire, uint32_t spi_bus_speed = 4000000);
    SPI_MSTransfer(const char *data, const char *mode);

    virtual void            digitalWrite(uint8_t pin, bool state);
    virtual void            digitalWriteFast(uint8_t pin, bool state);
    virtual bool            digitalRead(uint8_t pin);
    virtual bool            digitalReadFast(uint8_t pin);
    virtual void            pinMode(uint8_t pin, uint8_t state);
    virtual void            pinToggle(uint8_t pin);
    virtual uint16_t        transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID, bool fire_and_forget = 0);
    virtual uint16_t        events();
    virtual void            onTransfer(_slave_handler_ptr handler);
    virtual void            begin(uint32_t baudrate);
    virtual int             read();
    virtual int             available();
    virtual int             peek();
    virtual size_t          write(uint8_t val) { return write(&val, 1); } 
    virtual size_t          write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }
    virtual size_t          write(const uint8_t *buf, size_t size);
    virtual void            flush();
    virtual void            debug(Stream &serial);
    virtual void            watchdog(uint32_t value);
    virtual void            begin();
    virtual void            _detect();


//  below here future implementation
    virtual void      beginTransaction(uint32_t baudrate, uint8_t msblsb, uint8_t dataMode); // SPI
    virtual void      endTransaction(); // SPI 
    virtual void      beginTransmission(uint8_t addr); // I2C
    virtual void      endTransmission(); // I2C
    virtual void      requestFrom(uint8_t address, uint8_t bytes); // I2C
    virtual uint8_t   transfer(uint8_t data); // SPI 8bit
    virtual uint16_t  transfer16(uint16_t data); // SPI 16bit
    virtual uint16_t  analogRead(uint8_t pin); // 16bit
    virtual void      analogReadResolution(uint8_t value);
    virtual void      analogWrite(uint8_t pin, uint16_t value); // 16bit
    virtual void      analogWriteResolution(uint8_t value);
    virtual int       read(int addr); // EEPROM
    virtual size_t    write(int addr, uint8_t data); // EEPROM/PWMSERVO
    virtual void      update(int addr, uint8_t data); // EEPROM
    virtual uint16_t  length(); // EEPROM
    virtual uint32_t  crc(); // EEPROM
    //virtual void      show(uint8_t pin, CRGB *array, uint16_t array_length); // Fastled
    virtual size_t    print(const char *p);
    virtual size_t    println(const char *p);
    virtual void      software_reset();
    virtual bool      online();
    Stream*                 debugSerial;
    static            Circular_Buffer<uint16_t, 64, 250> mtsca;
    static            Circular_Buffer<uint16_t, 64, 250> stmca;

  private:
    SPIClass                *spi_port;
    static                  _slave_handler_ptr _slave_handler; 
    static                  _master_handler_ptr _master_handler; 
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




 //  below here future implementation / cleanup
   virtual  uint8_t   status_update();
    volatile uint8_t   wire_port = -1;
    volatile uint8_t   eeprom_support = -1;
    volatile uint8_t   spi_support = -1;
    volatile uint8_t   fastled_support = -1;
    volatile uint8_t   servo_support = -1;
    volatile uint8_t   serial_port = -1;
    volatile uint8_t   remote_spi_port = -1;

};
#endif
