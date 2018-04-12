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

#include <SPI_MSTransfer.h>
#include "Stream.h"
#include <SPI.h>
#include "circular_buffer.h"
#include <i2c_t3.h>
#include <EEPROM.h>

SPI_MSTransfer *_slave_pointer;
_slave_handler_ptr SPI_MSTransfer::_slave_handler = nullptr;
_master_handler_ptr SPI_MSTransfer::_master_handler = nullptr;
_slave_handler_ptr_uint8_t SPI_MSTransfer::_slave_handler_uint8_t = nullptr;
_master_handler_ptr_uint8_t SPI_MSTransfer::_master_handler_uint8_t = nullptr;
bool SPI_MSTransfer::watchdogEnabled = 0;
uint32_t SPI_MSTransfer::watchdogFeedInterval = millis();
uint32_t SPI_MSTransfer::watchdogTimeout = 0;
Circular_Buffer<uint16_t, SPI_MST_QUEUE_SLOTS, SPI_MST_DATA_BUFFER_MAX> SPI_MSTransfer::mtsca;
Circular_Buffer<uint16_t, SPI_MST_QUEUE_SLOTS, SPI_MST_DATA_BUFFER_MAX> SPI_MSTransfer::stmca;


void SPI_MSTransfer::onTransfer(_slave_handler_ptr handler) {
  if ( _master_access ) _master_handler = handler;
  if ( _slave_access ) _slave_handler = handler;
}
void SPI_MSTransfer::onTransfer(_slave_handler_ptr_uint8_t handler) {
  if ( _master_access ) _master_handler_uint8_t = handler;
  if ( _slave_access ) _slave_handler_uint8_t = handler;
}
SPI_MSTransfer::SPI_MSTransfer(const char *data, uint8_t cs, SPIClass *SPIWire, uint32_t spi_bus_speed) {
  chip_select = cs; spi_port = SPIWire; _master_access = 1; _spi_bus_speed = spi_bus_speed;
  ::pinMode(cs, OUTPUT); // make sure CS is OUTPUT before deassertion.
  ::digitalWriteFast(cs, HIGH); // deassert the CS way before SPI initializes :)
  if ( spi_bus_speed >= 12000000 ) _delay_before_deassertion = 0;
  debugSerial = nullptr;
  if ( !strcmp(data, "Serial")       ) { _serial_port_identifier = 0; }
  else if ( !strcmp(data, "Serial1") ) { _serial_port_identifier = 1; }
  else if ( !strcmp(data, "Serial2") ) { _serial_port_identifier = 2; }
  else if ( !strcmp(data, "Serial3") ) { _serial_port_identifier = 3; }
  else if ( !strcmp(data, "Serial4") ) { _serial_port_identifier = 4; }
  else if ( !strcmp(data, "Serial5") ) { _serial_port_identifier = 5; }
  else if ( !strcmp(data, "Serial6") ) { _serial_port_identifier = 6; }
  else if ( !strcmp(data, "EEPROM")  ) { eeprom_support = 1;          }
  else if ( !strcmp(data, "Wire") ) { wire_port = 0; }
  else if ( !strcmp(data, "Wire1") ) { wire_port = 1; }
  else if ( !strcmp(data, "Wire2") ) { wire_port = 2; }
  else if ( !strcmp(data, "Wire3") ) { wire_port = 3; }






  else if ( !strcmp(data, "SPI1") ) { remote_spi_port = 1; }
  else if ( !strcmp(data, "SPI2") ) { remote_spi_port = 2; }
  else if ( !strcmp(data, "SERVO") ) { servo_support = 1; }
  else if ( !strcmp(data, "FASTLED" ) ) { fastled_support = 1; }
}


void SPI_MSTransfer::debug(Stream &serial) {
  debugSerial = &serial;
  Serial.print("DBG: [S_CS "); Serial.print(chip_select);
  Serial.print("] CB Capacity: "); Serial.print(mtsca.capacity());
  Serial.print(" Length: "); Serial.println(mtsca.max_size());
}
void SPI_MSTransfer::SPI_assert() {
  spi_port->beginTransaction(SPISettings(_spi_bus_speed, MSBFIRST, SPI_MODE0)); ::digitalWriteFast(chip_select, LOW);
}
void SPI_MSTransfer::SPI_deassert() { delayMicroseconds(_delay_before_deassertion); ::digitalWriteFast(chip_select, HIGH); spi_port->endTransaction(); }

bool SPI_MSTransfer::command_ack_response(uint16_t *data, uint32_t len) {
  uint8_t resend_count = 0; uint32_t timeout = micros(); uint16_t _crc = 0;
  while ( 1 ) {
    _crc = spi_port->transfer16(0xFFFF);
    if ( _crc == 0xF00D ) {
      spi_port->transfer16(0xBEEF); break;
    }
    else if ( _crc == 0xBAAD || micros() - timeout > 1000 ) {
      resend_count++;
      if ( resend_count > 3 ) {
        if ( debugSerial != nullptr ) {
          Serial.print("DBG: [S_CS "); Serial.print(chip_select);
          Serial.print("] FAIL_RES #"); Serial.print(resend_count);
          Serial.println(" Tx ABORT. "); delay(1000);
        }
        SPI_deassert(); return 0;
      }
      if ( debugSerial != nullptr ) {
        Serial.print("FAIL_Res #"); Serial.print(resend_count);
        Serial.println(" RETRY..."); delay(1000);
      }
      spi_port->transfer16(0xBEEF);
      SPI_deassert();
      SPI_assert();
      for ( uint16_t i = 0; i < len; i++ ) { spi_port->transfer16(data[i]); }
      timeout = micros();
    }
  }
  return 1;
}

void SPI_MSTransfer::digitalWriteFast(uint8_t pin, bool state) { digitalWrite(pin, state); }
void SPI_MSTransfer::digitalWrite(uint8_t pin, bool state) {
  if ( _slave_access ) return;
  uint16_t data[5], checksum = 0, data_pos = 0;
  data[data_pos] = 0x9766; checksum ^= data[data_pos]; data_pos++; // HEADER
  data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
  data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
  data[data_pos] = ((uint16_t)(pin << 8) | state); checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = checksum;
  SPI_assert();
  for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
  if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
  for ( uint16_t i = 0; i < 3000; i++ ) {
    if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
      uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
      for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
      if ( checksum == buf[buf[1] - 1] ) {
        spi_port->transfer16(0xD0D0); // send confirmation
        SPI_deassert(); return;
      }
    }
  }
  SPI_deassert(); return;
}
bool SPI_MSTransfer::digitalReadFast(uint8_t pin) { return digitalRead(pin); }
bool SPI_MSTransfer::digitalRead(uint8_t pin) {
  if ( _slave_access ) return 0;
  uint16_t data[5], checksum = 0, data_pos = 0;
  data[data_pos] = 0x9766; checksum ^= data[data_pos]; data_pos++; // HEADER
  data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
  data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
  data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = checksum;
  SPI_assert();
  for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
  if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
  for ( uint16_t i = 0; i < 3000; i++ ) {
    if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
      uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
      for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
      if ( checksum == buf[buf[1] - 1] ) {
        spi_port->transfer16(0xD0D0); // send confirmation
        SPI_deassert(); return buf[2];
      }
    }
  }
  SPI_deassert(); return 0;
}
void SPI_MSTransfer::pinMode(uint8_t pin, uint8_t state) {
  if ( _slave_access ) return;
  uint16_t data[5], checksum = 0, data_pos = 0;
  data[data_pos] = 0x9766; checksum ^= data[data_pos]; data_pos++; // HEADER
  data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
  data[data_pos] = 0x0002; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
  data[data_pos] = ((uint16_t)(pin << 8) | state); checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = checksum;
  SPI_assert();
  for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
  if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
  for ( uint16_t i = 0; i < 3000; i++ ) {
    if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
      uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
      for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
      if ( checksum == buf[buf[1] - 1] ) {
        spi_port->transfer16(0xD0D0); // send confirmation
        SPI_deassert(); return;
      }
    }
  }
  SPI_deassert(); return;
}
void SPI_MSTransfer::pinToggle(uint8_t pin) { // code written by defragster
  if ( _slave_access ) return;
  uint16_t data[5], checksum = 0, data_pos = 0;
  data[data_pos] = 0x9766; checksum ^= data[data_pos]; data_pos++; // HEADER
  data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
  data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
  data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = checksum;
  SPI_assert();
  for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
  if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
  for ( uint16_t i = 0; i < 3000; i++ ) {
    if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
      uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
      for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
      if ( checksum == buf[buf[1] - 1] ) {
        spi_port->transfer16(0xD0D0); // send confirmation
        SPI_deassert(); return;
      }
    }
  }
  SPI_deassert(); return;
}
uint8_t SPI_MSTransfer::transfer(uint8_t *buffer, uint16_t length, uint16_t packetID, uint8_t fire_and_forget) {
  if ( _slave_access ) {
    uint16_t len = ( !(length % 2) ) ? ( length / 2 ) : ( length / 2 ) + 1;
    uint16_t data[6 + len], checksum = 0, data_pos = 0;
    data[data_pos] = 0xAA55; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = packetID; checksum ^= data[data_pos]; data_pos++;
    bool odd_or_even = ( (length % 2) );
    for ( uint16_t i = 0; i < length; i += 2 ) {
      if ( odd_or_even ) {
        if ( i + 1 < length ) {
          data[data_pos] = ((uint16_t)(buffer[i] << 8) | buffer[i+1]); checksum ^= data[data_pos]; data_pos++;
        }
        else {
          data[data_pos] = buffer[i]; checksum ^= data[data_pos]; data_pos++;
        }
      }
      else {
          data[data_pos] = ((uint16_t)(buffer[i] << 8) | buffer[i+1]); checksum ^= data[data_pos]; data_pos++;
      }
    }
    data[data_pos] = checksum;
    stmca.push_back(data,data[1]);
    return packetID;
  }
  if ( _master_access ) {
    uint16_t len = ( !(length % 2) ) ? ( length / 2 ) : ( length / 2 ) + 1;
    uint16_t data[6 + len], checksum = 0, data_pos = 0;
    data[data_pos] = ( fire_and_forget ) ? 0x9254 : 0x9253; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = packetID; checksum ^= data[data_pos]; data_pos++;
    bool odd_or_even = ( (length % 2) );
    for ( uint16_t i = 0; i < length; i += 2 ) {
      if ( odd_or_even ) {
        if ( i + 1 < length ) {
          data[data_pos] = ((uint16_t)(buffer[i] << 8) | buffer[i+1]); checksum ^= data[data_pos]; data_pos++;
        }
        else {
          data[data_pos] = buffer[i]; checksum ^= data[data_pos]; data_pos++;
        }
      }
      else {
          data[data_pos] = ((uint16_t)(buffer[i] << 8) | buffer[i+1]); checksum ^= data[data_pos]; data_pos++;
      }
    }
    data[data_pos] = checksum;
    if ( fire_and_forget == 1 ) {
      SPI_assert();
      for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
      uint32_t timeout = millis();
      while ( spi_port->transfer16(0xFFFF) != 0xBABE && millis() - timeout < 100 );
      spi_port->transfer16(0xD0D0); // send confirmation
      SPI_deassert();
      return data[4]; // F&F, RETURN PACKETID
    }
    else if ( fire_and_forget == 2 ) {
      mtsca.push_back(data, data[1]);
      return 0;
    }
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert();
  }
  return 0;
}
uint16_t SPI_MSTransfer::transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID, uint8_t fire_and_forget) {
  if ( _slave_access ) {
    uint16_t data[6 + length], checksum = 0, data_pos = 0;
    data[data_pos] = 0xAA55; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = packetID; checksum ^= data[data_pos]; data_pos++;
    for ( uint16_t i = 0; i < length; i++ ) { data[data_pos] = buffer[i]; checksum ^= data[data_pos]; data_pos++; }
    data[data_pos] = checksum;
    stmca.push_back(data,data[1]);
    return packetID;
  }
  if ( _master_access ) {
    uint16_t data[6 + length], checksum = 0, data_pos = 0;
    data[data_pos] = ( fire_and_forget ) ? 0x9244 : 0x9243; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = packetID; checksum ^= data[data_pos]; data_pos++;
    for ( uint16_t i = 0; i < length; i++ ) { data[data_pos] = buffer[i]; checksum ^= data[data_pos]; data_pos++; }
    data[data_pos] = checksum;
    if ( fire_and_forget == 1 ) {
      SPI_assert();
      for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
      uint32_t timeout = millis();
      while ( spi_port->transfer16(0xFFFF) != 0xBABE && millis() - timeout < 100 );
      spi_port->transfer16(0xD0D0); // send confirmation
      SPI_deassert();
      return data[4]; // F&F, RETURN PACKETID
    }
    else if ( fire_and_forget == 2 ) {
      mtsca.push_back(data, data[1]);
      return 0;
    }
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert();
  }
  return 0;
}
void SPI_MSTransfer::begin(uint32_t baudrate) { // set remote serial port and baud
  if ( _slave_access ) return;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[7], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = baudrate >> 16; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = baudrate; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
int SPI_MSTransfer::read(void) {
  if ( _slave_access ) return -1;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int16_t)buf[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
  if ( wire_port != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0008; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int16_t)buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
int SPI_MSTransfer::peek() {
  if ( _slave_access ) return -1;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int16_t)buf[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
  if ( wire_port != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0009; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int16_t)buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
int SPI_MSTransfer::available(void) {
  if ( _slave_access ) return 0;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0002; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
  if ( wire_port != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0007; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
size_t SPI_MSTransfer::write(const uint8_t *buf, size_t size) {
  if ( _slave_access ) return 0;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[6 + size], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0004; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = size; checksum ^= data[data_pos]; data_pos++;
    for ( uint16_t i = 0; i < size; i++ ) { data[data_pos] = buf[i]; checksum ^= data[data_pos]; data_pos++; }
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
  if ( wire_port != -1 ) {
    uint16_t data[6 + size], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0005; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = size; checksum ^= data[data_pos]; data_pos++;
    for ( uint16_t i = 0; i < size; i++ ) { data[data_pos] = buf[i]; checksum ^= data[data_pos]; data_pos++; }
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
void SPI_MSTransfer::flush() {
  if ( _slave_access ) return;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0005; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
size_t SPI_MSTransfer::print(const char *p) {
  if ( _slave_access ) return -1;
  write(p, strlen(p));
}
size_t SPI_MSTransfer::println(const char *p) {
  if ( _slave_access ) return -1;
  char _text[strlen(p) + 1]; for ( uint16_t i = 0; i < strlen(p); i++ ) _text[i] = p[i]; _text[sizeof(_text) - 1] = '\n'; write(_text, sizeof(_text));
}
void SPI_MSTransfer::begin() {
  if ( _slave_access ) {

    // Better to initialize the ports outside the ISR, and before slave is activated.

    //////// Pre-Set UART busses
    Serial.begin(115200); // usb serial
    NVIC_SET_PRIORITY(IRQ_USBOTG, 0);
    Serial1.begin(115200); // 0+1
    Serial2.begin(115200); // 9+10
    Serial3.begin(115200); // 7+8
    NVIC_SET_PRIORITY(IRQ_UART0_STATUS ,0);
    NVIC_SET_PRIORITY(IRQ_UART1_STATUS ,0);
    NVIC_SET_PRIORITY(IRQ_UART2_STATUS ,0);
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    Serial4.begin(115200); // 31+32
    Serial5.begin(115200); // 33+34
    Serial6.begin(115200); // 47+48
    NVIC_SET_PRIORITY(IRQ_UART3_STATUS ,0);
    NVIC_SET_PRIORITY(IRQ_UART4_STATUS ,0);
#endif
#if defined(__MK64FX512__)
    NVIC_SET_PRIORITY(IRQ_UART5_STATUS, 0);
#endif
#if defined(__MK66FX1M0__)
    NVIC_SET_PRIORITY(IRQ_LPUART0 ,0);
#endif

    //////// Pre-Set Wire busses
    Wire.begin(); // pins 18+19
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    Wire1.begin(); // pins 37+38
    Wire2.begin(); // pins 3+4
#endif
#if defined(__MK66FX1M0__)
    Wire3.begin(); // pins 56 + 57
#endif

    //////// Pre-Set SPI busses
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    SPI1.setSCK(20); SPI1.setMOSI(21); SPI1.setMISO(5); SPI1.begin();
    SPI2.setSCK(46); SPI2.setMOSI(44); SPI2.setMISO(45); SPI2.begin();
    SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // default speed
    SPI2.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // default speed
#endif



    NVIC_SET_PRIORITY(IRQ_I2C0, 0);
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    NVIC_SET_PRIORITY(IRQ_I2C1, 0);
    NVIC_SET_PRIORITY(IRQ_I2C2, 0);
#endif
#if defined(__MK66FX1M0__)
    NVIC_SET_PRIORITY(IRQ_I2C3, 0);
#endif

    SIM_SCGC6 |= SIM_SCGC6_SPI0; // enable slave clock
    SPI0_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS; // stop
    SPI0_CTAR0_SLAVE = SPI_CTAR_FMSZ(15) & SPI0_CTAR0_SLAVE & (~(SPI_CTAR_CPOL | SPI_CTAR_CPHA) | 0x00 << 25);
    SPI0_RSER = 0x00020000;
    CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN2_CONFIG =  PORT_PCR_PS | PORT_PCR_MUX(2); // this uses pin 2 for the CS so Serial2 can be used instead.
    SPI0_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS; // start
    NVIC_SET_PRIORITY(IRQ_SPI0, 1); // set priority
    NVIC_ENABLE_IRQ(IRQ_SPI0); // enable CS IRQ

    pinMode(13,OUTPUT); // Enable LED use
  }
  if ( wire_port != -1 ) {
    uint16_t data[4], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0004; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
int SPI_MSTransfer::read(int addr) {
  if ( _slave_access ) return -1;
  if ( eeprom_support != -1 ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x67BB; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = addr; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int16_t)buf[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
}
void SPI_MSTransfer::write(int addr, uint8_t value) {
  if ( _slave_access ) return;
  if ( eeprom_support != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x67BB; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = addr; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = value; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert();
        }
      }
    }
    SPI_deassert(); return;
  }
}
void SPI_MSTransfer::update(int addr, uint8_t value) {
  if ( _slave_access ) return;
  if ( eeprom_support != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x67BB; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0002; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = addr; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = value; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert();
        }
      }
    }
    SPI_deassert(); return;
  }
}
uint16_t SPI_MSTransfer::length() {
  if ( _slave_access ) return -1;
  if ( eeprom_support != -1 ) {
    uint16_t data[4], checksum = 0, data_pos = 0;
    data[data_pos] = 0x67BB; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
}
uint32_t SPI_MSTransfer::crc() {
  if ( _slave_access ) return -1;
  if ( eeprom_support != -1 ) {
    uint16_t data[4], checksum = 0, data_pos = 0;
    data[data_pos] = 0x67BB; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0004; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return ((uint32_t)buf[2] << 16 | buf[3]);
        }
      }
    }
    SPI_deassert(); return -1;
  }
}
void SPI_MSTransfer::setTX(uint8_t pin, bool opendrain) {
  if ( _slave_access ) return;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0006; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = ((uint16_t)(pin << 8) | opendrain); checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
void SPI_MSTransfer::setRX(uint8_t pin) {
  if ( _slave_access ) return;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0007; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
bool SPI_MSTransfer::attachRts(uint8_t pin) {
  if ( _slave_access ) return 0;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0008; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
bool SPI_MSTransfer::attachCts(uint8_t pin) {
  if ( _slave_access ) return 0;
  if ( _serial_port_identifier != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0009; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
void SPI_MSTransfer::analogReadResolution(unsigned int bits) {
  if ( _slave_access ) return;
  if ( _master_access ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x7429; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = bits; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
int SPI_MSTransfer::analogRead(uint8_t pin) {
  if ( _slave_access ) return 0;
  if ( _master_access ) {
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x7429; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) spi_port->transfer16(data[i]);
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return (int)buf[2];
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
uint32_t SPI_MSTransfer::analogWriteResolution(uint32_t bits) {
  if ( _slave_access ) return 0;
  if ( _master_access ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x7429; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0002; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = bits >> 16; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = bits; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return ((uint32_t)buf[2] << 16 | buf[3]);
        }
      }
    }
    SPI_deassert(); return 0;
  }
}
void SPI_MSTransfer::analogWrite(uint8_t pin, int val) {
  if ( _slave_access ) return;
  if ( _master_access ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x7429; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = val; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
void SPI_MSTransfer::beginTransmission(uint8_t addr) {
  if ( _slave_access ) return;
  if ( wire_port != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = addr; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          delayMicroseconds(300);
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
uint8_t SPI_MSTransfer::endTransmission(uint8_t sendStop) {
  if ( _slave_access ) return -1;
  if ( wire_port != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0001; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = sendStop; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          delay(1); // crashes when lower
          SPI_deassert(); return (uint8_t)buf[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
}
void SPI_MSTransfer::setSDA(uint8_t pin) {
  if ( _slave_access ) return;
  if ( wire_port != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0002; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
void SPI_MSTransfer::setSCL(uint8_t pin) {
  if ( _slave_access ) return;
  if ( wire_port != -1 ) {
    uint16_t data[6], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = pin; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
void SPI_MSTransfer::setClock(uint32_t frequency) {
  if ( _slave_access ) return;
  if ( wire_port != -1 ) {
    uint16_t data[7], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0006; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = frequency >> 16; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = frequency; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
uint8_t SPI_MSTransfer::requestFrom(uint8_t address, uint8_t length, uint8_t sendStop) {
  if ( _slave_access ) return -1;
  if ( wire_port != -1 ) {
    uint16_t data[8], checksum = 0, data_pos = 0;
    data[data_pos] = 0x66AA; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0010; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = wire_port; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = address; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = sendStop; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    SPI_assert();
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return -1; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          spi_port->transfer16(0xD0D0); // send confirmation
          SPI_deassert(); return data[2];
        }
      }
    }
    SPI_deassert(); return -1;
  }
}






























uint8_t SPI_MSTransfer::transfer(uint8_t data) {
}
uint16_t SPI_MSTransfer::transfer16(uint16_t data) {
}
bool SPI_MSTransfer::online() {
}
void SPI_MSTransfer::software_reset() {
}
uint8_t SPI_MSTransfer::status_update() {
}
void SPI_MSTransfer::beginTransaction(uint32_t baudrate, uint8_t msblsb, uint8_t dataMode) { // set remote serial port and baud
}
void SPI_MSTransfer::endTransaction() { // end remote SPI transaction
}
//void SPI_MSTransfer::show(uint8_t pin, CRGB *array, uint16_t array_length) { // update Fastled pixels
//}












































































SPI_MSTransfer::SPI_MSTransfer(const char *data, const char *mode) {
  if ( !strcmp(data, "SLAVE") && !strcmp(mode, "STANDALONE") ) {
    debugSerial = nullptr;
    ::pinMode(13, OUTPUT);  // detach pin13 from SPI0 for led use.
    _slave_pointer = this; _slave_access = 1;
  }
}

void spi0_isr(void) {
  static uint16_t data[SPI_MST_DATA_BUFFER_MAX];
  uint16_t buffer_pos = 0, len = 0, process_crc = 0;
  while ( !(GPIOD_PDIR & 0x01) ) {
    if ( SPI0_SR & 0xF0 ) {
      SPI0_PUSHR_SLAVE = 0xDEAF; data[buffer_pos] = SPI0_POPR; buffer_pos++;
      if ( buffer_pos >= SPI_MST_DATA_BUFFER_MAX ) break; // BUFFER LENGTH PROTECTION
    }
    if ( data[1] && buffer_pos >= data[1] ) break;
  }

  if ( data[1] && buffer_pos >= data[1] ) {
    len = data[1];
    static bool _swap = 1;


    /* F&F BLOCK START */
    if ( data[0] == 0x9244 || data[0] == 0x9254 ) {
      _slave_pointer->SPI_MSTransfer::mtsca.push_back(data, len);
      _slave_pointer->SPI_MSTransfer::_slave_processing_selftimer = millis();
      while ( !(GPIOD_PDIR & 0x01) ) { // wait here until MASTER confirms F&F receipt
        if ( SPI0_SR & 0xF0 ) {
          SPI0_PUSHR_SLAVE = 0xBABE;
          if ( SPI0_POPR == 0xD0D0 ) {
            SPI0_SR |= SPI_SR_RFDF; return;
          }
        }
      }
      SPI0_SR |= SPI_SR_RFDF; return;
    }
    /* F&F BLOCK END */



    /* QUEUE RUN BLOCK START */
    if ( data[0] == 0x99BB ) {
      _slave_pointer->SPI_MSTransfer::_slave_processing_busy = 1;
      _slave_pointer->SPI_MSTransfer::_slave_processing_selftimer = millis();
      while ( !(GPIOD_PDIR & 0x01) ) { // wait here until MASTER confirms F&F receipt
        if ( SPI0_SR & 0xF0 ) {
          SPI0_PUSHR_SLAVE = 0xBABE;
          if ( SPI0_POPR == 0xD0D0 ) {
            SPI0_SR |= SPI_SR_RFDF; return;
          }
        }
      }
      SPI0_SR |= SPI_SR_RFDF; return;
    }
    /* QUEUE RUN BLOCK END */



    /* STATUS BLOCK START */
    if ( data[0] == 0x98BB && data[2] == 0x98B8 ) {
      _swap = 1;
      while ( !(GPIOD_PDIR & 0x01) ) { // wait here until MASTER confirms STATUS receipt
        if ( SPI0_SR & 0xF0 ) {
          if ( _swap ) {
            _swap = 0;
            uint16_t _notify_status = 0xAD00;
            if ( _slave_pointer->SPI_MSTransfer::stmca.size() ) _notify_status |= 1 << 0;
            if ( _slave_pointer->SPI_MSTransfer::mtsca.size() ) _notify_status |= 1 << 1;
            SPI0_PUSHR_SLAVE = _notify_status;
            SPI0_POPR;
          }
          else {
            _swap = 0;
            SPI0_PUSHR_SLAVE = 0xBABE;
            if ( SPI0_POPR == 0xD0D0 ) {
              SPI0_SR |= SPI_SR_RFDF; return;
            }
          }
        }
      }
      SPI0_SR |= SPI_SR_RFDF; return;
    }
    /* STATUS BLOCK END */


    /* BEGIN PROCESSING */

    for ( uint16_t i = 0; i < len - 1; i++ ) process_crc ^= data[i];
    while ( 1 ) { // wait here until MASTER confirms ACK receipt
      if ( !(GPIOD_PDIR & 0x01) ) {
        if ( SPI0_SR & 0xF0 ) {
          if ( data[len - 1] != process_crc ) {
            SPI0_PUSHR_SLAVE = 0xBAAD; SPI0_POPR;
          }
          else {
            SPI0_PUSHR_SLAVE = 0xF00D;
            if ( SPI0_POPR == 0xBEEF ) break;
          }
        }
      }
      else {
        SPI0_SR |= SPI_SR_RFDF; return;
      }
    }


    if ( data[len - 1] != process_crc ) {
      if ( _slave_pointer->SPI_MSTransfer::debugSerial != nullptr ) {
        Serial.print("DEBUG: [CRC FAIL ISR] [DATA] ");
        for ( uint16_t i = 0; i < len; i++ ) {
          Serial.print(data[i], HEX); Serial.print(" ");
        } Serial.println(); Serial.flush();
      }
      SPI0_SR |= SPI_SR_RFDF; return; // CRC FAILED, DON'T PROCESS!
    }


    if ( _slave_pointer->SPI_MSTransfer::debugSerial != nullptr ) {
      Serial.print("DEBUG: [DATA] ");
      for ( uint16_t i = 0; i < len; i++ ) {
        Serial.print(data[i], HEX); Serial.print(" ");
      } Serial.println(); Serial.flush();
    }


    switch ( data[0] ) {
      case 0x9766: {
          switch ( data[2] ) {
            case 0x0000: { // SLAVE DIGITALWRITE(FAST) CONTROL
                ::digitalWriteFast(data[3] >> 8, data[3]);
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0001: { // SLAVE DIGITALREAD(FAST) CONTROL
                uint16_t checksum = 0, buf_pos = 0, buf[4] = { 0xAA55, 4, 0, 0 };
                if ( ::digitalReadFast(data[3]) ) {
                  buf[2] = 1; buf[3] = 0xAA50;
                }
                else {
                  buf[2] = 0; buf[3] = 0xAA51;
                }
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0002: { // SLAVE PINMODE CONTROL
                ::pinMode(data[3] >> 8, data[3]);
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0003: { // SLAVE PIN TOGGLE, code written by defragster
                uint16_t tPin = data[3];
                if ( LED_BUILTIN == tPin ) GPIOC_PTOR = 32;
                else digitalWrite(tPin, !digitalRead(tPin) );
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        }
      case 0x9243: { // MASTER SENDS PACKET TO SLAVE QUEUE WITH CRC ACKNOWLEDGEMENT (UINT16_T)
          switch ( data[2] ) {
            case 0x0000: {
                _slave_pointer->SPI_MSTransfer::mtsca.push_back(data, len);
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, data[4], checksum ^= data[4] };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        }
      case 0x9253: { // MASTER SENDS PACKET TO SLAVE QUEUE WITH CRC ACKNOWLEDGEMENT (UINT8_T)
          switch ( data[2] ) {
            case 0x0000: {
                _slave_pointer->SPI_MSTransfer::mtsca.push_back(data, len);
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, data[4], checksum ^= data[4] };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        }
      case 0x9712: {
          switch ( data[2] ) {
            case 0x0000: {
                if ( _slave_pointer->SPI_MSTransfer::stmca.size() > 0 ) { // IF QUEUES EXIST, ONE WILL BE DEQUEUED TO MASTER
                  uint16_t checksum = 0, buf_pos = 0;
                  while ( !(GPIOD_PDIR & 0x01) ) {
                    if ( SPI0_SR & 0xF0 ) {
                      SPI0_PUSHR_SLAVE = _slave_pointer->SPI_MSTransfer::stmca.front()[ ( buf_pos > _slave_pointer->SPI_MSTransfer::stmca.front()[1] ) ? buf_pos = 0 : buf_pos++];
                      if ( SPI0_POPR == 0xD0D0 ) {
                        _slave_pointer->SPI_MSTransfer::stmca.pop_front();
                        break;
                      }
                    }
                  }
                  SPI0_SR |= SPI_SR_RFDF; return;
                }
                else { // OTHERWISE, SEND MINIMAL PACKET FOR NO QUEUES
                  uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 4, 0, 0xAA51 };
                  while ( !(GPIOD_PDIR & 0x01) ) {
                    if ( SPI0_SR & 0xF0 ) {
                      SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                      if ( SPI0_POPR == 0xD0D0 ) break;
                    }
                  }
                  SPI0_SR |= SPI_SR_RFDF; return;
                }
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        }
      case 0x3235: { // SERIAL PORT BEGIN METHOD
          switch ( data[2] ) {
            case 0x0000: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Serial.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
                  case 0x0001: {
                      Serial1.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
                  case 0x0002: {
                      Serial2.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
                  case 0x0003: {
                      Serial3.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      Serial4.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
                  case 0x0005: {
                      Serial5.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
                  case 0x0006: {
                      Serial6.begin((uint32_t)data[4] << 16 | data[5]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0001: {
                int16_t _read = 0;
                switch ( data[3] ) {
                  case 0x0000: {
                      _read = Serial.read(); break;
                    }
                  case 0x0001: {
                      _read = Serial1.read(); break;
                    }
                  case 0x0002: {
                      _read = Serial2.read(); break;
                    }
                  case 0x0003: {
                      _read = Serial3.read(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      _read = Serial4.read(); break;
                    }
                  case 0x0005: {
                      _read = Serial5.read(); break;
                    }
                  case 0x0006: {
                      _read = Serial6.read(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, _read, checksum ^= _read };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0002: {
                int16_t _available = 0;
                switch ( data[3] ) {
                  case 0x0000: {
                      _available = Serial.available(); break;
                    }
                  case 0x0001: {
                      _available = Serial1.available(); break;
                    }
                  case 0x0002: {
                      _available = Serial2.available(); break;
                    }
                  case 0x0003: {
                      _available = Serial3.available(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      _available = Serial4.available(); break;
                    }
                  case 0x0005: {
                      _available = Serial5.available(); break;
                    }
                  case 0x0006: {
                      _available = Serial6.available(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, _available, checksum ^= _available };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0003: {
                int16_t _peek = 0;
                switch ( data[3] ) {
                  case 0x0000: {
                      _peek = Serial.peek(); break;
                    }
                  case 0x0001: {
                      _peek = Serial1.peek(); break;
                    }
                  case 0x0002: {
                      _peek = Serial2.peek(); break;
                    }
                  case 0x0003: {
                      _peek = Serial3.peek(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      _peek = Serial4.peek(); break;
                    }
                  case 0x0005: {
                      _peek = Serial5.peek(); break;
                    }
                  case 0x0006: {
                      _peek = Serial6.peek(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, _peek, checksum ^= _peek };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0004: {
                uint16_t _written = 0; uint8_t _buf[data[4]];
                for ( uint16_t i = 0; i < data[4]; i++ ) _buf[i] = data[i + 5];
                switch ( data[3] ) {
                  case 0x0000: {
                      _written = Serial.write(_buf, data[4]); break;
                    }
                  case 0x0001: {
                      _written = Serial1.write(_buf, data[4]); break;
                    }
                  case 0x0002: {
                      _written = Serial2.write(_buf, data[4]); break;
                    }
                  case 0x0003: {
                      _written = Serial3.write(_buf, data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      _written = Serial4.write(_buf, data[4]); break;
                    }
                  case 0x0005: {
                      _written = Serial5.write(_buf, data[4]); break;
                    }
                  case 0x0006: {
                      _written = Serial6.write(_buf, data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, _written, checksum ^= _written };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0005: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Serial.flush(); break;
                    }
                  case 0x0001: {
                      Serial1.flush(); break;
                    }
                  case 0x0002: {
                      Serial2.flush(); break;
                    }
                  case 0x0003: {
                      Serial3.flush(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      Serial4.flush(); break;
                    }
                  case 0x0005: {
                      Serial5.flush(); break;
                    }
                  case 0x0006: {
                      Serial6.flush(); break;
                    }
#endif
                }
                uint16_t buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0006: {
                switch ( data[3] ) {
                  case 0x0001: {
                      Serial1.setTX(data[4] >> 8, data[4]); break;
                    }
                  case 0x0002: {
                      Serial2.setTX(data[4] >> 8, data[4]); break;
                    }
                  case 0x0003: {
                      Serial3.setTX(data[4] >> 8, data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      Serial4.setTX(data[4] >> 8, data[4]); break;
                    }
                  case 0x0005: {
                      Serial5.setTX(data[4] >> 8, data[4]); break;
                    }
                  case 0x0006: {
                      Serial6.setTX(data[4] >> 8, data[4]); break;
                    }
#endif
                }
                uint16_t buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0007: {
                switch ( data[3] ) {
                  case 0x0001: {
                      Serial1.setRX(data[4]); break;
                    }
                  case 0x0002: {
                      Serial2.setRX(data[4]); break;
                    }
                  case 0x0003: {
                      Serial3.setRX(data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      Serial4.setRX(data[4]); break;
                    }
                  case 0x0005: {
                      Serial5.setRX(data[4]); break;
                    }
                  case 0x0006: {
                      Serial6.setRX(data[4]); break;
                    }
#endif
                }
                uint16_t buf_pos = 0, buf[] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0008: {
                bool value = 0;
                switch ( data[3] ) {
                  case 0x0001: {
                      value = Serial1.attachRts(data[4]); break;
                    }
                  case 0x0002: {
                      value = Serial2.attachRts(data[4]); break;
                    }
                  case 0x0003: {
                      value = Serial3.attachRts(data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      value = Serial4.attachRts(data[4]); break;
                    }
                  case 0x0005: {
                      value = Serial5.attachRts(data[4]); break;
                    }
                  case 0x0006: {
                      value = Serial6.attachRts(data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 4, value, checksum };
                for ( uint16_t i = 0; i < buf[1] - 1; i++ ) checksum ^= buf[i];
                buf[buf[1] - 1] = checksum;
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0009: {
                bool value = 0;
                switch ( data[3] ) {
                  case 0x0001: {
                      value = Serial1.attachCts(data[4]); break;
                    }
                  case 0x0002: {
                      value = Serial2.attachCts(data[4]); break;
                    }
                  case 0x0003: {
                      value = Serial3.attachCts(data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0004: {
                      value = Serial4.attachCts(data[4]); break;
                    }
                  case 0x0005: {
                      value = Serial5.attachCts(data[4]); break;
                    }
                  case 0x0006: {
                      value = Serial6.attachCts(data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 4, value, checksum };
                for ( uint16_t i = 0; i < buf[1] - 1; i++ ) checksum ^= buf[i];
                buf[buf[1] - 1] = checksum;
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }





          }
          SPI0_SR |= SPI_SR_RFDF; return;
        } // END OF UART SECTION






      case 0x67BB: { // EEPROM SECTION
          switch ( data[2] ) {
            case 0x0000: {
                uint16_t val = EEPROM.read(data[3]);
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, val, checksum ^= val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0001: {
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) {
                      EEPROM.write(data[3], data[4]);
                      break;
                    }
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0002: {
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) {
                      EEPROM.update(data[3], data[4]);
                      break;
                    }
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0003: {
                uint16_t val = EEPROM.length();
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, val, checksum ^= val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0004: { // CRC code Written by Christopher Andrews.
                const unsigned long crc_table[16] = {
                  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
                  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
                  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
                  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
                };
                unsigned long crc = ~0L;
                for (int index = 0 ; index < EEPROM.length()  ; ++index) {
                  crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
                  crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
                  crc = ~crc;
                }
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 5, crc >> 16, crc, checksum };
                for ( uint16_t i = 0; i < buf[1] - 1; i++ ) checksum ^= buf[i];
                buf[buf[1] - 1] = checksum;
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        } // END OF EEPROM SECTION


      case 0x7429: { // ANALOG SECTION
          switch ( data[2] ) {
            case 0x0000: {
                analogReadResolution(data[3]);
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0001: {
                uint16_t val = analogRead(data[3]);
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, val, checksum ^= val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0002: {
                uint32_t val = analogWriteResolution((uint32_t)data[3] << 16 | data[4]);
                uint16_t checksum = 0, buf_pos = 0, buf[] = { 0xAA55, 5, val >> 16, val, checksum };
                for ( uint16_t i = 0; i < buf[1] - 1; i++ ) checksum ^= buf[i];
                buf[buf[1] - 1] = checksum;
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0003: {
                analogWrite(data[3],data[4]);
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
          }
          SPI0_SR |= SPI_SR_RFDF; return;
        } // END OF ANALOG SECTION


      case 0x66AA: { // WIRE SECTION
          switch ( data[2] ) {
            case 0x0000: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Wire.beginTransmission((uint8_t)data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      Wire1.beginTransmission((uint8_t)data[4]); break;
                    }
                  case 0x0002: {
                      Wire2.beginTransmission((uint8_t)data[4]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      Wire3.beginTransmission((uint8_t)data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0001: {
                uint8_t val;
                switch ( data[3] ) {
                  case 0x0000: {
                      val = Wire.endTransmission((uint8_t)data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      val = Wire1.endTransmission((uint8_t)data[4]); break;
                    }
                  case 0x0002: {
                      val = Wire2.endTransmission((uint8_t)data[4]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      val = Wire3.endTransmission((uint8_t)data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, (uint16_t)val, checksum ^= (uint16_t)val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0002: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Wire.setSDA((uint8_t)data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      Wire1.setSDA((uint8_t)data[4]); break;
                    }
                  case 0x0002: {
                      Wire2.setSDA((uint8_t)data[4]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      Wire3.setSDA((uint8_t)data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0003: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Wire.setSCL((uint8_t)data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      Wire1.setSCL((uint8_t)data[4]); break;
                    }
                  case 0x0002: {
                      Wire2.setSCL((uint8_t)data[4]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      Wire3.setSCL((uint8_t)data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0004: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Wire.begin(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      Wire1.begin(); break;
                    }
                  case 0x0002: {
                      Wire2.begin(); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      Wire3.begin(); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0005: {
                uint16_t _written = 0; uint8_t _buf[data[4]];
                for ( uint16_t i = 0; i < data[4]; i++ ) _buf[i] = data[i + 5];
                switch ( data[3] ) {
                  case 0x0000: {
                      _written = Wire.write(_buf, (uint8_t)data[4]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      _written = Wire1.write(_buf, (uint8_t)data[4]); break;
                    }
                  case 0x0002: {
                      _written = Wire2.write(_buf, (uint8_t)data[4]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      _written = Wire3.write(_buf, (uint8_t)data[4]); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, _written, checksum ^= _written };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0006: {
                switch ( data[3] ) {
                  case 0x0000: {
                      Wire.setClock(((uint32_t)data[4] << 16 | data[5])); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      Wire1.setClock(((uint32_t)data[4] << 16 | data[5])); break;
                    }
                  case 0x0002: {
                      Wire2.setClock(((uint32_t)data[4] << 16 | data[5])); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      Wire3.setClock(((uint32_t)data[4] << 16 | data[5])); break;
                    }
#endif
                }
                uint16_t checksum = 0, buf_pos = 0, buf[3] = { 0xAA55, 3, 0xAA56 };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0007: {
                int16_t val;
                switch ( data[3] ) {
                  case 0x0000: {
                      val = Wire.available(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      val = Wire1.available(); break;
                    }
                  case 0x0002: {
                      val = Wire2.available(); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      val = Wire3.available(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, (uint16_t)val, checksum ^= (uint16_t)val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0008: {
                int16_t val;
                switch ( data[3] ) {
                  case 0x0000: {
                      val = Wire.read(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      val = Wire1.read(); break;
                    }
                  case 0x0002: {
                      val = Wire2.read(); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      val = Wire3.read(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, (uint16_t)val, checksum ^= (uint16_t)val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0009: {
                int16_t val;
                switch ( data[3] ) {
                  case 0x0000: {
                      val = Wire.peek(); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      val = Wire1.peek(); break;
                    }
                  case 0x0002: {
                      val = Wire2.peek(); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      val = Wire3.peek(); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, (uint16_t)val, checksum ^= (uint16_t)val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }
            case 0x0010: {
                uint8_t val;
                switch ( data[3] ) {
                  case 0x0000: {
                      val = Wire.requestFrom(data[4],data[5],data[6]); break;
                    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
                  case 0x0001: {
                      val = Wire1.requestFrom(data[4],data[5],data[6]); break;
                    }
                  case 0x0002: {
                      val = Wire2.requestFrom(data[4],data[5],data[6]); break;
                    }
#endif
#if defined(__MK66FX1M0__)
                  case 0x0003: {
                      val = Wire3.requestFrom(data[4],data[5],data[6]); break;
                    }
#endif
                }
                uint16_t checksum = 0xAA51, buf_pos = 0, buf[] = { 0xAA55, 4, (uint16_t)val, checksum ^= (uint16_t)val };
                while ( !(GPIOD_PDIR & 0x01) ) {
                  if ( SPI0_SR & 0xF0 ) {
                    SPI0_PUSHR_SLAVE = buf[ ( buf_pos > buf[1] ) ? buf_pos = 0 : buf_pos++];
                    if ( SPI0_POPR == 0xD0D0 ) break;
                  }
                }
                SPI0_SR |= SPI_SR_RFDF; return;
              }








          }
          SPI0_SR |= SPI_SR_RFDF; return;
        } // END OF WIRE SECTION













    }




  }
  SPI0_SR |= SPI_SR_RFDF; return;
}








































void SPI_MSTransfer::_detect() {
  if ( _slave_access ) return;
  if ( _serial_port_identifier != -1 ) {
    SPI_assert();
    uint16_t data[5], checksum = 0, data_pos = 0;
    data[data_pos] = 0x3235; checksum ^= data[data_pos]; data_pos++; // HEADER
    data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
    data[data_pos] = 0x0003; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
    data[data_pos] = _serial_port_identifier; checksum ^= data[data_pos]; data_pos++;
    data[data_pos] = checksum;
    for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
    if ( !command_ack_response(data, sizeof(data) / 2) ) return; // RECEIPT ACK
    for ( uint16_t i = 0; i < 3000; i++ ) {
      if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
        uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
        for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
        if ( checksum == buf[buf[1] - 1] ) {
          SPI_deassert(); return;
        }
      }
    }
    SPI_deassert(); return;
  }
}
uint16_t SPI_MSTransfer::events(uint32_t MinTime) {
  if ( _master_access ) {

    static uint32_t LastTime = 0;
    if ( micros() - LastTime < MinTime ) return 0;
    LastTime = micros();


    /* CHECK STATUS FLAGS */

      uint16_t data[3], checksum = 0, data_pos = 0;
      data[data_pos] = 0x98BB; checksum ^= data[data_pos]; data_pos++; // HEADER
      data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
      data[data_pos] = checksum;
      SPI_assert();
      for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
      uint32_t timeout = millis();
      uint16_t value = 0;
      while ( ( value = spi_port->transfer16(0xFFFF) ) != 0xBABE && millis() - timeout < 100 ) {
        if ( value >> 8 == 0xAD ) {
          ( value >> 0 & 0x01 ) ? _slave_data_available = 1 : _slave_data_available = 0;
          ( value >> 1 & 0x01 ) ? _slave_processing_busy = 1 : _slave_processing_busy = 0;
        }
      }
      spi_port->transfer16(0xD0D0); // send confirmation
      SPI_deassert();


    /* TRANSFER SYSTEM QUEUES */

  if ( !_slave_processing_busy ) {
    while ( mtsca.size() ) {
      if ( debugSerial != nullptr ) {
        if ( mtsca.size() == mtsca.capacity() ) {
          Serial.print("DBG: [S_CS "); Serial.print(chip_select);
          Serial.println("] F&F MAX QUEUE REACHED");
        }
        else {
          Serial.print("DBG: [S_CS "); Serial.print(chip_select);
          Serial.print("] ");
          Serial.print(mtsca.size());
          Serial.print("/");
          Serial.print(mtsca.capacity());
          Serial.println(" DEQUEUE(S) LEFT");
        }
      }
      SPI_assert();
      uint16_t buffer[mtsca.peek_front()[1]];
      mtsca.pop_front(buffer,sizeof(buffer)/2);
      for ( uint16_t i = 0; i < buffer[1]; i++ ) { spi_port->transfer16(buffer[i]); }
      uint32_t timeout = millis();
      while ( spi_port->transfer16(0xFFFF) != 0xBABE && millis() - timeout < 100 );
      spi_port->transfer16(0xD0D0); // send confirmation
      SPI_deassert();
    }
    { // SEND SIGNAL TO START PROCESSING
      uint16_t data[3], checksum = 0, data_pos = 0;
      data[data_pos] = 0x99BB; checksum ^= data[data_pos]; data_pos++; // HEADER
      data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
      data[data_pos] = checksum;
      SPI_assert();
      for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
      uint32_t timeout = millis();
      while ( spi_port->transfer16(0xFFFF) != 0xBABE && millis() - timeout < 100 );
      spi_port->transfer16(0xD0D0); // send confirmation
      SPI_deassert();
    }
  }




    /* GET CALLBACKS FROM SLAVE */
    if ( _slave_data_available ) {
      uint16_t data[4], checksum = 0, data_pos = 0;
      data[data_pos] = 0x9712; checksum ^= data[data_pos]; data_pos++; // HEADER
      data[data_pos] = sizeof(data) / 2; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
      data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
      data[data_pos] = checksum;
      SPI_assert();
      for ( uint16_t i = 0; i < data[1]; i++ ) { spi_port->transfer16(data[i]); }
      if ( !command_ack_response(data, sizeof(data) / 2) ) return 0; // RECEIPT ACK
      for ( uint16_t i = 0; i < 3000; i++ ) {
        if ( spi_port->transfer16(0xFFFF) == 0xAA55 ) {
          uint16_t buf[spi_port->transfer16(0xFFFF)]; buf[0] = 0xAA55; buf[1] = sizeof(buf) / 2; checksum = buf[0]; checksum ^= buf[1];
          for ( uint16_t i = 2; i < buf[1]; i++ ) { delayMicroseconds(_transfer_slowdown_while_reading); buf[i] = spi_port->transfer16(0xFFFF); if ( i < buf[1] - 1 ) checksum ^= buf[i]; }
          if ( checksum == buf[buf[1] - 1] ) {
            if ( buf[1] > 4 ) {
              if ( buf[2] == 0x0001 ) {
                bool odd_or_even = ( (buf[3] % 2) );
                uint8_t _slave_array[buf[3]];
                for ( uint16_t i = 0, j = 0; i < buf[3]/2; i++ ) {
                  _slave_array[j] = buf[5+i] >> 8;
                  _slave_array[j+1] = (uint8_t)buf[5+i];
                  j+=2;
                }
                if ( odd_or_even ) _slave_array[sizeof(_slave_array)-1] = buf[buf[1]-2];
                AsyncMST info; info.packetID = buf[4]; info.slave = chip_select;
                _master_handler_uint8_t(_slave_array, buf[3], info);
                spi_port->transfer16(0xD0D0); // send confirmation
                SPI_deassert(); return 0x00A6;
              }
              else if ( buf[2] == 0x0000 ) {
                uint16_t _slave_array[buf[3]];
                memmove (&_slave_array[0], &buf[5], buf[3] * 2 );
                AsyncMST info; info.packetID = buf[4]; info.slave = chip_select;
                _master_handler(_slave_array, buf[3], info);
                spi_port->transfer16(0xD0D0); // send confirmation
                SPI_deassert(); return 0x00A6;
              }
            }
            spi_port->transfer16(0xD0D0); // send confirmation
            SPI_deassert(); return 0;
          }
        }
      }
      SPI_deassert();
    }
  }



  if ( _slave_access ) {
    if ( watchdogEnabled && millis() - watchdogFeedInterval > ( watchdogTimeout / 4 ) ) {
      watchdogFeedInterval = millis();
      __disable_irq(); WDOG_REFRESH = 0xA602; WDOG_REFRESH = 0xB480; __enable_irq();
    }
    if ( !mtsca.size() ) _slave_processing_busy = 0;
    if ( !_slave_processing_busy && mtsca.size() && _slave_processing_selftimer > 500 ) _slave_processing_busy = 1;
    if ( _slave_processing_busy && mtsca.size() ) {
      uint16_t array[mtsca.front()[1]];
      mtsca.pop_front(array,sizeof(array)/2 );
      if ( array[0] == 0x9254 || array[0] == 0x9253 ) {
        uint16_t checksum = 0, buf_pos = 0; AsyncMST info; info.packetID = array[4];
        for ( uint16_t i = 0; i < array[1] - 1; i++ ) checksum ^= array[i];
        ( checksum == array[array[1]-1] ) ? info.error = 0 : info.error = 1;
        bool odd_or_even = ( (array[3] % 2) );
        uint8_t buf[array[3]];
        for ( uint16_t i = 0, j = 0; i < array[3]/2; i++ ) {
          buf[j] = array[5+i] >> 8;
          buf[j+1] = (uint8_t)array[5+i];
          j+=2;
        }
        if ( odd_or_even ) buf[sizeof(buf)-1] = array[array[1]-2];
        if ( _slave_handler_uint8_t != nullptr ) _slave_handler_uint8_t(buf, array[3], info);
      }
      else if ( array[0] == 0x9244 || array[0] == 0x9243 ) {
        uint16_t checksum = 0, buf_pos = 0, buf[array[3]]; AsyncMST info; info.packetID = array[4];
        for ( uint16_t i = 0; i < array[1] - 1; i++ ) checksum ^= array[i];
        ( checksum == array[array[1]-1] ) ? info.error = 0 : info.error = 1;
        memmove (&buf[0], &array[5], array[3] * 2 );
        if ( _slave_handler != nullptr ) _slave_handler(buf, array[3], info);
      }
    }
  }
  return 0;
}











namespace std {
void __attribute__((weak)) __throw_bad_alloc() {
  Serial.println("Unable to allocate memory");
}
void __attribute__((weak)) __throw_length_error( char const*e ) {
  Serial.print("Length Error :"); Serial.println(e);
}
}




























void SPI_MSTransfer::watchdog(uint32_t value) {
  if ( !_slave_access ) return;
  watchdogEnabled = 1;
  __disable_irq();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  __asm__ volatile ("nop");
#if (F_CPU / F_BUS) > 1
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 2
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 3
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 4
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 5
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 6
  __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 7
  __asm__ volatile ("nop");
#endif
  WDOG_STCTRLH = WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_ALLOWUPDATE;
  WDOG_TOVALH = value >> 16;
  WDOG_TOVALL = value;
  WDOG_PRESC = 0;
  __enable_irq();
  for (int i = 0; i < 256; i++) {
    __asm__ volatile ("nop");
#if (F_CPU / F_BUS) > 1
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 2
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 3
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 4
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 5
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 6
    __asm__ volatile ("nop");
#endif
#if (F_CPU / F_BUS) > 7
    __asm__ volatile ("nop");
#endif
  }
}
