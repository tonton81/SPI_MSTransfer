/* BaroSensor Arduino library, for Freetronics BARO module (MS5637-02BA03)
 * http://www.freetronics.com/baro
 *
 * Copyright (C)2014 Freetronics Pty Ltd. Licensed under GNU GPLv3 as described in the LICENSE file.
 *
 * Written by Angus Gratton (angus at freetronics dot com)
 */
#include "BaroSensor.h"

/* i2c address of module */
#define BARO_ADDR 0x76

/* delay to wait for sampling to complete, on each OSR level */
const uint8_t SamplingDelayMs[6] PROGMEM = {
  2,
  4,
  6,
  10,
  18,
  34
};

/* module commands */
#define CMD_RESET 0x1E
#define CMD_PROM_READ(offs) (0xA0+(offs<<1)) /* Offset 0-7 */
#define CMD_START_D1(oversample_level) (0x40 + 2*(int)oversample_level)
#define CMD_START_D2(oversample_level) (0x50 + 2*(int)oversample_level)
#define CMD_READ_ADC 0x00

//****BaroSensorClass BaroSensor;

void BaroSensorClass::begin(SPI_MSTransfer &bus)
{
  _i2c = &bus;
  _i2c -> begin();
  _i2c -> beginTransmission(BARO_ADDR);
  _i2c -> write(CMD_RESET);
  err = _i2c -> endTransmission();
  Serial.print("Begin err: "); Serial.println(err);
  if(err) return;
  //Taken from Onehorse's MS5637 implementation  
    uint8_t data[2] = {0,0};
    uint8_t err;
    uint16_t destination[7];
    for (uint8_t ii = 0; ii < 7; ii++) {
      _i2c -> beginTransmission(BARO_ADDR);  // Initialize the Tx buffer
      _i2c -> write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
      err = _i2c -> endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
      //Serial.println(err);
      uint8_t i = 0;
      _i2c -> requestFrom(BARO_ADDR, 2);   // Read two bytes from slave PROM address 
      while (_i2c -> available()) {
        data[i++] = _i2c -> read(); }               // Put read results in the Rx buffer
      destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
    }
      c1 = destination[1];
      c2 = destination[2];
      c3 = destination[3];
      c4 = destination[4];
      c5 = destination[5];
      c6 = destination[6];
  initialised = true;
}

float BaroSensorClass::getTemperature(TempUnit scale, BaroOversampleLevel level)
{
  float result;
  if(getTempAndPressure(&result, NULL, scale, level))
    return result;
  else
    return NAN;
}

float BaroSensorClass::getPressure(BaroOversampleLevel level)
{
  float result;
  if(getTempAndPressure(NULL, &result, CELSIUS, level))
    return result;
  else
    return NAN;
}

bool BaroSensorClass::getTempAndPressure(float *temperature, float *pressure, TempUnit tempScale, BaroOversampleLevel level)
{
  if(err || !initialised)
    return false;

  int32_t d2 = takeReading(CMD_START_D2(level), level);
  if(d2 == 0)
    return false;
  int64_t dt = d2 - c5 * (1L<<8);

  int32_t temp = 2000 + (dt * c6) / (1L<<23);

  /* Second order temperature compensation */
  int64_t t2;
  if(temp >= 2000) {
    /* High temperature */
    t2 = 5 * (dt * dt) / (1LL<<38);
  } else {
      /* Low temperature */
    t2 = 3 * (dt * dt) / (1LL<<33);
  }

  if(temperature != NULL) {
    *temperature = (float)(temp - t2) / 100;
    if(tempScale == FAHRENHEIT)
      *temperature = *temperature * 9 / 5 + 32;
  }

  if(pressure != NULL) {
    int32_t d1 = takeReading(CMD_START_D1(level), level);
    if(d1 == 0)
      return false;

    int64_t off = c2 * (1LL<<17) + (c4 * dt) / (1LL<<6);
    int64_t sens = c1 * (1LL<<16) + (c3 * dt) / (1LL<<7);

    /* Second order temperature compensation for pressure */
    if(temp < 2000) {
      /* Low temperature */
      int32_t tx = temp-2000;
      tx *= tx;
      int32_t off2 = 61 * tx / (1<<4);
      int32_t sens2 = 29 * tx / (1<<4);
      if(temp < -1500) {
        /* Very low temperature */
        tx = temp+1500;
        tx *= tx;
        off2 += 17 * tx;
        sens2 += 9 * tx;
      }
      off -= off2;
      sens -= sens2;
    }

    int32_t p = ((int64_t)d1 * sens/(1LL<<21) - off) / (1LL << 15);
    *pressure = (float)p / 100;
  }
  return true;
}

uint32_t BaroSensorClass::takeReading(uint8_t CMD, BaroOversampleLevel oversample_level)
{
  _i2c -> beginTransmission(BARO_ADDR);
  _i2c -> write(CMD | oversample_level);
  err = _i2c -> endTransmission();

  if(err)
    return 0;

  uint8_t sampling_delay = pgm_read_byte(SamplingDelayMs + (int)oversample_level);
  delay(sampling_delay);

  //Taken from Onehorse's MS5637 implementation  
    uint8_t data[3] = {0,0,0};
    uint8_t err;
    _i2c -> beginTransmission(BARO_ADDR);  // Initialize the Tx buffer
    _i2c -> write(0x00);                   // Put ADC read command in Tx buffer
    err = _i2c -> endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
    //Serial.println(err);

    uint8_t i = 0;
    _i2c -> requestFrom(BARO_ADDR, 3);     // Read three bytes from slave PROM address 
    while (_i2c -> available()) {
        data[i++] = _i2c -> read(); }      // Put read results in the Rx buffer
    uint32_t result = (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program

  return result;
}

void BaroSensorClass::dumpDebugOutput()
{
  Serial.print(F("C1 = "));
  Serial.println(c1);
  Serial.print(F("C2 = "));
  Serial.println(c2);
  Serial.print(F("C3 = "));
  Serial.println(c3);
  Serial.print(F("C4 = "));
  Serial.println(c4);
  Serial.print(F("C5 = "));
  Serial.println(c5);
  Serial.print(F("C6 = "));
  Serial.println(c6);
  Serial.print(F("d1 first = "));
  Serial.println(takeReading(CMD_START_D1(OSR_8192), OSR_8192));
  Serial.print(F("d2 first = "));
  Serial.println(takeReading(CMD_START_D2(OSR_8192), OSR_8192));
  Serial.print(F("d1 second = "));
  Serial.println(takeReading(CMD_START_D1(OSR_8192), OSR_8192));
  Serial.print(F("d2 second = "));
  Serial.println(takeReading(CMD_START_D2(OSR_8192), OSR_8192));
  Serial.print(F("d1 third = "));
  Serial.println(takeReading(CMD_START_D1(OSR_8192), OSR_8192));
  Serial.print(F("d2 third = "));
  Serial.println(takeReading(CMD_START_D2(OSR_8192), OSR_8192));
  float temp, pressure;
  bool res = getTempAndPressure(&temp, &pressure);
  Serial.print(F("result (fourth) = "));
  Serial.println(res ? F("OK") : F("ERR"));
  Serial.print(F("Temp (fourth) = "));
  Serial.println(temp);
  Serial.print(F("Pressure (fourth) = "));
  Serial.println(pressure);
  Serial.print(F("Error (fourth) = "));
  Serial.println(err);
}

void BaroSensorClass::I2Cscan() {
  //Taken from Onehorse's MS5637 implementation  
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    _i2c -> beginTransmission(address);
    error = _i2c -> endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    while (1) {};
  } else {
    Serial.println("done\n");
  }
}