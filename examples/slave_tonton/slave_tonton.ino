#include <SPI_MSTransfer.h>
#include "FastCRC.h"
#include "atomic"
#include "TeensyThreads.h"
#define DEBUGHACK 1 // Set this to 1 to run against TEST MASTER EXAMPLE

SPI_MSTransfer slave = SPI_MSTransfer("SLAVE", "STANDALONE");




void setup() {
  slave.begin();
//  slave.watchdog(10000);
//attachInterrupt(2, spi0_isr, FALLING);

  Serial.begin(115200);
  while (!Serial && micros() < 5000 )
  { Serial.print( "Teensy Online @ millis=" );
    Serial.println( millis() );
    delay(10);
  }
  Serial.print( "Teensy Online @ millis=" );
  Serial.println( millis() );
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  slave.onTransfer(myCallback);
  // slave.debug(Serial); // SPI_MST Debug error tracking


  int val = 0;

  //  std::lock_guard<std::mutex> lock(g_mutex);
  //  Serial.print("LOCKED");

  //  g_mutex.unlock();


}
void led() {
  digitalWrite(13, !digitalRead(13));
}
void yield() {}
void loop() {

//if (GPIOD_PDIR & 0x01) SPI0_SR |= SPI_SR_RFDF;
  
  slave.events();
//return;
  static uint32_t _timer = millis();
  if ( millis() - _timer > 500 ) {
    _timer = millis();
    uint16_t buf[20];
    for ( uint16_t i = 0; i < 20; i++ ) buf[i] = random(0x10,0xFF);
    slave.transfer16(buf, 20, random(1,65534));
  }

}

double LastVal = 100000;
uint32_t ChkErr = 0;
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
  double DiffMiss=0;
  if ( 55 == info.packetID ) {
    if ( 48 != length ) {
      Serial.print("Bad Length: "); Serial.println(length);
    }
    else if ( 0 != info.error ) {
      Serial.println("\nBad CRC: ");
    }
    else {
      double* MST_PrintVals;
      MST_PrintVals = (double *)buffer;

      if ( 1 == DEBUGHACK ) {
        for ( uint32_t ii = 0; ii < 12; ii++ ) {
          LastVal++;
          if ( 65536 == LastVal ) LastVal = 0;
          if ( 100001 == LastVal ) LastVal = MST_PrintVals[ii];
          if ( LastVal != MST_PrintVals[ii] ) {
            DiffMiss = LastVal - MST_PrintVals[ii];
            ChkErr++;
            Serial.print("\nBad LASTVAL TEST INCREMENT <<<<<<<<<<<<<<<<<<<< DIFF OF> ");
            Serial.println( DiffMiss );
            LastVal = MST_PrintVals[ii];
          }
          Serial.print( MST_PrintVals[ii] );
          Serial.print(",");
        }
          Serial.print( ChkErr );
          Serial.println();
      }
      else
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

      /*      for ( uint16_t i = 0; i < (length / 2); i++ ) {
              Serial.print(MST_PrintVals[i], HEX); Serial.print(" __ ");
              Serial.print(MST_PrintVals[i]); Serial.print(" | ");
            }
            */
    }
  }
  else {
    Serial.print("PacketID: ");
    Serial.println(info.packetID);
  }
}