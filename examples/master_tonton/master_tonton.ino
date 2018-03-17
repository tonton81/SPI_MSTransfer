#include <SPI.h>
#include <SPI_MSTransfer.h>
#include "A_ConfigDefines.h"

SPI_MSTransfer teensy_gpio = SPI_MSTransfer("Serial", SPI_MST_CS, &SPI_MST_BUS, 30000000 );

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000 ) {}
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  teensy_gpio.onTransfer(myCallback);
  //  teensy_gpio.debug(Serial);

    #ifdef SPI_MST_SCK
      SPI_MST_BUS.setSCK( SPI_MST_SCK );
    #endif
      SPI_MST_BUS.begin();
}

elapsedMillis ioTest;
void yield() {}
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
  Serial.print("PacketID: "); Serial.println(info.packetID);
  Serial.print("Length: "); Serial.println(length);
  for ( uint16_t i = 0; i < length; i++ ) {
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

void loop() {
//  teensy_gpio._detect();
//  delay(1000);
//  return;
  static uint32_t _timer = millis();
  if ( !(millis() % 1000) )  {
        teensy_gpio.digitalWrite(13, !teensy_gpio.digitalRead(13));
    delay(1);
  }
  if ( millis() - _timer > 10 ) {
    _timer = millis();
    uint32_t _time = micros();

    uint16_t *buf;
    double MST_PrintVals[12];
    buf = (uint16_t *)MST_PrintVals;
    int ii = 0;
    static uint16_t __count = 0;
    for ( uint32_t i = 0; i < sizeof(MST_PrintVals) / sizeof( MST_PrintVals[0]  ); i++ ) MST_PrintVals[i] = __count++;
    Serial.print("returned value ");
    //    teensy_gpio.digitalWrite(13, 1);
    _time = micros();
    teensy_gpio.transfer16((uint16_t *)MST_PrintVals, sizeof(MST_PrintVals) / 2, 55, 1);
    _time = micros() - _time;
    Serial.print("micros() _time==");
    Serial.println(_time);
   }
}