#include <SPI.h>
#include <SPI_MSTransfer.h>
#include "A_ConfigDefines.h"

//#define SPI_SPEED 30500
#define SPI_SPEED 30000000
#define OT_CALC   100*(30000000/SPI_SPEED)

SPI_MSTransfer teensy_gpio = SPI_MSTransfer("Serial", SPI_MST_CS, &SPI_MST_BUS, SPI_SPEED ); // bad with default timeouts

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000 ) {}
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  teensy_gpio.onTransfer(myCallback);
  teensy_gpio.debug(Serial);

#ifdef SPI_MST_SCK
  SPI_MST_BUS.setSCK( SPI_MST_SCK );
#endif
  SPI_MST_BUS.begin();
}

void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
  Serial.print("PacketID: "); Serial.println(info.packetID);
  Serial.print("Length: "); Serial.println(length);
  for ( uint16_t i = 0; i < length; i++ ) {
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

uint32_t OverTime = 0;
void loop() {
  static uint32_t _timer = millis();
  if ( !(millis() % 100) )  { teensy_gpio.pinToggle(LED_BUILTIN); Serial.print("^LT"); delay(1); }
  if ( millis() - _timer >= 1 ) {
    _timer = millis();
    uint32_t _time = micros();

    uint16_t *buf;
    double MST_PrintVals[12];
    buf = (uint16_t *)MST_PrintVals;
    int ii = 0;
    static uint16_t __count = 0;
    static uint16_t __countB = 0;
    for ( uint32_t i = 0; i < sizeof(MST_PrintVals) / sizeof( MST_PrintVals[0]  ); i++ ) MST_PrintVals[i] = __count++;
    Serial.print("F&F (OT=");
    Serial.print( OverTime );
    Serial.print(")");
    _time = micros();
    __countB++;
    if ( __countB % 25 )
      teensy_gpio.transfer16((uint16_t *)MST_PrintVals, sizeof(MST_PrintVals) / 2, 60, 1); // DEBUGHACK output
    else
      teensy_gpio.transfer16((uint16_t *)MST_PrintVals, sizeof(MST_PrintVals) / 2, 55, 1);
    _time = micros() - _time;
    Serial.print(" OT_CALC==");
    Serial.print(OT_CALC);
    Serial.print("  micros() _time==");
    Serial.println(_time);
    if ( _time > OT_CALC ) OverTime++;
    teensy_gpio.events();
  }
}
