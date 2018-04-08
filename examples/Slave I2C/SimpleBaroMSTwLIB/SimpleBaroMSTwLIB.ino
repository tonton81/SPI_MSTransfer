#include "SPI_MSTransfer.h"
#include "A.Configs.h"
#include <BaroSensor.h>

//#define SPI_SPEED 30500
#define OT_CALC   100*(30000000/SPI_SPD)
SPI_MSTransfer teensy_gpio = SPI_MSTransfer("Serial", SPI_MST_CS, &SPI_MST_BUS, SPI_SPD ); // bad with default timeouts
SPI_MSTransfer sWire = SPI_MSTransfer("Wire", SPI_MST_CS, &SPI_MST_BUS, SPI_SPD);

BaroSensorClass BaroSensor;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000 ) {}
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  teensy_gpio.onTransfer(myCallback);
  teensy_gpio.debug(Serial);

  #ifdef SPI_MST_SCK
    SPI_MST_BUS.setSCK( SPI_MST_SCK );
  #endif
  SPI_MST_BUS.begin();
  
  sWire.begin();

  BaroSensor.I2Cscan();
  BaroSensor.begin(sWire);
  BaroSensor.dumpDebugOutput();
}

uint32_t OverTime = 0;
void loop()
{
  static uint32_t _timer = millis();
  if ( millis() - _timer >= 100 ) {
    teensy_gpio.pinToggle(LED_BUILTIN);
    _timer = millis();
    uint32_t _time = micros();
    _time = micros();
    if(!BaroSensor.isOK()) {
      Serial.print("Sensor not Found/OK. Error: "); 
      Serial.println(BaroSensor.getError());
      BaroSensor.begin(sWire); // Try to reinitialise the sensor if we can
    }
    else {
      Serial.print("Temperature: "); 
      Serial.println(BaroSensor.getTemperature());
      Serial.print("Pressure:    ");
      Serial.println(BaroSensor.getPressure());
    }
    
    Serial.print("F&F (OT=");
    Serial.print( OverTime );
    Serial.print(")");
    _time = micros() - _time;
    Serial.print(" OT_CALC==");
    Serial.print(OT_CALC);
    Serial.print("  micros() _time==");
    Serial.println(_time);
    if ( _time > OT_CALC ) OverTime++;

   Serial.println();  Serial.println();
  
  }
  teensy_gpio.events();
  sWire.events();
}

void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
  Serial.print("PacketID: "); Serial.println(info.packetID);
  Serial.print("Length: "); Serial.println(length);
  for ( uint16_t i = 0; i < length; i++ ) {
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

