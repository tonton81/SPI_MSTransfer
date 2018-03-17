#include <SPI_MSTransfer.h>
#include "A_ConfigDefines.h"

SPI_MSTransfer slave = SPI_MSTransfer("SLAVE", "STANDALONE");


void setup() {
	Serial.begin(115200);
	while (!Serial && micros() < 5000 )
	{	}
  delay(5000);
	Serial.print( "Teensy Online @ millis=" );
	Serial.println( millis() );
	Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  LINE("F_CPU", F_CPU);
	
	slave.begin();
	slave.onTransfer(myCallback);
	//slave.debug(Serial); // SPI_MST Debug error tracking
}
void led() {
	digitalWrite(13, !digitalRead(13));
}

void loop() {
  static uint32_t _timer = millis();
  //if ( !(millis() % 1000) )  { teensy_gpio.pinToggle(LED_BUILTIN); Serial.print("^LT"); delay(1); }
  if ( millis() - _timer > 40 ) {
    uint16_t *buf;
    double MST_PrintVals[12];
    buf = (uint16_t *)MST_PrintVals;
    int ii = 0;
    static uint16_t __count = 0;
    for ( uint32_t i = 0; i < sizeof(MST_PrintVals) / sizeof( MST_PrintVals[0]  ); i++ ) MST_PrintVals[i] = __count++;
    _timer = millis();
    slave.transfer16((uint16_t *)MST_PrintVals, sizeof(MST_PrintVals) / 2, 55, 1);
  }
  slave.events();  //original
}

FASTRUN void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
	if ( 55 == info.packetID ) {
		if ( 48 != length ) {
			Serial.print("Bad Length: "); Serial.println(length); // If this shows then the SPI Passed array size is nor wrong
		}
		else if ( 0 != info.error ) {
			Serial.println("\nBad CRC: ");
		}
		else {
			double* MST_PrintVals;
			MST_PrintVals = (double *)buffer;

#ifndef DEBUGHACK // FOR Runtime to match the FORUM MPU_GPS code edit this to match TViewer output
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
#else 	// This provides a terse output with error checked based on MASTER output to the 12 DOUBLE value Array
			{
				static double LastVal = 100000;
				static uint32_t ChkErr = 0;
				static uint32_t CBcount = 0;
				elapsedMillis LastCB;
				double DiffMiss = 0;
				uint32_t ErrSeen = 0;
				CBcount++;
				for ( uint32_t ii = 0; ii < 12; ii++ ) {
					LastVal++;
					if ( 65536 == LastVal ) LastVal = 0;
					if ( LastCB > 10 || 100001 == LastVal ) LastVal = MST_PrintVals[ii];
					if ( LastVal != MST_PrintVals[ii] ) {
						DiffMiss = LastVal - MST_PrintVals[ii];
						ChkErr++;
						Serial.print("\nBad LASTVAL TEST INCREMENT <<<<<<<<<<<<<<<<<<<< DIFF OF> ");
						Serial.println( DiffMiss );
						LastVal = MST_PrintVals[ii];
						ErrSeen++;
					}
					if ( 0 != ErrSeen || 1 > ii ) {
						Serial.print( MST_PrintVals[ii] );
						Serial.print(",");
					}
					else {
						Serial.print(" #,");
					}
				}
				Serial.print( CBcount );
				Serial.print(",");
				Serial.print( ChkErr );
				Serial.println();
				LastCB = 0;
			}
#endif
		}
	}
	else {
		Serial.print("PacketID: ");
		Serial.println(info.packetID);
	}
}
