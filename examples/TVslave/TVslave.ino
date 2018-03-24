#include <SPI_MSTransfer.h>

SPI_MSTransfer slave = SPI_MSTransfer("SLAVE", "STANDALONE");


void setup() {
	Serial.begin(115200);
	while (!Serial && millis() < 5000 )
	{	Serial.print( "Teensy NOT Online @ millis=" );
		Serial.println( millis() );
		delay(30);
	}
	Serial.print( "Teensy Online @ millis=" );
	Serial.println( millis() );
	Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);

	if ( ARM_DWT_CYCCNT == ARM_DWT_CYCCNT ) {
		Serial.print( "Cycle Counter Not Enabled :" );
		Serial.println( ARM_DWT_CYCCNT );
	}
	if ( ARM_DWT_CYCCNT == ARM_DWT_CYCCNT ) {
		// Enable CPU Cycle Count
		ARM_DEMCR |= ARM_DEMCR_TRCENA;
		ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	}
	if ( ARM_DWT_CYCCNT != ARM_DWT_CYCCNT ) {
		Serial.print( "Cycle Counter Enabled! :" );
		Serial.println( ARM_DWT_CYCCNT );
	}
	slave.begin( );
	slave.onTransfer(myCallback);
	// slave.debug(Serial); // SPI_MST Debug error tracking

}
void led() {
	digitalWrite(13, !digitalRead(13));
}

void loop() {
	slave.events();
}

static uint16_t last_packetID = 0;
elapsedMillis TogClk;
uint32_t TogCnt = 0;
uint32_t FaFCnt = 0;
void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
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
		}
	}
	else if ( 60 == info.packetID ) {
		if ( 48 != length ) {
			Serial.print("Bad Length: "); Serial.println(length); // If this shows then the SPI Passed array size is nor wrong
		}
		else if ( 0 != info.error ) {
			Serial.println("\nBad CRC: ");
		}
		else {
			double* MST_PrintVals;
			MST_PrintVals = (double *)buffer;

// This provides a terse output with error checked based on MASTER output to the 12 DOUBLE value Array
			{
				static uint16_t TogLast = 0;
				static uint16_t TogHz =0;
				static uint16_t FaFHz =0;
				static double LastVal = 100000;
				static uint32_t ChkErr = 0;
				static uint32_t CBcount = 0;
				if ( digitalReadFast( LED_BUILTIN) != TogLast ) { 
					TogLast = !TogLast; 
					TogCnt++; 
				}
				if ( TogClk >=1000 ) {
					TogClk -= 1000;
					TogHz = TogCnt;
					TogCnt = 0;
					FaFHz = FaFCnt;
					FaFCnt = 0;
				}
				if ( last_packetID != info.packetID ) {
					LastVal += 12;
					if ( LastVal > 65536 && LastVal < 100000 ) LastVal -= 65536;
				}
				elapsedMillis LastCB;
				double DiffMiss = 0;
				uint32_t ErrSeen = 0;
				CBcount++;
				FaFCnt++;
				for ( uint32_t ii = 0; ii < 12; ii++ ) {
					LastVal++;
					if ( 65536 == LastVal ) LastVal = 0;
					if ( LastCB > 10 || 100001 == LastVal || 100013 == LastVal ) LastVal = MST_PrintVals[ii];
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
				Serial.print(" [");
				Serial.print( FaFHz );
				Serial.print(" ,");
				Serial.print( TogHz );
				Serial.println();
				LastCB = 0;
			}
		}
	}
	else {
		Serial.print("PacketID: ");
		Serial.println(info.packetID);
	}
	last_packetID = info.packetID;
}
