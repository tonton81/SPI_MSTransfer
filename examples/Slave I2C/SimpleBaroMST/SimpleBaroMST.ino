#include "SPI_MSTransfer.h"
#include "A.Configs.h"

//#define SPI_SPEED 30500
#define OT_CALC   100*(30000000/SPI_SPD)

SPI_MSTransfer teensy_gpio = SPI_MSTransfer("Serial", SPI_MST_CS, &SPI_MST_BUS, SPI_SPD ); // bad with default timeouts
SPI_MSTransfer sWire = SPI_MSTransfer("Wire", SPI_MST_CS, &SPI_MST_BUS, SPI_SPD);

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

#define MS5637_ADDRESS 0x76   // Address of altimeter

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

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

  delay(2500);
  I2Cscan();
  delay(2500);
  
 // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  Serial.println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  delay(100);
  Serial.println("PROM dta read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  delay(100);
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
  Serial.println();
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

    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    //Serial.print("D1/2: "); Serial.print(D1); Serial.print(", "); Serial.println(D2);

    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa
  
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters
    float baroin = Pressure; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
   
    Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    Serial.println();  
  }
  teensy_gpio.events();
  sWire.events();

}

// I2C communication with the MS5637 is a little different from that with the MPU9250 and most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

void MS5637Reset()
{
  uint8_t err;
  sWire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
  sWire.write(MS5637_RESET);                // Put reset command in Tx buffer
  err = sWire.endTransmission();            // Send the Tx buffer
  //Serial.println(err);
}
        
void MS5637PromRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};
    uint8_t err;
    for (uint8_t ii = 0; ii < 7; ii++) {
      sWire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
      sWire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
      err = sWire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
      //Serial.println(err);
      uint8_t i = 0;
      sWire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
      while (sWire.available()) {
        data[i++] = sWire.read(); }               // Put read results in the Rx buffer
      destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
    }
}

uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
{
    uint8_t data[3] = {0,0,0};
    uint8_t err;
    sWire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    sWire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
    err = sWire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    //Serial.println(err);
    switch (OSR)
    {
      case ADC_256: delay(1); break;  // delay for conversion to complete
      case ADC_512: delay(3); break;
      case ADC_1024: delay(4); break;
      case ADC_2048: delay(6); break;
      case ADC_4096: delay(10); break;
      case ADC_8192: delay(20); break;
    }
   
    sWire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    sWire.write(0x00);                        // Put ADC read command in Tx buffer
    err = sWire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    //Serial.println(err);

    uint8_t i = 0;
    sWire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
    while (sWire.available()) {
        data[i++] = sWire.read(); }               // Put read results in the Rx buffer
    return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;
  
  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
}

// I2C scan function

void I2Cscan() {
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
    sWire.beginTransmission(address);
    error = sWire.endTransmission();

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
    Serial.println("No I2C devices found");
    Serial.println("Check Wiring !\n");
  } else {
    Serial.println("done\n");
  }
}

void myCallback(uint16_t *buffer, uint16_t length, AsyncMST info) {
  Serial.print("PacketID: "); Serial.println(info.packetID);
  Serial.print("Length: "); Serial.println(length);
  for ( uint16_t i = 0; i < length; i++ ) {
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

