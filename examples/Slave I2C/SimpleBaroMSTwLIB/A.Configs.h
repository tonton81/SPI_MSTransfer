
#define mjs
//#define defragster
//#define DK

#if defined(mjs)
  //--------------------------------------------------------------------
  // SPI_MSTranfer defines
  //--------------------------------------------------------------------
  #define SPI_MST_BUS   SPI
  #define SPI_MST_CS    15
  #define SPI_MST_SCK   14
  #define SPI_SPD       30000000
  
  // TViewer Declares
  #define TViewerSPI     1

  // IMU Declares ----------------------------------
  #define IMU_BUS      Wire  //Wire // SPI
  #define IMU_ADDR     0x68  //0x69 // 0x68 // SPI 9
  #define IMU_SCL        19  //47 // 0x255
  #define IMU_SDA        18  //48 // 0x255
  #if (F_CPU == 240000000)
    #define IMU_SPD   3400000  //1000000 // 0==NULL or other
  #else
    #define IMU_SPD   2000000  //1000000 // 0==NULL or other
  #endif
  #define IMU_SRD         9  // Used in initIMU setSRD() - setting SRD to 9 for a 100 Hz update rate
  #define IMU_INT_PIN    24  //50 // 1 // 16

  // Serial Declares --------------------------------
  #define cout        Serial
  #define cout_BAUD   115200
  #define coutD       Serial  //Serial4  // Serial2
  #define coutD_BAUD  2000000  // coutD for DEBUG port
  
  //---- GPS Declares -------------------------------
  #define GPS_Start       1  // 1 // UBLOX gps(gps_Start);
  #define GPS_PORT  Serial1  // Serial1
  #define GPS_BAUD   460800
  #define GPS_SRX        27  //27 //9 // Must be set to Serial# Rx Pin #
  #define GPS_AltRx      27  // 27

  //Set print rate, sincePrint > printRate
  #define printRate     10  //milliseconds
  
#elif defined(defragster)
  //--------------------------------------------------------------------
  // SPI_MSTranfer defines
  //--------------------------------------------------------------------
  #define SPI_MST_BUS   SPI
  #define SPI_MST_CS    15
  #define SPI_MST_SCK   14
  #define SPI_SPD       30000000

  // TViewer Declares
  #define TViewerSPI     1

  // IMU Declares
  #define IMU_BUS      Wire  //Wire // SPI
  #define IMU_ADDR     0x69  //0x69 // 0x68 // SPI 9
  #define IMU_SCL       47  //47 // 0x255
  #define IMU_SDA       48  //48 // 0x255
  #if (F_CPU == 240000000)
    #define IMU_SPD   3400000  //1000000 // 0==NULL or other
  #else
    #define IMU_SPD   2000000  //1000000 // 0==NULL or other
  #endif  #define IMU_SRD       19   // Used in initIMU setSRD() - setting SRD to 9 for a 100 Hz update rate
  #define IMU_INT_PIN   50  //50 // 1 // 14

  // Serial Declares
  #define cout        Serial
  #define cout_BAUD   115200
  // #define coutD      Serial1  //Serial4  // Serial2
  #define coutD_BAUD 2000000 // 115200  // coutD for DEBUG port

  // GPS Declares
  #define GPS_Start       2  // 1 // UBLOX gps(gps_Start);
  #define GPS_PORT  Serial2  // Serial1
  #define GPS_BAUD   460800
  #define GPS_SRX         9  //27 //9 // Must be set to Serial# Rx Pin #
  #define GPS_AltRx     255  // 27 // else 255

  //Set print rate, sincePrint > printRate
  #define printRate      25  //milliseconds

#elif defined(DK)
  //--------------------------------------------------------------------
  // SPI_MSTranfer defines
  //--------------------------------------------------------------------
  #define SPI_MST_BUS   SPI
  #define SPI_MST_CS    15
  #define SPI_MST_SCK   14
  #define SPI_SPD       30000000
  
  // TViewer Declares
  #define TViewerSPI     1

  // IMU Declares
  #define IMU_BUS       Wire  //Wire // SPI
  #define IMU_ADDR     0x68  //0x69 // 0x68 // SPI 9
  #define IMU_SCL       19  //47 // 0x255
  #define IMU_SDA       18  //48 // 0x255
  #if (F_CPU == 240000000)
    #define IMU_SPD   3400000  //1000000 // 0==NULL or other
  #else
    #define IMU_SPD   2000000  //1000000 // 0==NULL or other
  #endif  #define IMU_SRD        9   // data output rate = 1000 / (1 + SRD)
  #define IMU_INT_PIN    1  //50 // 1 // 14

  // Serial Declares
  #define cout       Serial
  #define cout_BAUD  115200
  //#define coutD     Serial3  //Serial4  // Serial2
  //#define coutD_BAUD 2000000  // coutD for DEBUG port
  
  //---- GPS Declares
  #define GPS_Start       3  // 1 // UBLOX gps(gps_Start);
  #define GPS_PORT  Serial3  // Serial1
  #define GPS_BAUD   115200  // 460800 // 115200
  #define GPS_SRX         9  //27 //9 // Must be set to Serial# Rx Pin #
  #define GPS_AltRx     255  // 27

  //Set print rate, sincePrint > printRate
  //#define printRate      25  //milliseconds
  #define printRate       100  //milliseconds
  
#endif



