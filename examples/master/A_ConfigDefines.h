
#define defragster
// tonton81


#if defined(tonton81)

#define SPI_MST_BUS		SPI2
#define SPI_MST_CS 		43

#elif defined(defragster)

#define SPI_MST_BUS		SPI
#define SPI_MST_CS 		15
#define SPI_MST_SCK 	14

#endif

