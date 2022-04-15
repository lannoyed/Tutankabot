# include <stdio.h>
# include <stdlib.h> 
# include <unistd.h> 
# include <wiringPiSPI.h>
# include <math.h>


#define factor 6.8 * (10**(-6))  // Speed of sound in air (340 [m/s]) divided by the CLOCK_50 (50 * 10**6 [Hz]).

void SPI_init(); 
void SPI_receive(int spi_number, unsigned char* buffer); 
double buffer_to_double(unsigned char* buffer); 

double buffer_to_double_distance(unsigned char* buffer);

double get_speed(int spi_number); 

double get_distance(int spi_number);
