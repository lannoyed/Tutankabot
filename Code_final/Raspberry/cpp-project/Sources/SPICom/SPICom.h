# include <stdio.h>
# include <stdlib.h> 
# include <iostream>
# include <string>
# include <unistd.h> 
# include <wiringPiSPI.h>
# include <math.h>


#define FACTOR 3.4 / pow(10, 6)  // Speed of sound in air (340 [m/s]) divided by the CLOCK_50 (50 * 10**6 [Hz]).
#define OFFSET 0.025 // Offset remarqué dans la mesure


void SPI_init(); 
void SPI_receive(int spi_number, unsigned char* buffer); 
double buffer_to_double(unsigned char* buffer); 

double buffer_to_double_distance(unsigned char* buffer);

double get_speed(int spi_number); 

double get_distance(int spi_number);
