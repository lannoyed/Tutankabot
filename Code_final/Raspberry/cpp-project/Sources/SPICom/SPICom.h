# include <stdio.h>
# include <stdlib.h> 
# include <unistd.h> 
# include <wiringPiSPI.h>
# include <math.h>

void SPI_init() ; 
void SPI_receive(int spi_number, unsigned char* buffer) ; 
double buffer_to_double(unsigned char* buffer) ; 
double get_speed(int spi_number) ; 
