# include <stdio.h>
# include <stdlib.h> 
# include <unistd.h> 
# include <wiringPiSPI.h>
# include <math.h>

void SPI_init(){
	int fd ; 
	fd = wiringPiSPISetup(0.0, 500000) ; 
	sleep(1.0) ; 
	printf("SPI intialized\n") ; 
}

void SPI_receive(int spi_number, unsigned char* buffer){
	/*
	Asks the DE0 nano to send the data located in its memory at the number "spi_number" and write it in the buffer "buffer"
	Buffer must be an 5 char long array
	*/
	buffer[0] = spi_number ; // Initialization of the buffer with the appropriate number 
	buffer[1] = 0x00 ; 
	buffer[2] = 0x00 ; 
	buffer[3] = 0x00 ; 
	buffer[4] = 0x00 ; 
	buffer[5] = 0x00 ; 
	wiringPiSPIDataRW(0.0, buffer, 5) ; 
	buffer[0] = spi_number ; // Repeat the operation to get the information on the buffer
	buffer[1] = 0x00 ; 
	buffer[2] = 0x00 ; 
	buffer[3] = 0x00 ; 
	buffer[4] = 0x00 ; 
	buffer[5] = 0x00 ; 
	wiringPiSPIDataRW(0.0, buffer, 5) ; 
}

double buffer_to_double(unsigned char* buffer){
	int sign ; 
	if (buffer[1] == 1){
		sign = -1 ; 
	} else {
		sign = 1 ; 
	}
	double count = 0 ; 
	for (int i = 0 ; i < 3 ; i++){
		count += pow(16,i)*(double)buffer[4-i] ; 
	} 
	return count*sign ; 
}

double get_speed(int spi_number){
	// Returns the speed of the odometer represented by spi_number 
	double tick_diff ; 
	unsigned char buffer[5] ; 
	SPI_receive(spi_number, buffer) ;
	tick_diff = buffer_to_double(buffer) ; 
	tick_diff /= 0.02 ; // The tick diff is now a number of tick per second 
	tick_diff /= 8192 ; // Now a number of turn per second  
	if (spi_number == 4){
		tick_diff *= -1 ; 
	}
	return tick_diff ; 
}