# include "SPICom.h" 

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
	buffer[5] = 0x00 ; //Bizarre ? Faut s'arrêter à 4 logiquement.
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


double buffer_to_double_distance(unsigned char* buffer)
{
	double count = 0 ; 
	// Only 3 bytes are necessary : the 2,3,4 car le 1 est à 0 en permanence.
	count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16);
	/*for (int i = 0 ; i < 3 ; i++)
	{
		std::string sName(reinterpret_cast<char*>(buffer[4-i]));
		count += pow(16, i*2) * (double) std::stoi(sName, 0, 16);

		//count += pow(16,i)*(double)buffer[4-i]; 
	} */
	return count; 
}

double get_speed(int spi_number){
	// Returns the speed of the odometer represented by spi_number 
	double tick_diff ; 
	unsigned char buffer[5] ; 
	SPI_receive(spi_number, buffer) ;
	tick_diff = buffer_to_double(buffer) ;
	if (spi_number == 4 || spi_number == 5){
		tick_diff /= 0.02 ; // The tick diff is now a number of tick per second 
		tick_diff /= 8192 ; // Now a number of turn per second  
		if (spi_number == 4){
			tick_diff *= -1 ; 
		}
	} else { 
		tick_diff /= 0.02 ; 	// The tick_diff is now a number of tick per second
		tick_diff /= 2465 ; 	// Now a number of turn of the wheel per second  
		if (spi_number == 7){
			tick_diff *= -1 ; 
		}
	}
	return -tick_diff*2*M_PI ; 
}


double get_distance(int spi_number)
{
	double number_of_ticks;
	unsigned char buffer[5];

	SPI_receive(spi_number, buffer);
	number_of_ticks = buffer_to_double_distance(buffer);

	return number_of_ticks * FACTOR + OFFSET;
}