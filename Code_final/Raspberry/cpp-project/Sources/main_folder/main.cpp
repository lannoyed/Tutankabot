#include <stdio.h>

#include "rplidar.h"

int main(int argc, char* argv)
{
   rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

   u_result res = lidar->connect("/dev/ttyUSB0", 115200);
   if (IS_OK(res)){
      printf("Success \n");
   }
   else{
      printf("Failed to connect to LIDAR %08x\r\n", res);
   }


   rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
}