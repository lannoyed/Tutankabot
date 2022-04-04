#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "lidar.h"

char data_file[] = "prout.csv";

int main(int argc, char* argv){
   connectLidar();

   lidarToFile(data_file);

   disconnectLidar();
}