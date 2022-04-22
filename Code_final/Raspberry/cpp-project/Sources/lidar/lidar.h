#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h"
#include <chrono>
#include <math.h>


#include <thread>
#include <mutex>
#include <atomic>

#ifndef M_PI
    #define M_PI 3.1415926
#endif


void connectLidar();

void disconnectLidar();

void lidarToFile(char *filename);

void update_lidar_data(std::chrono::high_resolution_clock::time_point last_lidar_update, double* angles, double* dist, double* quality)  ;

int check_beacon1(double x, double y, int team) ; 

int check_beacon2(double x, double y, int team) ; 

int check_beacon3(double x, double y, int team) ; 