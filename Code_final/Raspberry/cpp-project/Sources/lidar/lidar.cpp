#include "lidar.h"

rp::standalone::rplidar::RPlidarDriver* lidar;

void connectLidar(){ //connects lidar and starts scanning in background

	lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

	u_result res = lidar->connect("/dev/ttyUSB0", 115200);
    if (IS_OK(res)){
    	printf("Connected\n");

    	lidar->startMotor();
    	rp::standalone::rplidar::RplidarScanMode scanMode;
    	lidar->startScan(false, true, 0, &scanMode);
    }
    else{
    	printf("Connection failed\n");

    	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
    }

}

void disconnectLidar(){
	lidar->stopMotor();
    lidar->disconnect();

    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
}

void lidarToFile(char *filename){

	rplidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
    
    u_result scan = lidar->grabScanDataHq(nodes, nodeCount);

    if (IS_OK(scan)){
    	lidar->ascendScanData(nodes, nodeCount);

        FILE *fpt;

        fpt = fopen(filename, "w+");
        fprintf(fpt,"Angle, Distance\n");

        for(int i = 0; i < nodeCount; i++){
        	double angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
            double dist = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
            double quality = nodes[i].quality;
            if(quality > 0.0){
               fprintf(fpt,"%f, %f\n", angle, dist);
            }
        }

        fclose(fpt);
    }
}