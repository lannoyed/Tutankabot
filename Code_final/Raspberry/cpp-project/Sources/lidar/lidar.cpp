#include "lidar.h"

rp::standalone::rplidar::RPlidarDriver* lidar;
enum {PURPLE, YELLOW} ; 

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

void update_lidar_data(std::chrono::high_resolution_clock::time_point last_lidar_update, double* angles, double* dist, double* quality){
	rplidar_response_measurement_node_hq_t nodes[8192]; 
	size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t) ; 
	u_result scan = lidar->grabScanDataHq(nodes, nodeCount) ;
	if (IS_OK(scan)){
		lidar->ascendScanData(nodes, nodeCount);
		for(int i = 0 ; i < nodeCount; i++){
			angles[i] = 2*M_PI - ((nodes[i].angle_z_q14 * 90.f / (1 << 14))*M_PI/180.0 - M_PI/2 );
			if (angles[i] > 2*M_PI){
				angles[i] -= 2*M_PI ; 	
			}
			dist[i] = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
			quality[i] = nodes[i].quality;
		}
	}
	last_lidar_update = std::chrono::high_resolution_clock::now() ;
}

int check_beacon1(double x, double y, int team){
	if (team == YELLOW){
		if (y < 3.3 && y > 2.9){
			if (x > -0.1 && x < 0.4){
				return 1 ; 
			}
		} return 0 ; 
	} else if (team == PURPLE){
		if (y > -0.3 && y < 0.1){
			if (x >1.7 && x < 2.1){
				return 1 ; 
			}
		} return 0 ; 
	} else{
		return 0 ; 
	}
	return 0 ; 
}

int check_beacon2(double x, double y, int team){
	if (team == YELLOW){
		if (y < 3.3 && y > 2.9){
			if (x > 1.9 && x < 2.1){
				return 1 ;
			}				
		} return 0 ; 
	} else if (team == 	PURPLE){
		if (y > -0.3 && y < 0.1){
			if(x > 1.7 && x < 2.1){
				return 1 ; 
			}
		} return 0 ; 
	} else {
		return 0 ; 
	}
	return 0 ; 
}

int check_beacon3(double x, double y, int team){
	if (team == YELLOW){
		if (y > -0.2 && y < 0.05){
			if (x > 0.8 && x < 1.2){
				return 1 ; 
			}
		} return 0 ;
	} else if (team == PURPLE){
		if (y < 3.3 && y > 2.9){
			if (x > 0.8 && x < 1.2){
				return 1 ; 
			}
		} return 0 ; 
	} else {
		return 0 ; 
	}
	return 0 ; 
}
