/*
# include "ctrlOut.h"
# include <chrono>


// ajouter la structure ctrl out à ctrl struct

int main(){
    ctrlOut outputs;
    ctrlOut_init(&outputs);
    
   	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now() ; 
	double deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0).count();
    
    int i = 0;
    while (i < 20 ){
        
        
        outputs.M1 += 2.0;
        outputs.M2 += 2.0;
        
        t0 = std::chrono::high_resolution_clock::now() ;
        send_commands(&outputs);
        t1 = std::chrono::high_resolution_clock::now() ;
        
        deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0).count();

        std::cout << deltaT << "\n"; 
        
        usleep(1000000);
        i++;

    }
    motors_stop(&outputs);
}
*/