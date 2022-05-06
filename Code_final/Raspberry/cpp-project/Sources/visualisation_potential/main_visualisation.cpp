
#include <math.h>
#include "visual_potential.h"


int main_visualisation (int argc, char** argv){

    PotentialStruct * myPotentialField = (PotentialStruct *) malloc(sizeof(PotentialStruct));
    initPotentialField ( myPotentialField);

    visualisation_potential(-1.0, 1.0, -1.0,1.0, 100, myPotentialField);
	
	
	return 0;
}

// g++ main.cpp model.h model.cpp visual_potential.h visual_potential.cpp -o visual_potential