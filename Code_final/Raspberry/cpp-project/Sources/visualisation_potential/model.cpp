#include "model.h"

double potential_Field_iter (double x, double y, PotentialStruct* potential_Field){
    double Force= 0.0;

    double distance = 0.0; 
    double rho = 0.1;
    double Fmax = 10.0;

    distance = fabs(potential_Field->border1_x - x);
    if (distance < rho){
        Force += (-1/rho + 1/distance) *  1/pow(distance,2);
    }

    distance = fabs(potential_Field->border2_x - x);
    if (distance < rho){
        Force += (-1/rho + 1/distance) *  1/pow(distance,2);
    }

    
    distance = fabs(potential_Field->border3_y - y);
    if (distance < rho){
        Force += (-1/rho + 1/distance) *  1/pow(distance,2);
    }

    distance = fabs(potential_Field->border4_y - y);
    if (distance < rho){
        Force += (-1/rho + 1/distance) *  1/pow(distance,2);
    }

    distance = sqrt( pow(potential_Field->obstacle1_x - x, 2 ) + pow(potential_Field->obstacle2_y - y, 2) );
    if (distance < rho){
        Force += (-1/rho + 1/distance) *  1/pow(distance,2);
    }

    Force *= 100;
    if (Force > Fmax){
        Force = Fmax;
    } else if (Force < -Fmax)
    {
        Force = -Fmax;
    }
    
    return Force ;
}

void initPotentialField ( PotentialStruct * potential_Field){
    potential_Field ->border1_x = -1;
    potential_Field ->border2_x =  1;
    potential_Field ->border3_y = -1;
    potential_Field ->border4_y =  1;

    potential_Field ->obstacle1_x = 0.2;
    potential_Field ->obstacle2_y = 0.2; 

}


