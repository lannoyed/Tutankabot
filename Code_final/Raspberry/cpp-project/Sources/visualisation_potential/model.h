#include <iostream>
#include <stdio.h>
#include <math.h>

typedef struct PotentialStruct
{
    double border1_x;
    double border2_x;

    double border3_y;
    double border4_y;

    double obstacle1_x;
    double obstacle2_y;

}PotentialStruct;



double potential_Field_iter (double x, double y, PotentialStruct * myPotentialField);



void initPotentialField ( PotentialStruct * myPotentialField);



