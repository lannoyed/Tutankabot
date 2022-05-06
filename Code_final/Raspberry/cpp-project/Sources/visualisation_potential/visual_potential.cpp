#include "visual_potential.h"

// ajouter cvs ici 
void visualisation_potential(double x_min, double x_max, double y_min, double y_max, double N_points, PotentialStruct * myPotentialField){
    double x_iter = (x_max - x_min) / N_points; 
    double y_iter = (y_max - y_min) / N_points; 
    FILE * myPotentialFieldFile;
    myPotentialFieldFile =fopen("myPotentialField.txt", "w");
    for (double x_i = x_min; x_i < x_max; x_i += x_iter){
        for (double y_i = y_min; y_i < y_max; y_i += y_iter){
                                                                // norm( totalRepulsiveForce() )
            fprintf(myPotentialFieldFile, "%f %f %f \n", x_i, y_i, potential_Field_iter(x_i, y_i, myPotentialField));
        }
    }
    fclose(myPotentialFieldFile);
}


