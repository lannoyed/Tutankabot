#include <cstdio>
#include <cmath>
#include <iostream>
#include <tuple>
#include <string.h>

using namespace std;

// Instantiation of class Potential Field : mother class of Attractive and Repulsive forces.

class Potential_Field
{
    // Access specifier: Public class accessible everywhere.
    public:

    // Data Members : the attribute of the class : the current position given by a tuple (position_vector) and the coefficient (attraction or repulsion).
    tuple <float, float> position_vector;
    tuple <float, float> goal_position;
    float coefficient; // FLOAT OR DOUBLE ?

    // A priori no functions needed to be implemented here.

};

class Attractive_Force : public Potential_Field
{
    public:

    void setCoefficient(float value) {
        coefficient = value;
    }
    void setPosition(tuple <float, float> position){
        position_vector = position;
    }

    void setGoalPosition(tuple <float, float> current_goal){
        goal_position = current_goal; 
    }

    float forceApplied(float value){
        setCoefficient(value);
        float k_att = coefficient;
        
        return; // Calculation of the total attractive force at the present point.
    }

};

