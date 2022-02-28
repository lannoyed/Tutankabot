#include <cstdio>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>
#include <list>
#include <string>
#include <cmath>

// Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.

// ========================
// == INCLUDES & DEFINES ==
// ========================

using namespace std; // Might pose a problem later in the work on tuples.

typedef vector< tuple<double,double> > tuple_coord_list; // List of coordinates as tuples.
// Vector : basically works as a list. vector.push_back : add element at the end.

// The obstacles in the map will be represented as circles, just like the robot. 
// Borders will be represented as being rectangles.
#define rho_0_border 0.1; // Distance of influence of border set to 10 [cm]. Every border has the same distance of influence.
#define rho_0_circle 0.15; // Distance of influence of obstacle in map : the samples on the floor have a diameter of 15 [cm], so we say r_influence = 15 [cm].
// Every obstacle in the map has the same distance of influence, as a first generalisation.





// ================
// == CLASS PART ==
// ================


// Instantiation of class Potential Field : mother class of Obstacle.
class Potential_Field
{
    // Access specifier: Public class accessible everywhere.
    public:

    // Data Members : the attribute of the class.
    tuple_coord_list obstacle_list;         // Position of obstacles on the board (not counting in borders).
    tuple <double, double> current_position; // Current position of the robot.
    tuple <double, double> goal_position;   // Current goal position.

    // Default constructor.
    Potential_Field()
    {
        cout << "The default constructor of 'Potential_Field' has been called." << endl; // endl = end_line => '\n' in Python.
        // cout means "character output" and wcout means "wide character output". std::cout format. Quick way of saying "print".
    }

    // Parameterized constructor.
    Potential_Field(tuple <double, double> position, tuple <double, double> goal)
    {
        current_position = position;
        goal_position = goal;
    }

    // Update function to get a new position.
    void setPosition(tuple <double, double> position){
        current_position = position;
    }
    
    // Self explanatory function.
    void setGoalPosition(tuple <double, double> current_goal){
        goal_position = current_goal; 
    }

    // Calculation of the attractive force at the present point towards the goal. This gives back a vector.
    tuple <double, double> forceApplied(double value){
        double co_X = -value * ( get<0>(current_position) - get<0>(goal_position) );
        double co_Y = -value * ( get<1>(current_position) - get<1>(goal_position) );
        tuple <double, double> directionOfForce = make_tuple(co_X, co_Y);
        return directionOfForce; 
    }

    // Calculation of the total repulsive force at the current position. Takes into account every known obstacles.
    tuple <double, double> repulsiveForce(){
        return make_tuple<double, double>(0.0, 0.0);
    }
    // The idea : calculate the repulsive for of each an every single known obstacle. 6 borders and known samples. 


    // Implémentation primitive : on définit chaque obstacle comme un point : sur map : un seul point et on fait un cercle centré en ce point (voir class Sample)

    // À implémenter dans les classes obstacles :
    // IDÉE : si distance entre les deux centres <= 30 [cm] : ça craint !
    // Les bords : if distance(centre-droite du bord) <= 15 [cm] : ça craint !  

};


// Class of an obstacle. Mother class of Border, Robot, Samples
class Obstacle : public Potential_Field
{
    public : 

    // Attributes : common to every obstacles : the repulsive coefficient and the distance of influence.
    double coeff;
    double rho0;

    Obstacle()
    {
        cout << "Default constructor of class 'Obstacle'" << endl;
    }

    // Class constructor. Parameterized constructor : the position of the center is not specified, as a border is represented as a line. Use "setPosition" for other obstacles.
    Obstacle(double k_rep, double distanceOfInfluence)
    {
        coeff = k_rep;
        rho0 = distanceOfInfluence;
    }

    // Adding new obstacles to the list of obstacles. This will allow us to put the number of known obstacles on the map at first and then update it with, typically, 
    // the position of the opponent !
    // Assumption : new obstacles will only be a moving robot or a fallen sample that the other robot may have mooved. 
    tuple_coord_list addObstacleCoord(tuple <double, double> newObstacleCoord){
        obstacle_list.push_back(newObstacleCoord); // obstacle_list is an attribute of the 'Potential_Field' class.
        return obstacle_list;
    }

};


class SimpleBorder : public Obstacle
{
    // Simple border permet le gestion de bordure horizontales ou verticales

    public :

    // Attributes : those inhehirted from 'Obstacles'.
    //            : borderType (0 -> vertical, 1 -> horizontal)
    int borderType;
    double position;

    // Default constructor.
    SimpleBorder()
    {
        cout << "Default constructor of class 'Border'" << endl;
    }

    // Real constructor
    SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition )
    {
        coeff = k_rep;
        rho0 = distanceOfInfluence;
        borderType = border_type;
        position = xoryposition;

    }

    // give the euclidean distance (square) between the line and the center of the robot
    double computeDistance (tuple <double, double> robotPosition)
    {
        if (borderType == 0){
            return pow(get<0>(robotPosition) - position,2);
        }
        else if (borderType == 1){
            return  pow(get<1>(robotPosition) - position, 2);
        }
        else {
            throw "invalide Border type";
        }
    }
};

class OblicBorder : public Obstacle {
    public :

    // Attributes : those inhehirted from 'Obstacles'.
    //            : borderType (0 -> vertical, 1 -> horizontal)
    int borderType;
    // y = mx + p
    double m;
    double p;

    // Default constructor.
    OblicBorder()
    {
        cout << "Default constructor of class 'Border'" << endl;
    }

    // Real constructor
    OblicBorder(double k_rep, double distanceOfInfluence, int border_type, double pente, double offset)
    {
        coeff = k_rep;
        rho0 = distanceOfInfluence;
        borderType = border_type;
        m = pente;
        p = offset;
    }

    // give a approximation the euclidean distance (square) between the line and the center of the robot
    // l idee est que la distance euclidenne sera toujours superieure à la plus petite distance selon les x ou selon les y
    double computeDistance (tuple <double, double> robotPosition)
    {
        double yup; double xup; double distance1; double distance2;

        yup = m * get<0>(robotPosition) + p;
        xup = (get<1>(robotPosition) - p) / m;

        distance1 = pow(get<0>(robotPosition) - xup,2);
        distance2 = pow(get<1>(robotPosition) - yup,2);

        if (distance1 < distance2){
            return distance1;
        }
        else {
            return distance2;
        }
    }

};

class Opponent : public Obstacle
{

    public:

    // Attributes : those inherited from 'Obstacle' and the position of the detected border (estimation of distance from the robot).
    tuple <double, double> position;

    Opponent(tuple <double, double> center, double k_rep, double distanceOfInfluence)
    {
        position = center;
        coeff = k_rep;
        rho0 = distanceOfInfluence;
    }

    // To be implemented : bool isInZone() => if distance de centre robot à point détecté <= 30 [cm] (ou une distance qu'on aura fixé), alors BAD.


    // Harder part : to be implemented.
    // The information we get is coming from the sensors. They will give us an estimation of the distance towards the obstacle.
    // By having an idea of the smallest distance to the opponent and the angle at which it is, we can estimate the position of the opponent.
    void setPosition(tuple <double, double> obstaclePosition){
    
    }
};

class Sample : public Obstacle
{
    public :

    // Attributes : those inherited from 'Obstacle' and the position of its center (known).
    tuple <double, double> position;

    Sample(tuple <double, double> center, double k_rep, double distanceOfInfluence)
    {
        position = center;
        coeff = k_rep;
        rho0 = distanceOfInfluence;
    }

    // To be implemented : bool isInZone() => if distance de centre robot à centre known obstacle <= 30 [cm] (ou une distance qu'on aura fixé), alors BAD.

    // Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
    void setPosition(tuple <double, double> obstaclePosition){
        position = obstaclePosition; 
    }
};




// ===============
// == TEST PART ==
// ===============

string tupleToString(tuple <double, double> entry){
    string output_s;
    output_s = "(" + to_string(get<0>(entry)) + "," + to_string(get<1>(entry)) + ")";
    return output_s;
}

// Test function : instantiate here to see if it works.
int main(int arg, char* argv[]){
    double valeurqcq = 3.0;
    tuple <double, double> position_1 = make_tuple(1.5, 1.5);
    tuple <double, double> goal_1 = make_tuple(1.0, 1.5);

    Potential_Field testObject;
    testObject.setPosition(position_1);
    testObject.setGoalPosition(goal_1);

    // To be modified if we want to test.
    /*
    double result_forceApp = testObject.forceApplied(valeurqcq);

    cout << "Test if everything works fine" << endl;
    cout << "Position: " << tupleToString(testObject.position_vector) << endl; // << here only works with strings !
    cout << "Goal: " << tupleToString(testObject.goal_position) << endl;
    cout << "Force: " << to_string(result_forceApp) << endl;*/ 
}