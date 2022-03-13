//
// Created by Nicolas ISENGUERRE and Diego LANNOYE
//

/*!
 * \file Potential_Field_Forces_gr4.h
 * \brief Potential field header
 */


// include ctrlStruct !!!! Pour les coordonnées !

#include <cstdio>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <queue>

// The obstacles in the map will be represented as circles, just like the robot.
// Borders will be represented as being rectangles.
#define rho_0_border 0.1   // Distance of influence of border set to 10 [cm]. Every border has the same distance of influence.
#define rho_0_circle 0.15  // Distance of influence of obstacle in map : the samples on the floor have a diameter of 15 [cm], so we say r_influence = 15 [cm].
#define radius_robot 0.15  // Our robot is represented as a point in the middle of a circle of radius radius_robot, which is here 15 [cm].
#define PI 3.14159265
// Every obstacle in the map has the same distance of influence, as a first generalisation.



// ====================================================================================================================================================================================================================
// Obstacle Declaration
// ====================================================================================================================================================================================================================

class Obstacle
{

public :

    // Attributes : common to every obstacles : the repulsive coefficient and the distance of influence.
    double coeff{};
    double rho0{};
    std::string type;

    Obstacle();
    Obstacle(double k_rep, double distanceOfInfluence, std::string typeName);
    void setWeight (double newWeight);
    void setInfluence (double newInfluence);
};


// ====================================================================================================================================================================================================================
// SimpleBorder Declaration
// ====================================================================================================================================================================================================================

// Simple border permet le gestion de bordure horizontales ou verticales
class SimpleBorder : public Obstacle
{

public :

    // Attributes : those inherited from 'Obstacles'.
    //            : borderType (0 -> vertical, 1 -> horizontal)

    int         borderType{};
    double      position{};
    double      hitBox{}; // Permet de prendre en compte les bouts d'obstacles qui sont avancés sur le terrain.


    SimpleBorder();
    SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition, double hitBoxObstacle);

    double          computeDistance(std::tuple<double, double> robotPosition) const;

};


// ====================================================================================================================================================================================================================
// ObliqueBorder Declaration
// ====================================================================================================================================================================================================================

class ObliqueBorder : public Obstacle
{

public :


    int             borderType{};
    double          m{};
    double          p{};
    double          hitBox{};


    ObliqueBorder();
    ObliqueBorder(double k_rep, double distanceOfInfluence, int border_type, double slope, double offset, double hitBoxObstacle);

    double                      computeDistance(std::tuple<double, double> robotPosition) const;
    std::tuple <double, double> closestPoint(std::tuple <double, double> robotPosition) const;
};


// ====================================================================================================================================================================================================================
// Opponent Declaration
// ====================================================================================================================================================================================================================

class Opponent : public Obstacle
{

public:

    // Attributes : those inherited from 'Obstacle' and the position of the detected border (estimation of distance from the robot).
    std::tuple<double, double> position;
    Opponent(const std::tuple<double, double>& center, double k_rep, double distanceOfInfluence,  double hitBoxRadius);
    double hitBox{};

    double computeDistance(std::tuple<double, double> robotPosition);


    void setPositionOpponent(const std::tuple<double, double>& obstaclePosition);

};


// ====================================================================================================================================================================================================================
// Sample Declaration
// ====================================================================================================================================================================================================================

class Sample : public Obstacle
{

public :

    // Attributes : those inherited from 'Obstacle' and the position of its center (known).

    std::tuple <double, double>  position;
    double                  hitBox;
    Sample                  (const std::tuple <double, double>& center, double k_rep, double distanceOfInfluence, double hitboxRadius);

    double computeDistance  (std::tuple <double, double> robotPosition);

    // Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
    void setPositionSample  (const std::tuple <double, double>& obstaclePosition);
};


// ====================================================================================================================================================================================================================
// Goal Declaration
// ====================================================================================================================================================================================================================

class Goal
{

    public:

    std::tuple <double, double> position;

    double weight{};

    Goal();
    Goal(const std::tuple <double, double>& goal_position, double goalWeight);
    std::tuple <double, double> attForce(std::tuple <double, double> position_robot); // Goal list est dans le problème général, pas comme arg ici.
    
    double computeDistance(std::tuple <double, double> position_robot);
    bool goalReached(std::tuple <double, double> position_robot);

    void setWeight(double value);

};


// ====================================================================================================================================================================================================================
// Potential_Field Declaration
// ====================================================================================================================================================================================================================

class Potential_Field
{
public:
    //obstacle_list listOfObstacles;                        // List of all obstacles known at start.
    std::tuple <double, double> current_position;           // Current position of the robot.

    std::vector<SimpleBorder>   simpleBorderList;           // List of simple border obstacle type.
    std::vector<ObliqueBorder>    obliqueBorderList;        // List of oblique border obstacle type.
    std::vector<Opponent>       opponentList;               // List of opponent obstacle type.
    std::vector<Sample>         sampleList;                 // List of sample obstacle type.
    std::tuple <double, double> currentSpeedVector;         // Vref , omega_ref
    std::queue<double>          list_for_speed_filtering;

    double                      filter_output_Vref{};
    double                      filter_output_Wref{};

    std::vector< Goal >         listOfGoal;                 // List of goals.
    int                         numberOfGoals{};              // Self-explanatory.
    Goal                        currentGoal;                // Same.


    // constructors
    Potential_Field();
    Potential_Field(const std::tuple <double, double>& position);

    // communication with external structures  to update in the real project
    void setPosition(const std::tuple <double, double>& position);
    void setSpeedVector(const std::tuple <double, double>& initialSpeedVector);

    // update of obstacle's list
    void addSimpleBorder(const SimpleBorder&);
    void addObliqueBorder(const ObliqueBorder &object);
    void addOpponent(const Opponent&);
    void addSample(const Sample&);
    void removeSimpleBorder(int borderNumber);
    void removeObliqueBorder(int borderNumber);
    void removeSample();

    // search of value functions
    std::tuple <double, double> getPosition() const;
    // return True if the center of the robot is at a acceptable distance of the goal
    bool GoalTest(double precision);

    // return the repulsive value from the potential field
    std::tuple <double, double> totalRepulsiveForce();
    // return the attractive value from the potential field
    std::tuple <double, double> attractiveForce(std::tuple <double, double> position);
    // return the speed (Vref Wref) from the potential field forces modulated by the maximal wheel speed
    std::tuple <double, double> getSpeedVector(double dt, double vMax, double omegaMax, std::tuple <double, double> position);
    // filter in cpp : deactivated bc it create non-linearity
    std::tuple <double, double> speedFilter(std::tuple <double, double> speedVector);


    // Goal gestion
    void addGoal(const std::tuple <double, double>& newGoalPosition, double goalWeight);
    void removeGoal();
    void nextGoal(const std::vector<double>& weightSimpleBorder, const std::vector<double>& weightObliqueBorder, const std::vector<double>& weightSample);

};


// Global functions


// permet l'affichage des tuples
std::string tupleToString(std::tuple <double, double> entry);


Potential_Field initPotentialField();


// permet d'itérer le potential field à chaque pas de temps :
//  *   update the repulsive force
//  *   update the attaction force
//  *   update vRef and wRef

std::tuple<double,double> iterPotentialFieldWithLogFile(Potential_Field myPotential_Field, double dt, std::ofstream & myFile);
std::tuple<double,double> iterPotentialField(Potential_Field myPotential_Field, double dt);
