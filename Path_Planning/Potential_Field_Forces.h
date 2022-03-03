//
// Created by Insomniaque on 01-03-22.
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



class Obstacle
{

public :

    // Attributes : common to every obstacles : the repulsive coefficient and the distance of influence.
    double coeff;
    double rho0;
    std::string type;

    Obstacle();
    Obstacle(double k_rep, double distanceOfInfluence, std::string typeName);
    void setWeight (double newWeight);
    void setInfluence (double  newInfluence);


};


// Simple border permet le gestion de bordure horizontales ou verticales
class SimpleBorder : public Obstacle
{

public :
    // Attributes : those inhehirted from 'Obstacles'.
    //            : borderType (0 -> vertical, 1 -> horizontal)
    int             borderType;
    double          position;

    SimpleBorder();
    SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition);

    double          computeDistance(std::tuple<double, double> robotPosition);

};


class OblicBorder : public Obstacle
{
public :

    int             borderType;
    double          m;
    double          p;

    OblicBorder();
    OblicBorder(double k_rep, double distanceOfInfluence, int border_type, double pente, double offset);

    double          computeDistance(std::tuple<double, double> robotPosition);
};


class Opponent : public Obstacle
{

public:
    // Attributes : those inherited from 'Obstacle' and the position of the detected border (estimation of distance from the robot).
    std::tuple<double, double> position;
    double hitBoxRadius;

    Opponent(std::tuple<double, double> center, double k_rep, double distanceOfInfluence);


    double computeDistance(std::tuple<double, double> robotPosition);

    void setPositionOpponent(std::tuple<double, double> obstaclePosition);
};


class Sample : public Obstacle
{

public :
    // Attributes : those inherited from 'Obstacle' and the position of its center (known).
    std::tuple <double, double>  position;
    double                  hitBoxRadius;
    Sample                  (std::tuple <double, double> center, double k_rep, double distanceOfInfluence, double hibox);
    double computeDistance  (std::tuple <double, double> robotPosition);

    // Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
    void setPositionSample  (std::tuple <double, double> obstaclePosition);
};


// Plus forcément très utile.

// typedef std::vector< Obstacle > obstacle_list; // List of coordinates as tuples.
// Vector : basically works as a list. vector.push_back : add element at the end.

class Goal
{
public:
    std::tuple <double, double> position;
    double Weight;
    Goal();
    Goal(std::tuple <double, double> goal_position, double goalWeight);
};

class Potential_Field
{
public:
    //obstacle_list listOfObstacles;                  // List of all obstacles known at start.
    std::tuple <double, double> current_position;   // Current position of the robot.
    Goal                        current_goal;      // Current goal position.
    std::vector<SimpleBorder>   simpleBorderList;   // List of simple border obstacle type.
    std::vector<OblicBorder>    oblicBorderList;    // List of oblic border obstacle type.
    std::vector<Opponent>       opponentList;       // List of opponent obstacle type.
    std::vector<Sample>         sampleList;         // List of sample obstacle type.
    std::tuple <double, double> currentSpeedVector; // Vref , omega_ref
    std::queue<double>          list_for_speed_filtering;
    double                      filter_output_Vref;
    double                      filter_output_Wref;



    Potential_Field();
    Potential_Field(std::tuple <double, double> position, Goal goal);
    void setPosition(std::tuple <double, double> position);
    void setGoal(std::tuple <double, double> position, double weight);
    void setSpeedVerctor(std::tuple <double, double> initialSpeedVector);
    std::tuple <double, double> attractiveForce();

    void addSimpleBorder(SimpleBorder);
    void addOblicBorder(OblicBorder);
    void addOpponent(Opponent);
    void addSample(Sample);

    std::tuple <double, double> totalRepulsiveForce();

    std::tuple <double, double> getSpeedVector(double dt, double vMax, double omegaMax);
    std::tuple <double, double> speedFilter(std::tuple <double, double> speedVector);


};


