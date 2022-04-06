//
// Created by Insomniaque on 01-03-22.
//

/*!
 * \file Potential_Field_Forces_gr4.h
 * \brief Potential field header
 */

#ifndef _POTENTIAL_FIELD_FORCES_GR4_H_ // adapt it with the name of this file (header guard)
#define _POTENTIAL_FIELD_FORCES_GR4_H_ // must be the same name as the line before

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
#include <fstream>

#include "data.h"
#include "MainController.h"


NAMESPACE_INIT(ctrlGr4); // where X should be replaced by your group number


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
// Rectangle Declaration
// ====================================================================================================================================================================================================================


class Rectangle : public Obstacle
{
public:

    double hitBox;
    std::vector< std::tuple<double, double> > coordonnees; // Vecteur qui va contenir les points du rectangle.
    // On modélise un rectangle par ses 4 sommets (au minimum) et on peut rajouter des poids en fonction de la précision qu'on souhaite obtenir.

    Rectangle();
    Rectangle(double k_rep, double distanceOfInfluence, double hitBoxObstacle, std::vector< std::tuple<double, double> > listOfCoords);

    std::tuple< double, std::tuple<double,double> > computeDistance(std::tuple<double, double> robotPosition) const;
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

    double computeDistance(std::tuple<double, double> robotPosition) const;

};


// ====================================================================================================================================================================================================================
// OblicBorder Declaration
// ====================================================================================================================================================================================================================

class OblicBorder : public Obstacle
{

public :

    double          m{};
    double          p{};
    double          hitBox{};

    OblicBorder();
    OblicBorder(double k_rep, double distanceOfInfluence, double slope, double offset, double hitBoxObstacle);

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
    double hitBox{};

    Opponent(const std::tuple<double, double>& center, double k_rep, double distanceOfInfluence,  double hitBoxRadius);

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
    /*std::vector<double> weightSimpleBorder;
    std::vector<double> weightOblicBorder;
    std::vector<double> weightSample;
    std::vector<double> weightRectangles;*/

    // true if real goal ; false if passage goal 
    bool goalType; 
    double weight{};

    Goal();
    Goal(const std::tuple<double, double> &goal_position, double goalWeight, bool type);

    std::tuple <double, double> attForce(std::tuple <double, double> position_robot);

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
    std::tuple <double, double> coordonneesBase;            // Co de la base (si on est team bleue ou jaune)

    std::vector<SimpleBorder>   simpleBorderList;           // List of simple border obstacle type.
    std::vector<OblicBorder>    oblicBorderList;            // List of oblic border obstacle type.
    std::vector<Opponent>       opponentList;               // List of opponent obstacle type.
    std::vector<Sample>         sampleList;                 // List of sample obstacle type.
    std::vector<Rectangle>      rectangleList;              // List of rectangle obstacle type.
    std::tuple <double, double> currentSpeedVector;         // Vref , omega_ref
    std::queue<double>          list_for_speed_filtering;

    double                      filter_output_Vref{};
    double                      filter_output_Wref{};

    std::vector< Goal >         listOfGoal;                 // List of goals.
    int                         numberOfGoals{};              // Self-explanatory.
    Goal                        currentGoal;                // Same.
    bool                        isStuck{};
    int                         didntMove{};
    bool                        willNotMove{};
    int                         didntRotate{};



    // constructors
    Potential_Field();
    Potential_Field(const std::tuple <double, double>& position);

    // communication with external structures  to update in the real project
    void setPosition(const std::tuple <double, double>& position);
    void setSpeedVector(const std::tuple <double, double>& initialSpeedVector);
    void setSpeedVector(double theta);

    // update of obstacle's list
    void addSimpleBorder(const SimpleBorder &object);
    void addOblicBorder(const OblicBorder &object);
    void addOpponent(const Opponent &object);
    void addSample(const Sample &object);
    void addRectangle(const Rectangle &object);
    void removeSimpleBorder(int borderNumber);
    void removeOblicBorder(int borderNumber);
    void removeSample();

    // search of value functions
    std::tuple <double, double> getPosition() const;
    // return True if the center of the robot is at a acceptable distance of the goal
    bool GoalTest();

    // return the repulsive value from the potential field
    std::tuple <double, double> totalRepulsiveForce();

    // return the attractive value from the potential field
    std::tuple <double, double> attractiveForce(std::tuple <double, double> position);
    // return the speed (Vref Wref) from the potential field forces modulated by the maximal wheel speed
    std::tuple <double, double> getSpeedVector(double dt, double vMax, double omegaMax, std::tuple <double, double> position);
    // filter in cpp : deactivated bc it create non-linearity
    std::tuple <double, double> speedFilter(std::tuple <double, double> speedVector);


    // Goal gestion
    void addGoal(const std::tuple <double, double>& newGoalPosition, double goalWeight, bool type);
    void removeGoal();
    void addIntermediateGoal(); // Sets intermediate goals to avoid being stuck.
    void nextGoal(double newWeight);
    void nextGoalStuck(double newWeight);
    // If we are stuck or if we need to go to base
    void nextGoalBase(const std::tuple<double, double>& newPosition, double goalWeight);

    void goalStolenByOpponent(std::tuple<double, double> positionOpponent1Averaged, std::tuple<double, double> positionOpponent2Averaged);

    //global 
    void updatePotentialField(Controller *cvs);
    
};


// Global functions


// permet l'affichage des tuples
std::string tupleToString(std::tuple <double, double> entry);


Potential_Field initPotentialField();
void initGoals(Potential_Field * myPotential_Field, int teamNumber);

// permet d'itérer le potential field à chaque pas de temps :
//  *   update the repulsive force
//  *   update the attaction force
//  *   update vRef and wRef

std::tuple<double,double> iterPotentialFieldWithLogFile(Potential_Field * myPotential_Field, double dt, std::ofstream & myFile);
std::tuple<double,double> iterPotentialField(Potential_Field myPotential_Field, double dt);

std::tuple<double, double> Filter(std::tuple<double, double> speedVector,  std::deque<double>*  stack, double * ouput1_filtered, double *output2_filtered);


NAMESPACE_CLOSE();

// end of header guard
#endif
