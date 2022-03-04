/*!
 * \file Potential_Field_Forces_gr4.cc
 * \brief File description : Potential field calculation (technique of path planning and obstacle avoidance). There are obstacles classes, etc.
 */

#include <fstream>
using namespace std; 



# include "Potential_Field_Forces.h"

// Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.



/*          LISTE DES PROBLEMES
 *  où sont init les listes des différents obstacles ?
 *  mise en place de fonctions globales pour simplifier l'exécution dans la main
 *
 */



// ================
// == CLASS PART ==
// ================


// ====================================================================================================================================================================================================================
// Potential_Field Class
// ====================================================================================================================================================================================================================

// Default constructor.
Potential_Field::Potential_Field()
{
    std::cout << "The default constructor of 'Potential_Field' has been called." << std::endl; // endl = end_line => '\n' in Python.
    // cout means "character output" and wcout means "wide character output". std::cout format. Quick way of saying "print".
}

// Parameterized constructor.
Potential_Field::Potential_Field(std::tuple <double, double> position, Goal goal)
{
    current_position = position;
    current_goal = goal;
    filter_output_Vref = 0.0;
    filter_output_Wref = 0.0;
    // 10 push pour faire une moyenne sur 5 éléments pour V et W
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
}

// Update function to get a new position.
void Potential_Field::setPosition(std::tuple <double, double> position){
    current_position = position;
}

// Self explanatory function.
void Potential_Field::setGoal(std::tuple <double, double> position, double wieght){
    current_goal = Goal(position, wieght);
}

void Potential_Field::setSpeedVerctor(std::tuple <double, double> initialSpeedVector){
    currentSpeedVector = initialSpeedVector;
}

// Calculation of the attractive force at the present point towards the goal. This gives back a vector.
std::tuple <double, double> Potential_Field::attractiveForce(std::tuple <double, double> position){
    return current_goal.attForce(position);
}

std::tuple <double, double> Potential_Field::getSpeedVector(double dt, double vMax, double omegaMax, std::tuple <double, double> position){
    std::tuple <double, double> myAttractiveForce = attractiveForce(position);
    std::tuple <double, double> myRepulsiveForce = totalRepulsiveForce();
    std::tuple <double, double> nextSpeedVector = std::make_tuple(std::get<0>(myAttractiveForce) + std::get<0>(myRepulsiveForce),std::get<1>(myAttractiveForce) + std::get<1>(myRepulsiveForce));
    double vRefNext = sqrt(pow(std::get<0>(nextSpeedVector),2) + pow(std::get<1>(nextSpeedVector), 2) );
    double vRef     = sqrt(pow(std::get<0>(currentSpeedVector),2) + pow(std::get<1>(currentSpeedVector), 2) );
    double cosTheta = (std::get<0>(nextSpeedVector) * std::get<0>(currentSpeedVector) + std::get<1>(nextSpeedVector) * std::get<1>(currentSpeedVector) ) / (vRefNext * vRef);

    double omega;
    if (cosTheta >= 1.001) {throw;}
    if (cosTheta >= 1.0){
        omega =0.0;
    } else if (cosTheta == 0.0 ){
        omega = -PI/dt;
    }else {
        omega = acos(cosTheta) / dt;
    }

    if (omega >= omegaMax ) {
        double dTheta = (omegaMax-omega)*dt;
        omega = omegaMax;
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector)* cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),std::get<0>(nextSpeedVector)* sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta) );

    }
    double vMaxReal = vMax - 0.18 * std::abs(omega);
    if (vRefNext > vMaxReal){
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector)* vMaxReal/vRefNext, std::get<1>(nextSpeedVector)* vMaxReal/vRefNext);
        vRefNext = vMaxReal;
    }
    //cout << "cosTheta" << cosTheta << "\n";
    setSpeedVerctor(nextSpeedVector); // update de speedVector
    return std::make_tuple(vRefNext, omega);
}

// We put at the end of the list the new object.
void Potential_Field::addSimpleBorder(SimpleBorder object)
{
    simpleBorderList.push_back(object);
}

void Potential_Field::addOblicBorder(OblicBorder object)
{
    oblicBorderList.push_back(object);
}

void Potential_Field::addOpponent(Opponent object)
{
    opponentList.push_back(object);
}

void Potential_Field::addSample(Sample object)
{
    sampleList.push_back(object);
}

std::tuple <double, double> Potential_Field::totalRepulsiveForce()
{
    std::tuple <double, double> totalRepForce = std::make_tuple(0,0);

    // Iterate over every single type of obstacle.

    // HUGE TAKE CARE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // q_obstacle, dans la formule, c'est la distance entre le robot et la zone d'influence, en fait. Il faut considérer la zone d'influence et pas le centre de l'obstacle.
    // En fait, jsp, demander.

    // SIMPLE BORDER.
    for(auto & obstacle : simpleBorderList)
    {
        // La realDistance = distance entre le centre du robot et la droite en elle-même.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        double distanceToObstacleInfluenceZone = realDistance - rho0_obstacle;
        double position = obstacle.position;
        int type = obstacle.borderType;

        // 
        if ( (type == 0) && (realDistance <=  rho0_obstacle + radius_robot) )
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - position) / realDistance );
        } else ( (type == 1) && (realDistance <=  rho0_obstacle + radius_robot) );
        {
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - position) / realDistance );
        }
    }

    for(auto & obstacle : sampleList)
    {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <=  rho0_obstacle + radius_robot) 
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (position)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (position)) / realDistance );
        }
    }

    for(auto & obstacle : opponentList)
    {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <=  rho0_obstacle + radius_robot) 
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (position)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (position)) / realDistance );
        }
    }

    for(auto & obstacle : oblicBorderList)
    {
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> closestPoint = obstacle.closestPoint(current_position);

        if (realDistance <=  rho0_obstacle + radius_robot) 
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (closestPoint)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (closestPoint)) / realDistance );
        }
    }


    return totalRepForce;
}

std::tuple <double, double> Potential_Field::speedFilter(std::tuple <double, double> speedVector){
    double V = std::get<0>(speedVector)/10.0;
    double W = std::get<1>(speedVector)/10.0;
    list_for_speed_filtering.push(V);
    list_for_speed_filtering.push(W);
    filter_output_Vref += V - list_for_speed_filtering.front();
    list_for_speed_filtering.pop();
    filter_output_Wref += W - list_for_speed_filtering.front();
    list_for_speed_filtering.pop();
    return std::make_tuple(filter_output_Vref, filter_output_Wref);
}



// ====================================================================================================================================================================================================================
// Goal Class
// ====================================================================================================================================================================================================================

Goal::Goal()
{
    std::cout << "Default constructor of class 'Goal'" << std::endl;
}

Goal::Goal(std::tuple<double, double> goal_position, double goalWeight)
{
    position = goal_position;
    Weight = goalWeight;
}

std::tuple <double, double> Goal::attForce(std::tuple <double, double> position_robot)
{
    double co_X = -Weight * ( std::get<0>(position_robot) - std::get<0>(position) );
    double co_Y = -Weight * ( std::get<1>(position_robot) - std::get<1>(position) );
    std::tuple <double, double> forceAtt = std::make_tuple(co_X, co_Y);
    return forceAtt;
}



// ====================================================================================================================================================================================================================
// Obstacle Class
// ====================================================================================================================================================================================================================

Obstacle::Obstacle()
{
    std::cout << "Default constructor of class 'Obstacle'" << std::endl;
}

// Class constructor. Parameterized constructor : the position of the center is not specified, as a border is represented as a line. Use "setPosition" for other obstacles.
Obstacle::Obstacle(double k_rep, double distanceOfInfluence, std::string typeName)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    type = typeName;
}

void Obstacle::setWeight (double newWeight) {
    coeff = newWeight;
}

void Obstacle::setInfluence (double  newInfluence){
    rho0 = newInfluence;
}



// ====================================================================================================================================================================================================================
// SimpleBorder Class
// ====================================================================================================================================================================================================================

// Default constructor.
SimpleBorder::SimpleBorder()
{
    std::cout << "Default constructor of class 'Border'" << std::endl;
}

// Parameterized constructor.
SimpleBorder::SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    borderType = border_type;
    position = xoryposition;
    type = "SimpleBorder";
}

// Gives the euclidean distance (squared) between the line and the center of the robot.
double SimpleBorder::computeDistance (std::tuple <double, double> robotPosition)
{
    if (borderType == 0){
        return abs(std::get<0>(robotPosition) - position);
    }
    else if (borderType == 1){
        return  abs(std::get<1>(robotPosition) - position);
    }
    else {
        throw "invalid Border type";
    }
}



// ====================================================================================================================================================================================================================
// OblicBorder Class
// ====================================================================================================================================================================================================================

// Default constructor.
OblicBorder::OblicBorder()
{
    std::cout << "Default constructor of class 'Border'" << std::endl;
}

// Parameterized constructor.
OblicBorder::OblicBorder(double k_rep, double distanceOfInfluence, int border_type, double pente, double offset)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    borderType = border_type;
    m = pente;
    p = offset;
    type="OblicBorder";
}

// Give an approximation of the euclidean distance (squared) between the line and the center of the robot.
// The idea is that the euclidean distance will always be longer than the smallest distance to the x or the y.
double OblicBorder::computeDistance (std::tuple <double, double> robotPosition)
{
    double y2; double x2; double pente_inverse; double b;

    pente_inverse = 1 / m ;
    b = std::get<1>(robotPosition) + pente_inverse * std::get<0>(robotPosition);
    x2 = (b - p) / (m + pente_inverse);
    y2 = b - pente_inverse * x2;

    return sqrt(pow(std::get<0>(robotPosition) - x2, 2) + pow(std::get<1>(robotPosition) - y2, 2));

    /*double yup; double xup; double distance1; double distance2;

    yup = m * std::get<0>(robotPosition) + p;
    xup = (std::get<1>(robotPosition) - p) / m;

    distance1 = abs(std::get<0>(robotPosition) - xup);
    distance2 = abs(std::get<1>(robotPosition) - yup);

    if (distance1 < distance2){
        return distance1;
    }
    else {
        return distance2;
    }*/
}

std::tuple <double, double> OblicBorder::closestPoint(std::tuple <double, double> robotPosition)
{
    double y2; double x2; double pente_inverse; double b;

    pente_inverse = 1 / m ;
    b = std::get<1>(robotPosition) + pente_inverse * std::get<0>(robotPosition);
    x2 = (b - p) / (m + pente_inverse);
    y2 = b - pente_inverse * x2;

    std::tuple <double, double> outputTupple = std::make_tuple(x2,y2);
    return outputTupple;
}



// ====================================================================================================================================================================================================================
// Opponent Class
// ====================================================================================================================================================================================================================

// Parameterized constructor.
Opponent::Opponent(std::tuple <double, double> center, double k_rep, double distanceOfInfluence)
{
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    type = "Opponent";
}

// gérer si le résultat est négatif -> shouldn't be, tho.
double Opponent::computeDistance (std::tuple <double, double> robotPosition){
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) + pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    return distanceToCenter - hitBoxRadius;
}


// Harder part : to be implemented.
// The information we get is coming from the sensors. They will give us an estimation of the distance towards the obstacle.
// By having an idea of the smallest distance to the opponent and the angle at which it is, we can estimate the position of the opponent.
// Récuperer la position du robot et de ce qui est détecté dans les autres fichiers, comme ctrlStruct !
void Opponent::setPositionOpponent(std::tuple <double, double> obstaclePosition){
    position = obstaclePosition;
}



// ====================================================================================================================================================================================================================
// Sample Class
// ====================================================================================================================================================================================================================

// Parameterized constructor.
Sample::Sample(std::tuple <double, double> center, double k_rep, double distanceOfInfluence, double hibox)
{
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    hitBoxRadius = hibox;
    type = "Sample";
}

double Sample::computeDistance (std::tuple <double, double> robotPosition){
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) + pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    return distanceToCenter - hitBoxRadius;
}

// Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
void Sample::setPositionSample(std::tuple <double, double> obstaclePosition){
    position = obstaclePosition;
}










































// À ne pas copier en 2002
// À faire passer dans un autre fichier -> main_ctrl_gr4

// ===============
// == TEST PART ==
// ===============

string tupleToString(std::tuple <double, double> entry){
    string output_s;
    output_s = to_string(get<0>(entry)) + " " + to_string(get<1>(entry));
    return output_s;
}



std::tuple<double,double> next_position(std::tuple<double,double> position, std::tuple<double,double> speed){
    double dt = 0.01;
    double next_x = 100 * std::get<0>(speed) * dt + std::get<0>(position);
    double next_y = 100 * std::get<1>(speed) * dt + std::get<1>(position);
    return std::make_tuple(next_x, next_y);
}






// Test function : instantiate here to see if it works.
int main(int arg, char* argv[]){

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                      TEST OF PATH                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


    // création des obstacles
    double hitbox = 15;
    double distanceOfInfluence = 30;
    std::tuple <double, double> center = std::make_tuple<double, double>(0.0, 145);
    double k_rep = 10;

    Sample Sample1 = Sample(center, k_rep, distanceOfInfluence, hitbox);



    center = std::make_tuple<double, double>(-60.0, 100.0);
    Sample Sample2 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 120.0);
    Sample Sample3 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 80.0);
    Sample Sample4 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 125.0);
    Sample Sample5 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 130.0);
    Sample Sample6 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 135.0);
    Sample Sample7 = Sample(center, k_rep, distanceOfInfluence, hitbox);

    center = std::make_tuple<double, double>(0.0, 140.0);
    Sample Sample8 = Sample(center, k_rep, distanceOfInfluence, hitbox);


    // creation potential field :
    std::tuple<double, double> goal_position     =  std::make_tuple<double, double>(50, 200);
    Goal goal = Goal(goal_position, 1);
    std::tuple<double, double> position =  std::make_tuple<double, double>(-100,50);
    Potential_Field myPotential_Field = Potential_Field(position, goal);
    std::tuple<double, double> initialSpeedVector =  std::make_tuple<double, double>(1.0,0.0);
    myPotential_Field.setSpeedVerctor(initialSpeedVector);


    // créatrion des listes :


    std::vector<SimpleBorder>   simpleBorderList;   // List of simple border obstacle type.
    std::vector<OblicBorder>    oblicBorderList;    // List of oblic border obstacle type.
    std::vector<Opponent>       opponentList;       // List of opponent obstacle type.
    std::vector<Sample>         sampleList;         // List of sample obstacle type.

    myPotential_Field.addSample(Sample1);
    myPotential_Field.addSample(Sample2);
    /*
    myPotential_Field.addSample(Sample3);
    myPotential_Field.addSample(Sample4);
     */
    /*
    myPotential_Field.addSample(Sample5);
    myPotential_Field.addSample(Sample6);
    myPotential_Field.addSample(Sample7);
    myPotential_Field.addSample(Sample8);
     */
    double dt = 0.1;

    ofstream myfile;
    myfile.open("data.txt", ios::out);
    if (!myfile){
        cout << "not open";
    }

    int i =0;
    while (i < 1000 && (pow(std::get<0>(myPotential_Field.current_goal.position) - std::get<0>(myPotential_Field.current_position),2) > 0.01 || pow(std::get<1>(myPotential_Field.current_goal.position) - std::get<1>(myPotential_Field.current_position),2) > 00.1) ) {
        myfile << tupleToString(myPotential_Field.current_position) << " ";
        std::tuple<double,double> precedentSpeedVector = myPotential_Field.currentSpeedVector;
        std::tuple <double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple <double, double> attractionForce = myPotential_Field.attractiveForce(myPotential_Field.current_position);
        myfile << tupleToString(myRepulsiveForce) << " ";
        myfile << tupleToString(attractionForce) << " ";
        std::tuple <double, double> mySpeed = myPotential_Field.getSpeedVector(dt, 0.84, 4.666, myPotential_Field.current_position);
        std::tuple <double, double> mySpeedFiltered = myPotential_Field.speedFilter(mySpeed);
        myfile << tupleToString(mySpeed) << "\n";
        myPotential_Field.current_position =  next_position(myPotential_Field.current_position, myPotential_Field.currentSpeedVector);
        i++;
    }
    myfile.close();







    // To be modified if we want to test.
    /*
    double result_forceApp = testObject.forceApplied(valeurqcq);

    cout << "Test if everything works fine" << endl;
    cout << "Position: " << std::tupleToString(testObject.position_vector) << endl; // << here only works with strings !
    cout << "Goal: " << std::tupleToString(testObject.goal_position) << endl;
    cout << "Force: " << to_string(result_forceApp) << endl;
*/

}

