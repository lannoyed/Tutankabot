/*!
 * \file Potential_Field_Forces_gr4.cc
 * \brief File description : Potential field calculation (technique of path planning and obstacle avoidance). There are obstacles classes, etc.
 */

#include <fstream>
#include <utility>




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

Potential_Field::Potential_Field(const std::tuple <double, double>& position)
{
    current_position = position;

    numberOfGoals = 0;

    filter_output_Vref = 0.0;
    filter_output_Wref = 0.0;
    // 10 push pour faire une moyenne sur 5 éléments pour V et W
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);list_for_speed_filtering.push(0.0);
}

// Update function to get a new position.
void Potential_Field::setPosition(const std::tuple <double, double>& position){
    current_position = position;
}


bool Potential_Field::GoalTest(double precision){
    double distanceToGoal = currentGoal.computeDistance(current_position);
    if (distanceToGoal <= precision){
        return true;
    } return false;
};


void Potential_Field::setSpeedVerctor(const std::tuple <double, double>& initialSpeedVector){
    currentSpeedVector = initialSpeedVector;
}

// We put at the end of the list the new object.
void Potential_Field::addSimpleBorder(const SimpleBorder& object)
{
    simpleBorderList.push_back(object);
}

void Potential_Field::addOblicBorder(const OblicBorder& object)
{
    oblicBorderList.push_back(object);
}

void Potential_Field::addOpponent(const Opponent& object)
{
    opponentList.push_back(object);
}

void Potential_Field::addSample(const Sample& object)
{
    sampleList.push_back(object);
}




std::tuple <double, double> Potential_Field::totalRepulsiveForce()
{
    std::tuple <double, double> totalRepForce = std::make_tuple(0,0);

    // Iterate over every single type of obstacle.

    // SIMPLE BORDER.
    for(auto & obstacle : simpleBorderList)
    {
        // La realDistance = distance entre le centre du robot et la droite en elle-même.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        double position = obstacle.position;
        int type = obstacle.borderType;

        if ( (type == 0) && (realDistance <=  rho0_obstacle) )
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - position ) / realDistance );
        } else ( (type == 1) && (realDistance <=  rho0_obstacle) );
        {
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - position) / realDistance );
        }
    }

    // SAMPLES.
    for(auto & obstacle : sampleList)
    {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <=  rho0_obstacle)
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (position)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (position)) / realDistance );
        }
    }

    // OPPONENT.
    for(auto & obstacle : opponentList)
    {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <=  rho0_obstacle) 
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (position)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (position)) / realDistance );
        }
    }

    // OBLIC BORDERS.
    for(auto & obstacle : oblicBorderList)
    {
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> closestPoint = obstacle.closestPoint(current_position);

        if (realDistance <=  rho0_obstacle) 
        {
            std::get<0>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<0>(current_position) - std::get<0> (closestPoint)) / realDistance );
            std::get<1>(totalRepForce) += krepObstacle * ( ( 1 / realDistance) - ( 1 - rho0_obstacle) ) * ( 1 / distanceSquared ) * ( ( std::get<1>(current_position) - std::get<1> (closestPoint)) / realDistance );
        }
    }

    return totalRepForce;
}


// Cette fonction va servir à limiter la vitesse et à physiquement avoir un modèle cohérent.
std::tuple <double, double> Potential_Field::getSpeedVector(double dt, double vMax, double omegaMax, std::tuple <double, double> position){
    dt = dt;
    std::tuple <double, double> myAttractiveForce = attractiveForce(std::move(position));
    std::tuple <double, double> myRepulsiveForce = totalRepulsiveForce();
    std::tuple <double, double> nextSpeedVector = std::make_tuple(std::get<0>(myAttractiveForce) + std::get<0>(myRepulsiveForce),std::get<1>(myAttractiveForce) + std::get<1>(myRepulsiveForce));

    // calcul des normes des vecteurs
    double vRefNext = sqrt(pow(std::get<0>(nextSpeedVector),2) + pow(std::get<1>(nextSpeedVector), 2) );
    double vRef     = sqrt(pow(std::get<0>(currentSpeedVector),2) + pow(std::get<1>(currentSpeedVector), 2) );

    // cos theta = (u * v) / (|| u || * || v || )
    double cosTheta = (std::get<0>(nextSpeedVector) * std::get<0>(currentSpeedVector) + std::get<1>(nextSpeedVector) * std::get<1>(currentSpeedVector) ) / (vRefNext * vRef);
    //Signe = Xa * Yb - Ya * Xb
    double sinusSigne = std::get<0> (currentSpeedVector) * std::get<1>(nextSpeedVector) - std::get<1>(currentSpeedVector) * std::get<0>(nextSpeedVector);
    // signbit renvoie 1 si le nombre est negatif 0 sinon
    double S = 1 - 2 * std::signbit(sinusSigne);

    double omega;
    if (cosTheta >= 1.001) {throw;}
    if (cosTheta >= 1.0){
        omega =0.0;
    } else if (cosTheta <= -1.0){
        omega =  PI/dt;
    } else {
        omega = S * acos(cosTheta) / dt;
    }

    // Limiteur de vitesse : on ne peut pas aller à max vitesse tout droit et angulaire.
    if (omega >= omegaMax) {
        double dTheta = (omegaMax-omega)*dt;
        omega = omegaMax;
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector)* cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),std::get<0>(nextSpeedVector)* sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta) );
    } else if (omega <= - omegaMax){
        double dTheta = (omegaMax-omega)*dt;
        omega = -omegaMax;
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

// Filtre sur les vitesses pour être physiquement cohérent.
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





// Add a goal at the end of the list.
// À l'initialisation, défini que le premier goal = le premier de la liste.
void Potential_Field::addGoal(std::tuple <double, double> newGoalPosition, double goalWeight, double hitboxGoal)
{
    if(numberOfGoals == 0)
    {
        numberOfGoals += 1;
        listOfGoal.push_back(Goal(newGoalPosition, goalWeight, hitboxGoal));
        currentGoal = listOfGoal.at(0);
    }
    else
    {
        numberOfGoals += 1;
        listOfGoal.push_back(Goal(newGoalPosition, goalWeight, hitboxGoal));
    }
    
}

// Remove the first goal added.
void Potential_Field::removeGoal()
{
    numberOfGoals = numberOfGoals - 1;
    listOfGoal.erase(listOfGoal.begin());
}

// If the goal has been reached, we delete it from the list and set the new goal to the next one.
void Potential_Field::nextGoal()
{
    if (currentGoal.goalReached(current_position))
    {
        removeGoal();
        currentGoal = listOfGoal.at(0);
    }
}

// Did all the goals got visited ?
bool Potential_Field::areWeDone()
{
    return (numberOfGoals == 0);
}


// Calculation of the attractive force at the present point towards the goal. This gives back a vector.
std::tuple <double, double> Potential_Field::attractiveForce(std::tuple <double, double> position){
    return currentGoal.attForce(position);
}




// ====================================================================================================================================================================================================================
// Goal Class
// ====================================================================================================================================================================================================================

Goal::Goal()
{
    std::cout << "Default constructor of class 'Goal'" << std::endl;
}


Goal::Goal(const std::tuple<double, double>& goal_position, double goalWeight, double hitboxGoal)

{
    position = goal_position;   // Position du goal.
    weight = goalWeight;        // Poids du goal. Default weight : 0.0.
    hitbox = hitboxGoal;        // Permet de donner une distance de hit du goal.
}

std::tuple <double, double> Goal::attForce(std::tuple <double, double> position_robot)
{
    double co_X = -weight * ( std::get<0>(position_robot) - std::get<0>(position) );
    double co_Y = -weight * ( std::get<1>(position_robot) - std::get<1>(position) );
    std::tuple <double, double> forceAtt = std::make_tuple(co_X, co_Y);
    return forceAtt;
}


double Goal::computeDistance(std::tuple <double, double> robotPosition){
    return  sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) + pow( std::get<1>(robotPosition) - std::get<1>(position) , 2 ));
}


bool Goal::goalReached(std::tuple <double, double> position_robot)
{
    double distanceToGoal = computeDistance(position_robot);
    return (distanceToGoal <= 0.0);
}

// Permet de donner un poids au goal. Par défaut, le goal a un poids de 0. 
void Goal::setWeight(double value)
{
    weight = value;
}


// Set new goal : si on a atteint le goal (bool de return), on set un nouveau goal en passant au goal suivant. Faire une liste d'objets goals !



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
    type = std::move(typeName);
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
SimpleBorder::SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition, double hitboxObstacle)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    borderType = border_type;
    position = xoryposition;
    hitbox = hitboxObstacle;
    type = "SimpleBorder";
}

// Gives the euclidean distance (squared) between the line and the center of the robot.
double SimpleBorder::computeDistance (std::tuple <double, double> robotPosition) const
{
    // La distance = la distance entre le centre du robot et la droite, à laquelle on soustrait le rayon du robot (le modélisant) et la hitbox (qui définit la largeur réelle de l'obstacle).
    if (borderType == 0){
        // If we hit the obstacle, return 0 ! No negative distances !
        double distanceFinale = abs(std::get<0>(robotPosition) - position) - hitbox - radius_robot;
        if (distanceFinale <= 0.0)
        {
            return 0.0;
        };
        return distanceFinale;
    }
    else if (borderType == 1){
        double distanceFinale = abs(std::get<1>(robotPosition) - position) - hitbox - radius_robot; // Attention : hitbox et radius_robot hors de la valeur absolue.
        if (distanceFinale <= 0.0)
        {
            return 0.0;
        };
        return distanceFinale;
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
OblicBorder::OblicBorder(double k_rep, double distanceOfInfluence, int border_type, double pente, double offset, double hitboxObstacle)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    borderType = border_type;
    m = pente;
    p = offset;
    hitbox = hitboxObstacle;
    type = "OblicBorder";
}

// Give an approximation of the euclidean distance (squared) between the line and the center of the robot.
// The idea is that the euclidean distance will always be longer than the smallest distance to the x or the y.
double OblicBorder::computeDistance (std::tuple <double, double> robotPosition) const
{
    double y2; double x2; double pente_inverse; double b;

    pente_inverse = 1 / m ;
    b = std::get<1>(robotPosition) + pente_inverse * std::get<0>(robotPosition);
    x2 = (b - p) / (m + pente_inverse);
    y2 = b - pente_inverse * x2;
    double distanceFinale = sqrt(pow(std::get<0>(robotPosition) - x2, 2) + pow(std::get<1>(robotPosition) - y2, 2)) - hitbox - radius_robot;

    // If we hit the obstacle, return 0 ! No negative distances !
    if (distanceFinale <= 0.0)
    {
        return 0.0;
    };
    return distanceFinale;

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


// Attention : ça retourne le point le plus proche sur la droite qui modélise l'obstacle, pas au niveau de la hitbox !
std::tuple <double, double> OblicBorder::closestPoint(std::tuple <double, double> robotPosition) const

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

Opponent::Opponent(const std::tuple <double, double>& center, double k_rep, double distanceOfInfluence, double hitboxRadius)

{
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    type = "Opponent";
    hitbox = hitboxRadius;
}

// gérer si le résultat est négatif -> shouldn't be, though.
double Opponent::computeDistance (std::tuple <double, double> robotPosition){
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) + pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    double distanceObstacle = distanceToCenter - hitbox - radius_robot;
    if(distanceObstacle <= 0.0)
    {
        return 0.0;
    };
    return distanceObstacle;
}


// Harder part : to be implemented.
// The information we get is coming from the sensors. They will give us an estimation of the distance towards the obstacle.
// By having an idea of the smallest distance to the opponent and the angle at which it is, we can estimate the position of the opponent.
// Récuperer la position du robot et de ce qui est détecté dans les autres fichiers, comme ctrlStruct !
void Opponent::setPositionOpponent(const std::tuple <double, double>& obstaclePosition){
    position = obstaclePosition;
}



// ====================================================================================================================================================================================================================
// Sample Class
// ====================================================================================================================================================================================================================

// Parameterized constructor.

Sample::Sample(const std::tuple <double, double>& center, double k_rep, double distanceOfInfluence, double hitboxRadius)
{
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    hitbox = hitboxRadius;
    type = "Sample";
}

double Sample::computeDistance (std::tuple <double, double> robotPosition){
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) + pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    double distanceObstacle = distanceToCenter - hitbox - radius_robot;
    if(distanceObstacle <= 0.0)
    {
        return 0.0;
    };
    return distanceObstacle;
}

// Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
void Sample::setPositionSample(const std::tuple <double, double>& obstaclePosition){
    position = obstaclePosition;
}


// ====================================================================================================================================================================================================================
// Global Functions
// ====================================================================================================================================================================================================================

std::string tupleToString(std::tuple <double, double> entry){
    std::string output_s;
    output_s = std::to_string(std::get<0>(entry)) + " " + std::to_string(std::get<1>(entry));
    return output_s;
}

Potential_Field initPotentialField(){
    // création de la map

    // recuperation de la position après X secondes

    // récuperation du goal / des goals

    // récuperation de l'orientation du robot
    return Potential_Field();
}

std::tuple<double,double> iterPotentialFieldWithLogFile(Potential_Field myPotential_Field, double dt,     std::ofstream & myfile){
    if (myPotential_Field.GoalTest(0.08)){
        return std::make_tuple(0.0,0.0);
    } else {
        myPotential_Field.current_position = myPotential_Field.getPosition();
        std::tuple <double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple <double, double> attractionForce = myPotential_Field.attractiveForce(myPotential_Field.current_position);
        std::tuple <double, double> mySpeed = myPotential_Field.getSpeedVector(dt, 0.84, 4.6666, myPotential_Field.current_position);
        myfile << tupleToString(myPotential_Field.current_position) << " ";
        myfile << tupleToString(myRepulsiveForce) << " ";
        myfile << tupleToString(attractionForce) << " ";
        myfile << tupleToString(mySpeed) << " ";
        myfile << tupleToString(myPotential_Field.currentSpeedVector) << "\n";
        return mySpeed;
    }
}

std::tuple<double,double> iterPotentialField(Potential_Field myPotential_Field, double dt){
    if (myPotential_Field.GoalTest(0.08)){
        return std::make_tuple(0.0,0.0);
    } else {
        myPotential_Field.current_position = myPotential_Field.getPosition();
        std::tuple <double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple <double, double> attractionForce = myPotential_Field.attractiveForce(myPotential_Field.current_position);
        std::tuple <double, double> mySpeed = myPotential_Field.getSpeedVector(dt, 0.84, 4.6666, myPotential_Field.current_position);
        return mySpeed;
    }
}




































// À ne pas copier en 2002
// À faire passer dans un autre fichier -> main_ctrl_gr4

// ===============
// == TEST PART ==
// ===============





std::tuple<std::tuple<double, double>,std::tuple<double, double>> next_position(const Potential_Field& myPotential_Field, std::tuple<double, double> orientation,std::tuple<double,double> speed, double dt){
    std::tuple<double,double> position = myPotential_Field.current_position;
    double Wref = std::get<1>(speed);
    double Vref = std::get<0>(speed);
    double dTheta = Wref * dt;
    double norm = sqrt(pow(std::get<0>(orientation), 2) + pow(std::get<1>(orientation), 2));

    //std::cout << "norm = " << norm << " ";

    std::tuple<double, double> nextSpeedVector = std::make_tuple(std::get<0>(orientation)* cos(dTheta) - std::get<1>(orientation) * sin(dTheta),std::get<0>(orientation)* sin(dTheta) + std::get<1>(orientation) * cos(dTheta) );
    /*
    std::cout << "speed vector" << tupleToString(nextSpeedVector) << '\n';
    std::cout << "Wref = " << Wref << " Vref = " << Vref << '\n';
    */

    double next_x = 100 * Vref / norm * std::get<0>(nextSpeedVector) * dt + std::get<0>(position);
    double next_y = 100 * Vref / norm * std::get<1>(nextSpeedVector) * dt + std::get<1>(position);

    /*
    std::cout <<std::get<0>(position) << " " << std::get<1>(position) << "\n";
    std::cout << next_x << " " << next_y << "\n";
     */
    return std::make_tuple(std::make_tuple(next_x, next_y), nextSpeedVector);

}



std::tuple <double, double> Potential_Field::getPosition() const {
    return current_position;
};


// Test function : instantiate here to see if it works.
int main(int arg, char* argv[]){

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                      TEST OF PATH                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


    // création des obstacles
    double hitbox = 10;
    double distanceOfInfluence = 100;
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
    Goal goal = Goal(goal_position, 1.0, 0.15);
    std::tuple<double, double> position =  std::make_tuple<double, double>(-100,50);
    Potential_Field myPotential_Field = Potential_Field(position);
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

    std::ofstream myfile;
    myfile.open("data.txt", std::ios::out);
    if (!myfile){
        std::cout << "not open";
    }

    int number_of_sub_steps = 1;
    int i =0;

    while (i < 1000 && (pow(std::get<0>(myPotential_Field.currentGoal.position) - std::get<0>(myPotential_Field.current_position),2) > 8 || pow(std::get<1>(myPotential_Field.currentGoal.position) - std::get<1>(myPotential_Field.current_position),2) > 8) ) {


        std::tuple<double, double> mySpeed = iterPotentialFieldWithLogFile(myPotential_Field, dt, myfile);
        std::tuple<std::tuple<double, double>,std::tuple<double, double>> result = next_position(myPotential_Field, myPotential_Field.currentSpeedVector, mySpeed, dt / number_of_sub_steps);
        myPotential_Field.current_position = std::get<0>(result);
        myPotential_Field.currentSpeedVector = std::get<1>(result);


        /*
        std::tuple<double,double> precedentSpeedVector = myPotential_Field.currentSpeedVector;
        std::tuple <double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple <double, double> attractionForce = myPotential_Field.attractiveForce(myPotential_Field.current_position);
        std::tuple <double, double> mySpeed = myPotential_Field.getSpeedVector(dt, 0.84, 4.6666, myPotential_Field.current_position);
        //std::tuple <double, double> mySpeedFiltered = myPotential_Field.speedFilter(mySpeed);

        for (int j =0; j <= number_of_sub_steps - 1; j++) {
            std::tuple<std::tuple<double, double>,std::tuple<double, double>> result = next_position(myPotential_Field, precedentSpeedVector, mySpeed, dt / number_of_sub_steps);
            myPotential_Field.current_position = std::get<0>(result);
            myPotential_Field.currentSpeedVector = std::get<1>(result);
            precedentSpeedVector = myPotential_Field.currentSpeedVector;
            myfile << tupleToString(myPotential_Field.current_position) << " ";
            myfile << tupleToString(myRepulsiveForce) << " ";
            myfile << tupleToString(attractionForce) << " ";
            myfile << tupleToString(mySpeed) << " ";
            myfile << tupleToString(myPotential_Field.currentSpeedVector) << "\n";

        } */

        i++;

    }

    myfile.close();

    std::cout << "Finish";





    // To be modified if we want to test.
    /*
    double result_forceApp = testObject.forceApplied(valeurqcq);

    cout << "Test if everything works fine" << endl;
    cout << "Position: " << std::tupleToString(testObject.position_vector) << endl; // << here only works with strings !
    cout << "Goal: " << std::tupleToString(testObject.goal_position) << endl;
    cout << "Force: " << to_string(result_forceApp) << endl;
*/

}

