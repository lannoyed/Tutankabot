/*!
 * \file Potential_Field_Forces_gr4.cc
 * \brief File description : Potential field calculation (technique of path planning and obstacle avoidance). There are obstacles classes, etc.
 */

# include "Potential_Field_Forces_gr4.h"

// Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.




// Default constructor.
Potential_Field::Potential_Field() {

}

// Parameterized constructor.

Potential_Field::Potential_Field(const std::tuple<double, double> &position) {
    current_position = position;
    numberOfGoals = 0;
}

// Update function to get a new position.
void Potential_Field::setPosition(const std::tuple<double, double> &position) {
    if ( fabs(std::get<0>(current_position) - std::get<0>(position)) <minimumPositionStep && fabs(std::get<1>(current_position) - std::get<1>(position)) <minimumPositionStep ){
        didntMove += 1;
    }else{
        didntMove = 0;
    }
    std::cout<< std::get<0>(position) << " " << std::get<1>(position) <<"\n"; 
    current_position = position;
}


bool Potential_Field::GoalTest() {
    double distanceToGoal = currentGoal.computeDistance(current_position);
    if (distanceToGoal <= precision) {
        return true;
    }
    return false;
}


void Potential_Field::setSpeedVector(const std::tuple<double, double> &initialSpeedVector) {
    currentSpeedVector = initialSpeedVector;
}

void Potential_Field::setSpeedVector(const double theta) {

    if ( fabs(std::get<0>( currentSpeedVector) - cos(theta))  < minimumAngleStep &&  fabs(std::get<1>( currentSpeedVector) - sin(theta) ) < minimumAngleStep ) {
        didntRotate += 1;
    }else{
        didntRotate = 0;
    }
    currentSpeedVector = std::make_tuple(cos(theta), sin(theta));

}

// We put at the end of the list the new object.
void Potential_Field::addSimpleBorder(const SimpleBorder &object) {
    simpleBorderList.push_back(object);
}

void Potential_Field::addOblicBorder(const OblicBorder &object) {
    oblicBorderList.push_back(object);
}

void Potential_Field::addOpponent(const Opponent &object) {
    opponentList.push_back(object);
}

void Potential_Field::addSample(const Sample &object) {
    sampleList.push_back(object);
}

void Potential_Field::addRectangle(const Rectangle &object) {
    rectangleList.push_back(object);
}


std::tuple<double, double> Potential_Field::totalRepulsiveForce() {

    double totalRepForceX = 0.0;
    double totalRepForceY = 0.0;

    //int i = 0;

    // Iterate over every single type of obstacle.

    // SIMPLE BORDER.
    for (auto &obstacle : simpleBorderList) {
        // La realDistance = distance entre le centre du robot et la droite en elle-même.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        double position = obstacle.position;
        int type = obstacle.borderType;

        if ((type == 0) && (realDistance <= rho0_obstacle)) {
            totalRepForceX +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<0>(current_position) - position) / realDistance);

        } else if ((type == 1) && (realDistance <= rho0_obstacle))
        {
            totalRepForceY +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<1>(current_position) - position) / realDistance);
        }

        //i++;
    }

    // SAMPLES.
    for (auto &obstacle : sampleList) {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <= rho0_obstacle) {
            totalRepForceX +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<0>(current_position) - std::get<0>(position)) / realDistance);
            totalRepForceY +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<1>(current_position) - std::get<1>(position)) / realDistance);
        }
    }

    // OPPONENT.
    for (auto &obstacle : opponentList) {
        // Square of the euclidean distance + real euclidean distance.
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> position = obstacle.position;
        if (realDistance <= rho0_obstacle) {
            totalRepForceX +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<0>(current_position) - std::get<0>(position)) / realDistance);
            totalRepForceY +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<1>(current_position) - std::get<1>(position)) / realDistance);
        }
    }

    // OBLIC BORDERS.
    for (auto &obstacle : oblicBorderList) {
        double realDistance = obstacle.computeDistance(current_position);
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;
        std::tuple<double, double> closestPoint = obstacle.closestPoint(current_position);

        if (realDistance <= rho0_obstacle) {
            totalRepForceX +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<0>(current_position) - std::get<0>(closestPoint)) / realDistance);
            totalRepForceY +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<1>(current_position) - std::get<1>(closestPoint)) / realDistance);
        }
    }

    // RECTANGLES.
    for (auto &obstacle : rectangleList) {
        // La realDistance = distance entre le centre du robot et le point le plus proche sur le rectangle.
        double realDistance = std::get<0>(obstacle.computeDistance(current_position));
        std::tuple<double,double> closestPoint = std::get<1>(obstacle.computeDistance(current_position));
        double distanceSquared = pow(realDistance, 2);
        double krepObstacle = obstacle.coeff;
        double rho0_obstacle = obstacle.rho0;

        if (realDistance <= rho0_obstacle) {
            totalRepForceX +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<0>(current_position) - std::get<0>(closestPoint)) / realDistance);
            totalRepForceY +=
                    krepObstacle * ((1.0 / realDistance) - (1.0 / rho0_obstacle)) * (1.0 / distanceSquared) *
                    ((std::get<1>(current_position) - std::get<1>(closestPoint)) / realDistance);
        }
    }

    // no failures check
    if (totalRepForceX > 10e5 ){
        totalRepForceX = 10e5;
    }else if (totalRepForceX < -10e5 ){
        totalRepForceX = -10e5;
    }

    if (totalRepForceY > 10e5 ){
        totalRepForceY = 10e5;
    }else if (totalRepForceY < -10e5 ){
        totalRepForceY = -10e5;
    }

    if (std::isnan(totalRepForceX ) || std::isnan(totalRepForceY) ) {
        std::cout << "nan value for position :" << tupleToString(current_position) << "\n for goal :" << tupleToString(currentGoal.position) << "and weight of : " << currentGoal.weight  << "\n"; 
        return std::make_tuple(0.0,0.0);
    }


    std::tuple<double, double> totalRepForce = std::make_tuple(totalRepForceX, totalRepForceY);

    if (sqrt( pow(totalRepForceX, 2) + pow(totalRepForceY, 2)) > LIMIT_REPULSIVE_FORCE)
    {
        // On normalise puis on retourne la valeur.
        double newValueX = (totalRepForceX / sqrt( pow(totalRepForceX, 2) + pow(totalRepForceY, 2))) * LIMIT_REPULSIVE_FORCE;
        double newValueY = (totalRepForceY / sqrt( pow(totalRepForceX, 2) + pow(totalRepForceY, 2))) * LIMIT_REPULSIVE_FORCE;
        totalRepForce = std::make_tuple(newValueX, newValueY);
    }

    return totalRepForce;
}

// Convention : il y a 4 simpleBorder. Il faut bien faire attention aux conventions de coordonnées. Ils sont définis dans cet ordre :
// 0: bord bas ; 1: bord gauche ; 2: bord haut ; 3: bord droit.
// On annule son effet, tout bonnement. On pourra le remettre après.
void Potential_Field::removeSimpleBorder(int borderNumber) {
    simpleBorderList.at(borderNumber).setWeight(0.0);
}

// Convention : il y a 2 oblicBorder. Ils sont définis dans cet ordre :
// 0: bord en bas à droite ; 1: bord en haut à droite.
// On annule son effet, tout bonnement. On pourra le remettre après.
void Potential_Field::removeOblicBorder(int borderNumber) {
    oblicBorderList.at(borderNumber).setWeight(0.0);
}

// Convention : on va encoder les sample dans l'ordre dans lequel on compte les exploiter comme Goal.
// Donc : dès qu'on veut aller vers un sample, on retire celui-ci de la liste.
// Et comme c'est le même ordre, il suffit de retirer le premier à chaque fois !
void Potential_Field::removeSample() {
    sampleList.pop_back();
}


// Cette fonction va servir à limiter la vitesse et à physiquement avoir un modèle cohérent.
std::tuple<double, double>
Potential_Field::getSpeedVector(double dt, double vMax, double omegaMax, std::tuple<double, double> position) {
    dt = dt;
    std::tuple<double, double> myAttractiveForce = attractiveForce(std::move(position));
    std::tuple<double, double> myRepulsiveForce = totalRepulsiveForce();
    std::tuple<double, double> nextSpeedVector = std::make_tuple(
            std::get<0>(myAttractiveForce) + std::get<0>(myRepulsiveForce),
            std::get<1>(myAttractiveForce) + std::get<1>(myRepulsiveForce));

    // calcul des normes des vecteurs
    double vRefNext = sqrt(pow(std::get<0>(nextSpeedVector), 2) + pow(std::get<1>(nextSpeedVector), 2));
    double vRef = sqrt(pow(std::get<0>(currentSpeedVector), 2) + pow(std::get<1>(currentSpeedVector), 2));

    // cos theta = (u * v) / (|| u || * || v || )
    double cosTheta = (std::get<0>(nextSpeedVector) * std::get<0>(currentSpeedVector) +
                       std::get<1>(nextSpeedVector) * std::get<1>(currentSpeedVector)) / (vRefNext * vRef);
    //Signe = Xa * Yb - Ya * Xb
    double sinusSigne = std::get<0>(currentSpeedVector) * std::get<1>(nextSpeedVector) -
                        std::get<1>(currentSpeedVector) * std::get<0>(nextSpeedVector);
    // signbit renvoie 1 si le nombre est negatif 0 sinon
    double S = 1.0 - 2.0 * std::signbit(sinusSigne);

    double omega;
    if (cosTheta >= 1.001) { throw; }
    if (cosTheta >= 1.0) {
        omega = 0.0;
    } else if (cosTheta <= -1.0) {
        omega = M_PI / dt;
    } else {
        omega = S * acos(cosTheta) / dt;
    }
    
    // Limiteur de vitesse : on ne peut pas aller à max vitesse tout droit et angulaire.
    if (omega >= omegaMax) {
        double dTheta = (omegaMax - omega) * dt;
        omega = omegaMax;
        // nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector) * cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),std::get<0>(nextSpeedVector) * sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta));
    } else if (omega <= -omegaMax) {
        double dTheta = (omegaMax - omega) * dt;
        omega = -omegaMax;
        // nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector) * cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),std::get<0>(nextSpeedVector) * sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta));
    }

    double vMaxReal = vMax; //*  ( 1 - std::fabs(omega) / omegaMax) ;
    //vRefNext = vRefNext / 7.5 * 0.33;   
    if (vRefNext > vMaxReal) {
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector) * vMaxReal / vRefNext,
                                          std::get<1>(nextSpeedVector) * vMaxReal / vRefNext);
        vRefNext = vMaxReal;
    }

    /*double norm = currentGoal.computeDistance(position);
    if(norm >= 0.15 && norm < 0.25)
    {
        currentGoal.setWeight(10.0);
    }
    if(norm > 0.08 && norm < 0.15)
    {
        currentGoal.setWeight(5.0);
    }*/

    //cout << "cosTheta" << cosTheta << "\n";
    //setSpeedVector(nextSpeedVector); // update de speedVector
    return std::make_tuple(vRefNext, omega);
}

// Filtre sur les vitesses pour être physiquement cohérent.
std::tuple<double, double> Potential_Field::speedFilter(std::tuple<double, double> speedVector) {
    double V = std::get<0>(speedVector) ;
    double W = std::get<1>(speedVector) ;
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
void Potential_Field::addGoal(const std::tuple<double, double>& newGoalPosition, double goalWeight, bool type) {
    if (numberOfGoals == 0) {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight, type));
        currentGoal = listOfGoal.back();
    } else {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight, type));
        currentGoal = listOfGoal.back();
    }
    std::cout << "Added a goal to return to base at : (" << std::get<0>(newGoalPosition) << "," << std::get<1>(newGoalPosition) << ").\n";

}

// Remove the first goal added.
void Potential_Field::removeGoal() {
    numberOfGoals = numberOfGoals - 1;
    listOfGoal.pop_back();
}


void Potential_Field::addIntermediateGoal()
{
    // Quandrant 1
    double robotPositionX = std::get<0>(current_position);
    double robotPositionY = std::get<1>(current_position);
    double goalX = std::get<0>(currentGoal.position);
    double goalY = std::get<1>(currentGoal.position);

    if(currentGoal.goalType == false)
    {
        return;
    }
    if ( ( (robotPositionX >= 1.3 && robotPositionY >= 2.30) && (goalX < 1.21 && goalY > 2.30) ) || ( (robotPositionX <= 1.21 && robotPositionY >= 2.3) && (goalX > 1.3 && goalY > 2.30) ) )
    {

        std::tuple<double, double> newGoalPosition = std::make_tuple(1.27, 2.55);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
    }

    // On est dans le Z3 à Z4 .
    else if ( ( (robotPositionX >= 1.3 && robotPositionY <= 0.70) && (goalX < 1.21 && goalY < 0.70) ) || ( (robotPositionX <= 1.21 && robotPositionY <= 0.70) && (goalX > 1.3 && goalY <= 0.70) ) )
    {

        std::tuple<double, double> newGoalPosition = std::make_tuple(1.27, 0.45);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back();
        numberOfGoals +=1; 
    }
    // De Z8 à Z9
    else if (( (robotPositionX <= 0.55) && (robotPositionY >= 1.5) && (robotPositionY <= 2.28721) ) && ((goalX <=  0.55) && (goalY <= 1.5) && (goalY >= 0.8037) )){
        std::tuple<double, double> newGoalPosition = std::make_tuple(0.65, 1.35);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;

        newGoalPosition = std::make_tuple(0.65, 1.65);
        weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
    }
    // De Z9 à Z8 
    else if (( (goalX <= 0.55) && (goalY >= 1.5) && (goalY <= 2.28721) ) && ( (robotPositionX <=  0.55) && (robotPositionY <= 1.5) && (robotPositionY >= 0.8037) ) ){
        
        std::tuple<double, double> newGoalPosition = std::make_tuple(0.65, 1.65);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
        
        newGoalPosition =  std::make_tuple(0.65, 1.35);
        weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
    }
    // Z6 à Z5 
    else if ( ( (robotPositionX <= 0.85 && robotPositionY >= 1.5 + precision/2 ) && (goalX <= 0.85 && goalY < 1.5 - precision/2) ) )  
    {
        std::tuple<double, double> newGoalPosition = std::make_tuple(0.6, 1.3);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
    }
    // Z5 à Z6
    else if ( ( (robotPositionX <= 0.85 && robotPositionY <= 1.5 - precision/2 ) && (goalX <= 0.85 && goalY > 1.5 + precision / 2) ) )
    {
        std::tuple<double, double> newGoalPosition = std::make_tuple(0.6, 1.7);
        double weight = currentGoal.weight;
        currentGoal.setWeight(0.0);
        listOfGoal.push_back(Goal(newGoalPosition, weight, false));
        currentGoal = listOfGoal.back(); 
        numberOfGoals +=1;
    }
}


// If the goal has been reached, we set the new goal to the next one.
void Potential_Field::nextGoal(double newWeight) {
    // Next goal : appelée que s'il est stuck ou s'il est arrivé au goal. Mais s'il est stuck, faudrait un truc genre swap pour ne pas supprimer le goal.
    // Faire nextGoalSwap pour les stucks.
    currentGoal = listOfGoal.back();
    currentGoal.setWeight(newWeight);
    /*int i = 0;
    // On réattribue les poids de chaque obstacle. Attention que pour Sample, ça va être un vecteur qui change de taille.
    for (auto &poids : currentGoal.weightSimpleBorder) // weightSimpleBorder de même longueur que simpleBorderList.
    {
        simpleBorderList.at(i).setWeight(poids);
        i += 1;
    }
    i = 0;
    for (auto &poids : currentGoal.weightOblicBorder) {
        oblicBorderList.at(i).setWeight(poids);
        i += 1;
    }
    i = 0;
    for (auto &poids : currentGoal.weightSample) {
        sampleList.at(i).setWeight(poids);
        i += 1;
    }
     */
    addIntermediateGoal(); // ! à cet appel.
}



void Potential_Field::nextGoalBase(const std::tuple<double, double>& newPosition, double goalWeight)
{
    currentGoal.setWeight(0.0);
    listOfGoal.push_back(Goal(newPosition, goalWeight, true));
    std::cout << "Added a goal to return to base at : (" << std::get<0>(newPosition) << "," << std::get<1>(newPosition) << ").\n";
    currentGoal = listOfGoal.back();
    numberOfGoals += 1;
    /*int i = 0;
    for (auto &poids : currentGoal.weightSimpleBorder)
    {
        simpleBorderList.at(i).setWeight(poids);
        i += 1;
    }
    i = 0;
    for (auto &poids : currentGoal.weightOblicBorder) {
        oblicBorderList.at(i).setWeight(poids);
        i += 1; 
    }
    i = 0;
    for (auto &poids : currentGoal.weightSample) {
        sampleList.at(i).setWeight(poids);
        i += 1;
    }*/

    addIntermediateGoal();
}

void Potential_Field::nextGoalStuck(double newWeight) 
{
    while (currentGoal.goalType == false )
    {
        removeGoal();
        currentGoal = listOfGoal.back();
    }
    currentGoal.setWeight(0.0);
    addGoal(std::make_tuple(1.1, 1.5),newWeight,  false); 
    
    /*if (listOfGoal.size() > 2 ){
        std::swap(listOfGoal.at(listOfGoal.size() -2 ),listOfGoal.at(listOfGoal.size() - 3  ));
    }*/
    currentGoal = listOfGoal.back();
    addIntermediateGoal();
}


// Calculation of the attractive force at the present point towards the goal. This gives back a vector.
std::tuple<double, double> Potential_Field::attractiveForce(std::tuple<double, double> position) {
    std::tuple<double, double> value ; 
    value = currentGoal.attForce(std::move(position));
    if (std::isnan(std::get<0>(value) )  || std::isnan(std::get<1>(value) )){
        std::cout << "nan value for position :" << tupleToString(current_position) << "\n for goal :" << tupleToString(currentGoal.position) << "and weight of : " << currentGoal.weight << "\n"; 
        return std::make_tuple(0.0,0.0);
    }

    // Limiteur de force. Si la norme du vecteur est plus grande que la limite, on limite.
    if (sqrt( pow(std::get<0>(value), 2) + pow(std::get<1>(value), 2)) > LIMIT_ATTRACTIVE_FORCE)
    {
        // On normalise puis on retourne la valeur.
        double newValueX = (std::get<0>(value) / sqrt( pow(std::get<0>(value), 2) + pow(std::get<1>(value), 2))) * LIMIT_ATTRACTIVE_FORCE;
        double newValueY = (std::get<1>(value) / sqrt( pow(std::get<0>(value), 2) + pow(std::get<1>(value), 2))) * LIMIT_ATTRACTIVE_FORCE;
        value = std::make_tuple(newValueX, newValueY);
    }

    return value;
}

std::tuple<double, double> Potential_Field::getPosition() const {
    return current_position;
}

void Potential_Field::goalStolenByOpponent(std::tuple<double, double> positionOpponent1Averaged, std::tuple<double, double> positionOpponent2Averaged)
{
    
        int goalNumber = 0;
        double opponent1X = std::get<0>(positionOpponent1Averaged);
        double opponent1Y = std::get<1>(positionOpponent1Averaged);
        double opponent2X = std::get<0>(positionOpponent2Averaged);
        double opponent2Y = std::get<1>(positionOpponent2Averaged);
        
        
        for(auto &monGoal : listOfGoal)
        {
            double ourPositionX = std::get<0>(current_position);
            double ourPositionY = std::get<1>(current_position);
            double goalX = std::get<0>(monGoal.position);
            double goalY = std::get<1>(monGoal.position);
            
            int tailleVecteur = listOfGoal.size();
                // Si la distance entre le point le plus proche détecté de l'opponent et la position du centre du goal < le rayon d'un sample, ça veut dire qu'il l'a atteint.
                if( ( (sqrt(pow(opponent1X - goalX, 2) + pow(opponent1Y - goalY, 2)) < sampleRadius) || (sqrt(pow(opponent2X - goalX, 2) + pow(opponent2Y - goalY, 2)) < sampleRadius) ) && (monGoal.goalType == true) )
                {
                    
                    if(goalNumber == (tailleVecteur-1) && numberOfGoals > 1) // Si le goal qu'il veut supprimer est l'actuel
                    {
                        double weight = currentGoal.weight;
                        currentGoal = listOfGoal.at(tailleVecteur - 2); // On chope l'avant-dernier élément
                        currentGoal.setWeight(weight);
                        listOfGoal.pop_back();
                        
                        addIntermediateGoal();

                        numberOfGoals = numberOfGoals - 1;
                    }
                     // Si on a un nombre de goal = 1 (il n'en reste qu'un), on supprime le dernier goal et on va à la base. 
                    else if (numberOfGoals == 1) 
                    {
                        nextGoalBase(coordonneesBase, WEIGHT_GOAL);
                        listOfGoal.erase(listOfGoal.begin() + 0);
                         // On supprime l'ancien goal qui vient d'être pris.
                        currentGoal = listOfGoal.back(); // On va vers la base.
                    } 
        
                    else
                    {
                        listOfGoal.erase(listOfGoal.begin() + goalNumber);


                        numberOfGoals = numberOfGoals - 1; 
                    }
                }
            goalNumber++;   
           
        }
    //}
}

void updatePotentialField(Potential_Field* myPotentialField, Controller *cvs){
    std::cout<<cvs->x << " " << cvs->y << "\n";
    std::cout<<(double) cvs->x << " " << (double) cvs->y << "\n";
    myPotentialField->setPosition(std::make_tuple((double) cvs->x, (double) cvs->y)); // récuperation de x et y
    myPotentialField->setSpeedVector( (double) cvs->theta);                  // récuperation de theta
    //std::tuple<double, double> positionOpponent1Averaged = myPotentialField.speedFilter(std::make_tuple((double) cvs-> loc_opponent1[0], (double) cvs-> loc_opponent1[1] ));
    /* TO DO ATTENTION LIDAR
    opponentList.at(0).setPositionOpponent(std::make_tuple((double) cvs-> loc_opponent1[0], (double) cvs-> loc_opponent1[1]));
    opponentList.at(1).setPositionOpponent(std::make_tuple((double) cvs-> loc_opponent2[0], (double) cvs-> loc_opponent2[1]));
    */ 
}


// ====================================================================================================================================================================================================================
// Goal Class
// ====================================================================================================================================================================================================================

Goal::Goal() {

}


Goal::Goal(const std::tuple<double, double> &goal_position, double goalWeight, bool type) {

    position = goal_position;   // Position du goal.
    weight = goalWeight;        // Poids du goal. Default weight : 0.0.
    goalType = type;
}

std::tuple<double, double> Goal::attForce(std::tuple<double, double> position_robot) {
    double co_X = -weight * (std::get<0>(position_robot) - std::get<0>(position));
    double co_Y = -weight * (std::get<1>(position_robot) - std::get<1>(position));
    std::tuple<double, double> forceAtt = std::make_tuple(co_X, co_Y);
    return forceAtt;
}

// Goal calcule la distance entre le centre du goal et le centre de la ligne joignant les deux roues du robot.
double Goal::computeDistance(std::tuple<double, double> robotPosition) {

    double distanceToReturn = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) +
                pow(std::get<1>(robotPosition) - std::get<1>(position), 2)); //- precision;
    
    if (distanceToReturn < 0.0)
    {
        return 0.0;
    }
    return distanceToReturn;
}

bool Goal::goalReached(std::tuple<double, double> position_robot) {
    double distanceToGoal = computeDistance(std::move(position_robot));
    return (distanceToGoal <= 0.0);
}

// Permet de donner un poids au goal. Par défaut, le goal a un poids de 0.
void Goal::setWeight(double value) {
    weight = value;
}




// ====================================================================================================================================================================================================================
// Obstacle Class
// ====================================================================================================================================================================================================================

Obstacle::Obstacle() {}

// Class constructor. Parameterized constructor : the position of the center is not specified, as a border is represented as a line. Use "setPosition" for other obstacles.
Obstacle::Obstacle(double k_rep, double distanceOfInfluence, std::string typeName) {
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    type = std::move(typeName);
}

void Obstacle::setWeight(double newWeight) {
    coeff = newWeight;
}

void Obstacle::setInfluence(double newInfluence) {
    rho0 = newInfluence;
}


// ====================================================================================================================================================================================================================
// Rectangle Class
// ====================================================================================================================================================================================================================

Rectangle::Rectangle()
{

}

Rectangle::Rectangle(double k_rep, double distanceOfInfluence, double hitBoxObstacle, std::vector< std::tuple<double, double> > listOfCoords)
{
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    hitBox = hitBoxObstacle;
    coordonnees = std::move(listOfCoords);
    type = "Rectangle";
}

std::tuple< double,std::tuple<double,double> > Rectangle::computeDistance(std::tuple<double, double> robotPosition) const
{
    double distanceToReturn = 10.0; // Nombre inutilement grand.
    std::tuple<double,double> currentCoord = std::make_tuple(0.0,0.0);

    for(auto &coord : coordonnees)
    {   // Compute distance : validé.
        currentCoord = coord;
        double distanceToTest = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(coord), 2) +
                                     pow(std::get<1>(robotPosition) - std::get<1>(coord), 2)) - hitBox - radius_robot;

        if(distanceToTest <= 0.0)
        {
            return std::make_tuple(0.0,currentCoord);
        }
        else
        {
            if(distanceToTest <= distanceToReturn)
            {
                distanceToReturn = distanceToTest;
            }
        }

    }
    return std::make_tuple(distanceToReturn,currentCoord);
}



// ====================================================================================================================================================================================================================
// SimpleBorder Class
// ====================================================================================================================================================================================================================

// Default constructor.
SimpleBorder::SimpleBorder() {

}

// Parameterized constructor.
SimpleBorder::SimpleBorder(double k_rep, double distanceOfInfluence, int border_type, double xoryposition, double hitBoxObstacle) {
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    borderType = border_type;
    position = xoryposition;
    hitBox = hitBoxObstacle;
    type = "SimpleBorder";
}

// Gives the euclidean distance (squared) between the line and the center of the robot.
double SimpleBorder::computeDistance(std::tuple<double, double> robotPosition) const {
    // La distance = la distance entre le centre du robot et la droite, à laquelle on soustrait le rayon du robot (le modélisant) et la hitBox (qui définit la largeur réelle de l'obstacle).
    if (borderType == 0) {
        // If we hit the obstacle, return 0 ! No negative distances !
        double distanceFinale = fabs(std::get<0>(robotPosition) - position) - hitBox - radius_robot;
        if (distanceFinale <= 0.0) {
            return 0.0;
        }
        return distanceFinale;
    } else if (borderType == 1) {
        double distanceFinale = fabs(std::get<1>(robotPosition) - position) - hitBox - radius_robot; // Attention : hitBox et radius_robot hors de la valeur fabsolue.
        if (distanceFinale <= 0.0) {
            return 0.0;
        }
        return distanceFinale;
    } else {
        throw "invalid Border type";
    }
}



// ====================================================================================================================================================================================================================
// OblicBorder Class
// ====================================================================================================================================================================================================================

// Default constructor.
OblicBorder::OblicBorder() {

}

// Parameterized constructor.
OblicBorder::OblicBorder(double k_rep, double distanceOfInfluence, double pente, double offset, double hitBoxObstacle) {
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    m = pente;
    p = offset;
    hitBox = hitBoxObstacle;
    type = "OblicBorder";
}

// Computes the distance to the closest point of the oblic border. 
double OblicBorder::computeDistance(std::tuple<double, double> robotPosition) const {
    double y2;
    double x2;
    double pente_inverse;
    double b;

    pente_inverse = 1 / m;
    b = std::get<1>(robotPosition) + pente_inverse * std::get<0>(robotPosition);
    x2 = (b - p) / (m + pente_inverse);
    y2 = b - pente_inverse * x2;
    double distanceFinale =
            sqrt(pow(std::get<0>(robotPosition) - x2, 2) + pow(std::get<1>(robotPosition) - y2, 2)) - hitBox - radius_robot;

    // If we hit the obstacle, return 0 ! No negative distances !
    if (distanceFinale <= 0.0) {
        return 0.0;
    }
    return distanceFinale;

    /*double yup; double xup; double distance1; double distance2;

    yup = m * std::get<0>(robotPosition) + p;
    xup = (std::get<1>(robotPosition) - p) / m;

    distance1 = fabs(std::get<0>(robotPosition) - xup);
    distance2 = fabs(std::get<1>(robotPosition) - yup);

    if (distance1 < distance2){
        return distance1;
    }
    else {
        return distance2;
    }*/
}


// Attention : ça retourne le point le plus proche sur la droite qui modélise l'obstacle, pas au niveau de la hitBox !
std::tuple<double, double> OblicBorder::closestPoint(std::tuple<double, double> robotPosition) const {
    double y2;
    double x2;
    double pente_inverse;
    double b;

    pente_inverse = 1 / m;
    b = std::get<1>(robotPosition) + pente_inverse * std::get<0>(robotPosition);
    x2 = (b - p) / (m + pente_inverse);
    y2 = b - pente_inverse * x2;

    std::tuple<double, double> outputTupple = std::make_tuple(x2, y2);
    return outputTupple;
}



// ====================================================================================================================================================================================================================
// Opponent Class
// ====================================================================================================================================================================================================================

// Parameterized constructor.

Opponent::Opponent(const std::tuple<double, double> &center, double k_rep, double distanceOfInfluence,
                   double hitBoxRadius) {
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    type = "Opponent";
    hitBox = hitBoxRadius;
}

// On ne tient pas en compte d'un potentiel rayon modélisé de l'opponent. On a juste notre propre rayon et une hitbox.
// Calcul a priori validé. Potentiellement un point d'amélioration. TAG
double Opponent::computeDistance(std::tuple<double, double> robotPosition) {
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) +
                                   pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    double distanceObstacle = distanceToCenter - hitBox - radius_robot; // ça suppose qu'on ne cherche à l'éviter que si on lui fonce dessus et pas l'inverse.
    if (distanceObstacle <= 0.0) {
        return 0.0;
    }
    return distanceObstacle;
}


// Harder part : to be implemented.
// The information we get is coming from the sensors. They will give us an estimation of the distance towards the obstacle.
// By having an idea of the smallest distance to the opponent and the angle at which it is, we can estimate the position of the opponent.
// Récuperer la position du robot et de ce qui est détecté dans les autres fichiers, comme ctrlStruct !
void Opponent::setPositionOpponent(const std::tuple<double, double> &obstaclePosition) {
    position = obstaclePosition;
}



// ====================================================================================================================================================================================================================
// Sample Class
// ====================================================================================================================================================================================================================

// Parameterized constructor.

Sample::Sample(const std::tuple<double, double> &center, double k_rep, double distanceOfInfluence,
               double hitBoxRadius) {
    position = center;
    coeff = k_rep;
    rho0 = distanceOfInfluence;
    hitBox = hitBoxRadius;
    type = "Sample";
}

double Sample::computeDistance(std::tuple<double, double> robotPosition) {
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) +
                                   pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    double distanceObstacle = distanceToCenter - hitBox - radius_robot;
    if (distanceObstacle <= 0.0) {
        return 0.0;
    }
    return distanceObstacle;
}

// Position of the center of an obstacle. If the determination of the center is impossible, we say that the point = the closest point detected.
void Sample::setPositionSample(const std::tuple<double, double> &obstaclePosition) {
    position = obstaclePosition;
}


// ====================================================================================================================================================================================================================
// Global Functions
// ====================================================================================================================================================================================================================

std::string tupleToString(std::tuple<double, double> entry) {
    std::string output_s;
    output_s = std::to_string(std::get<0>(entry)) + " " + std::to_string(std::get<1>(entry));
    return output_s;
}

Potential_Field initPotentialField() // Rajouter la position initiale pour savoir si on est bleu ou jaune.
{

// Initialisation : on commence par déclarer les obstacles connus de la carte :
// D'abord les bords dans l'ordre défini (simple puis oblic) puis les rectangles.
// On définit ensuite la liste de goals (la liste donne l'ordre des samples qu'on veut aller trouver).

// Ensuite, on chope la position de notre robot et son orientation dans l'espace.

    Potential_Field myPotentialField = Potential_Field();

    for(int i = 0; i < 1000; i++)
    {
        myPotentialField.list_for_speed_filtering.push(0.0);
    }

    myPotentialField.addSimpleBorder(SimpleBorder(krep_border, rho0_border, 1, 0.0, hitbox_obstacle)); // Bord horizontal bas.
    myPotentialField.addSimpleBorder(SimpleBorder(krep_border, rho0_border, 0, 0.0, hitbox_obstacle)); // Bord vertical gauche.
    myPotentialField.addSimpleBorder(SimpleBorder(krep_border, rho0_border, 1, 3.0, hitbox_obstacle)); // Bord horizontal haut.
    myPotentialField.addSimpleBorder(SimpleBorder(krep_border, rho0_border, 0, 2.0, hitbox_obstacle)); // Bord vertical droit.

    // Oblic borders.
    myPotentialField.addOblicBorder(OblicBorder(krep_border, rho0_border, 1.0, -1.49, hitbox_obstacle)); // Bord oblique en bas à droite.
    myPotentialField.addOblicBorder(OblicBorder(krep_border, rho0_border, -1.0, 4.49, hitbox_obstacle)); // Bord oblique en haut à droite.

    // Rectangles.
    std::vector< std::tuple<double, double> > rectangleBasDroite { std::make_tuple(1.175, 0.0),
        std::make_tuple(1.175, 0.102), std::make_tuple(1.325, 0.102), std::make_tuple(1.325, 0.0) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, hitbox_obstacle, rectangleBasDroite));

    std::vector< std::tuple<double, double> > galerieExpoBas { std::make_tuple(0.0, 0.45),
        std::make_tuple(0.1, 0.45), std::make_tuple(0.1, 0.6675), std::make_tuple(0.1, 0.8525),
        std::make_tuple(0.1, 1.0375), std::make_tuple(0.1, 1.17), std::make_tuple(0.0, 1.17) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, hitbox_obstacle, galerieExpoBas));

    std::vector< std::tuple<double, double> > galerieExpoHaut { std::make_tuple(0.0, 1.83),
        std::make_tuple(0.1, 1.83), std::make_tuple(0.1, 2.055), std::make_tuple(0.1, 2.24),
        std::make_tuple(0.1, 2.245), std::make_tuple(0.1, 2.55), std::make_tuple(0.0, 2.55) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, hitbox_obstacle, galerieExpoHaut));

    std::vector< std::tuple<double, double> > sproutchAuDessusGalerieExpoBas { std::make_tuple(0.0, 1.275),
        std::make_tuple(0.102, 1.275), std::make_tuple(0.102, 1.425), std::make_tuple(0.0, 1.425) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, hitbox_obstacle, sproutchAuDessusGalerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchEnDessousGalerieExpoHaut { std::make_tuple(0.0, 1.575),
        std::make_tuple(0.102, 1.575), std::make_tuple(0.102, 1.725), std::make_tuple(0.0, 1.725) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, hitbox_obstacle, sproutchEnDessousGalerieExpoHaut));

    std::vector< std::tuple<double, double> > laTigeAGauche { std::make_tuple(0.0, 1.4975),
        std::make_tuple(0.102, 1.4975), std::make_tuple(0.3, 1.4975), std::make_tuple(0.3, 1.525),
        std::make_tuple(0.102, 1.525), std::make_tuple(0.0, 1.525) };
    myPotentialField.addRectangle(Rectangle(krep_rectangle, rho0_rectangle, 0.00, laTigeAGauche));

    std::vector< std::tuple<double, double> > rectangleHautDroite { std::make_tuple(1.175, 3.0),
        std::make_tuple(1.175, 2.898), std::make_tuple(1.325, 2.898), std::make_tuple(1.325, 3.0) };
    myPotentialField.addRectangle(Rectangle(krep_tige, rho0_tige, hitbox_obstacle, rectangleHautDroite));


    // À l'initialisation, les opponents sont mis hors de la map. 
    // Arguments : position, k_rep, distanceOfInfluence, hitbox radius.
    // Son radius de base est de 15 [cm]. Il ne sert cependant à rien pour l'instant. Pour avoir de l'importance, mettre radiusOpponent en dernier argument. 
    //myPotentialField.addOpponent(Opponent(std::make_tuple(0.795,0.9), krep_opponent, rho0_opponent, hitbox_opponent));
    //myPotentialField.addOpponent(Opponent(std::make_tuple(8.0,8.0), krep_opponent, rho0_opponent, hitbox_opponent));
    
    /*
    std::vector< std::tuple<double, double> > carreDeFouille { std::make_tuple(2.0, 0.575),

        std::make_tuple(1.98, 0.575), std::make_tuple(1.98, 0.6675), std::make_tuple(1.98, 0.8525),
        std::make_tuple(1.98, 1.0375), std::make_tuple(1.98, 1.2225), std::make_tuple(1.98, 1.4075),
        std::make_tuple(1.98, 1.5925), std::make_tuple(1.98, 1.775), std::make_tuple(1.98, 1.9625),
        std::make_tuple(1.98, 2.1475), std::make_tuple(1.98, 2.3325), std::make_tuple(1.98, 2.425),
        std::make_tuple(2.0, 2.425) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, carreDeFouille));

    /*
    // Simple borders. Use of push_back : first in = the last one of the list.
    double krep = 0.02;
    myPotentialField.addSimpleBorder(SimpleBorder(krep, 0.15, 1, -1.5, 0.001)); // Bord horizontal bas.
    myPotentialField.addSimpleBorder(SimpleBorder(krep, 0.15, 0, -1.0, 0.001)); // Bord vertical gauche.
    myPotentialField.addSimpleBorder(SimpleBorder(krep, 0.15, 1, 1.5, 0.001));  // Bord horizontal haut.
    myPotentialField.addSimpleBorder(SimpleBorder(krep, 0.15, 0, 1.0, 0.001));  // Bord vertical droit.


    // Oblic borders.
    myPotentialField.addOblicBorder(OblicBorder(krep, 0.15, 1.0, -2.07484, 0.001)); // Bord oblique en bas à droite.
    myPotentialField.addOblicBorder(OblicBorder(krep, 0.15, -1.0, 2.07484, 0.001)); // Bord oblique en haut à droite.

    // Rectangles.
    std::vector< std::tuple<double, double> > rectangleBasDroite { std::make_tuple(0.175,-1.5),
                                                                   std::make_tuple(0.325,-1.5), std::make_tuple(0.325,-1.398), std::make_tuple(0.175, -1.398) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, rectangleBasDroite));

    std::vector< std::tuple<double, double> > rectangleHautDroite { std::make_tuple(0.175, 1.398),
                                                                    std::make_tuple(0.325, 1.398), std::make_tuple(0.325, 1.5), std::make_tuple(0.175, 1.5) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, rectangleHautDroite));

    std::vector< std::tuple<double, double> > galerieExpoHaut { std::make_tuple(-1.0, 0.33),
                                                                std::make_tuple(-0.915, 0.33), std::make_tuple(-0.915, 0.57), std::make_tuple(-0.915, 0.81),
                                                                std::make_tuple(-0.915, 1.05), std::make_tuple(-1.0, 1.05) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, galerieExpoHaut));

    std::vector< std::tuple<double, double> > galerieExpoBas { std::make_tuple(-1.0, -0.33),
                                                               std::make_tuple(-0.915, -0.33), std::make_tuple(-0.915, -0.57), std::make_tuple(-0.915, -0.81),
                                                               std::make_tuple(-0.915, -1.05), std::make_tuple(-1.0, -1.05) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, galerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchAuDessusGalerieExpoBas { std::make_tuple(-1.0, -0.225),
                                                                               std::make_tuple(-0.898, -0.225), std::make_tuple(-0.898, -0.075), std::make_tuple(-1.0, -0.075) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, sproutchAuDessusGalerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchEnDessousGalerieExpoHaut { std::make_tuple(-1.0, 0.075),
                                                                                 std::make_tuple(-0.898, 0.075), std::make_tuple(-0.898, 0.225), std::make_tuple(-1.0, 0.225) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.01, sproutchEnDessousGalerieExpoHaut));

    std::vector< std::tuple<double, double> > laTigeAGauche { std::make_tuple(-0.8, 0),
                                                              std::make_tuple(-0.6, 0.0), std::make_tuple(-0.5, 0.0), std::make_tuple(-0.4, 0.0) };
    myPotentialField.addRectangle(Rectangle(krep, 0.15, 0.001, laTigeAGauche));
    */

    return myPotentialField;
}

void initGoals(Potential_Field * myPotentialField, int teamNumber)
{
    // WEIGHT_GOAL est dans 'data.h'.
    if(teamNumber == 0) // ÉQUIPE BLEUE (démarre côté (0,3) )
    {   
        myPotentialField->coordonneesBase = std::make_tuple(0.7, 2.825);

        /*myPotentialField->addGoal(std::make_tuple(0.15, 0.15), 0.0,     true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(1.45, 2.8), 0.0,      true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(0.17, 2.80), 0.0,     true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(1.45, 0.2), 0.0,      true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(0.25, 1.7), 0.0,      true);      // 2 points.
        myPotentialField->addGoal(std::make_tuple(0.25, 1.3), 0.0,      true);      // 2 points.
        myPotentialField->addGoal(std::make_tuple(1.85, 1.5), WEIGHT_GOAL,    true);      // 3 points. First one so we give it a weight. */

        // More accessory goals on the other side of the map.
    }
    else if (teamNumber == 1) // ÉQUIPE JAUNE (démarre côté (0,0) )
    { 

       /* myPotentialField->addGoal(std::make_tuple(0.15, 2.85), 0.0,     true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(1.45, 0.2), 0.0,      true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(0.15, 0.15), 0.0,     true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(1.45, 2.8), 0.0,      true);      // 1 point.
        myPotentialField->addGoal(std::make_tuple(0.25, 1.3), 0.0,      true);      // 2 points.
        myPotentialField->addGoal(std::make_tuple(0.25, 1.7), 0.0,      true);      // 2 points.
        myPotentialField->addGoal(std::make_tuple(1.85, 1.5), WEIGHT_GOAL,    true);      // 3 points. First one so we give it a weight. */

        myPotentialField->coordonneesBase = std::make_tuple(0.7, 0.175);

        
        // ENCHAINEMENT DE GOALS : d'abord les probes, puis la statuette. Puis quid ?
        // RAPPEL : l'ordre semble inversé mais non, tout va bien, c'est une FIFO.
        myPotentialField->addGoal(std::make_tuple(1.49, 0.51),  0.0,    true);     // Statuette 2.
        myPotentialField->addGoal(std::make_tuple(0.3, 0.255),  0.0,    true);     // Statuette 1.
        myPotentialField->addGoal(std::make_tuple(0.6, 0.3),    0.0,    true);     // Retour à la base en passant par les samples.
        myPotentialField->addGoal(std::make_tuple(1.7, 1.5),    0.0,    true);     // Goal de modélisation, à supprimer après.
        myPotentialField->addGoal(std::make_tuple(1.7, 0.6675), WEIGHT_GOAL,   true);    // On arrive proche des premières probes. 

    }
    else
    {
        throw("Invalid team number (must be between 0 and 3).");
    }
    myPotentialField->addIntermediateGoal();
}


void initGoalsTest(Potential_Field * myPotentialField, int teamNumber){
    myPotentialField->addGoal(std::make_tuple(1.7, 1.2), 0.0, true);
    myPotentialField->addGoal(std::make_tuple(0.6, 1.2), WEIGHT_GOAL, true);      // Goal de test.
    myPotentialField->coordonneesBase = std::make_tuple(0.7, 0.25);

    /*
    // ENCHAINEMENT DE GOALS : d'abord les probes, puis la statuette. Puis quid ?
    myPotentialField->addGoal(std::make_tuple(1.7, 0.6675), 0.0, true);    // On arrive proche des premières probes. 
    myPotentialField->addGoal(std::make_tuple(1.7, 1.5), 0.0, true);       // Goal de modélisation, à supprimer après.
    myPotentialField->addGoal(std::make_tuple(0.6, 0.3), 0.0, true);       // Retour à la base en passant par les samples.
    myPotentialField->addGoal(std::make_tuple(0.3, 0.255), 0.0, true);     // Statuette 1.
    myPotentialField->addGoal(std::make_tuple(1.49, 0.51), 10.0, true);     // Statuette 2.*/
}


std::tuple<double, double> iterPotentialFieldWithLogFile(Potential_Field * myPotential_Field, double dt, FILE  * myFile) {
        //myPotential_Field->current_position = myPotential_Field->getPosition();
        std::tuple<double, double> myRepulsiveForce = myPotential_Field->totalRepulsiveForce();
        std::tuple<double, double> attractionForce = myPotential_Field->attractiveForce(myPotential_Field->current_position);
        std::tuple<double, double> mySpeed = myPotential_Field->getSpeedVector(dt, global_vMax, global_wMax, myPotential_Field->current_position);
        
        std::cout<< "in \n";
        std::cout << std::get<0>(myPotential_Field->current_position) << " " <<   std::get<1>(myPotential_Field->current_position) << "\n"; 
        fprintf( myFile, "%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \n",  
                                                              (float)std::get<0>(myPotential_Field->current_position),   (float)std::get<1>(myPotential_Field->current_position), 
                                                              (float)std::get<0>(myRepulsiveForce),                      (float)std::get<1>(myRepulsiveForce), 
                                                              (float)std::get<0>(attractionForce),                       (float)std::get<1>(attractionForce), 
                                                              (float)std::get<0>(mySpeed),                               (float)std::get<1>(mySpeed), 
                                                              (float)std::get<0>(myPotential_Field->currentSpeedVector), (float)std::get<1>(myPotential_Field->currentSpeedVector) ) ;
        if (std::get<0>(mySpeed) == 0 && std::get<1>(mySpeed) == 0){
            myPotential_Field->willNotMove = true;
        }else{
            myPotential_Field->willNotMove = false;
        }


        myPotential_Field->isStuck= (myPotential_Field->didntMove > 1000 ) && !(myPotential_Field->GoalTest()) && (myPotential_Field->didntRotate > 1000);

        return mySpeed;
}

std::tuple<double, double> iterPotentialField(Potential_Field myPotential_Field, double dt) {
    if (myPotential_Field.GoalTest()) {
        return std::make_tuple(0.0, 0.0);
    } else {
        myPotential_Field.current_position = myPotential_Field.getPosition();
        std::tuple<double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple<double, double> attractionForce = myPotential_Field.attractiveForce(
                myPotential_Field.current_position);
        std::tuple<double, double> mySpeed = myPotential_Field.getSpeedVector(dt, global_vMax, global_wMax,
                                                                              myPotential_Field.current_position);
        return mySpeed;
    }
}




// Filtre sur les vitesses pour être physiquement cohérent.
std::tuple<double, double> Filter(std::tuple<double, double> opponentPosition, std::deque<double>*  stack, double *ouput1_filtered, double *output2_filtered) {
        double x = std::get<0>(opponentPosition);
        double y = std::get<1>(opponentPosition);
        stack->push_back(x);
        stack->push_back(y);
        *ouput1_filtered += 2* (x - stack->front()) / stack->size();
        stack->pop_front();
        *output2_filtered += 2* (y - stack->front()) / stack->size();
        stack->pop_front();
        

        int type = 0;
        // on vérifie que les 5 dernières valeurs sont dans un rayon de 10 cm du centre -> rejet d'érreur de mesures importantes 
        for(auto it = stack->cbegin(); it != stack->cend(); ++it){
            if (type == 0){
                if (fabs(*ouput1_filtered - *it)  > 0.4 ) {
                    return std::make_tuple(10.0,10.0);
                }
                type = 1;
            }
            else if (type == 1){
                if (fabs(*output2_filtered - *it) > 0.4){
                    return std::make_tuple(10.0,10.0);
                }
                type = 0;
            }
        }

    return std::make_tuple(*ouput1_filtered, *output2_filtered);
}

