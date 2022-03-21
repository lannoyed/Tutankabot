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
Potential_Field::Potential_Field() {
    std::cout << "The default constructor of 'Potential_Field' has been called."
              << std::endl; // endl = end_line => '\n' in Python.
    // cout means "character output" and wcout means "wide character output". std::cout format. Quick way of saying "print".
}

// Parameterized constructor.

Potential_Field::Potential_Field(const std::tuple<double, double> &position) {
    current_position = position;

    numberOfGoals = 0;

    filter_output_Vref = 0.0;
    filter_output_Wref = 0.0;
    // 10 push pour faire une moyenne sur 5 éléments pour V et W
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
}

// Update function to get a new position.
void Potential_Field::setPosition(const std::tuple<double, double> &position) {
    current_position = position;
}


bool Potential_Field::GoalTest(double precision) {
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

            //std::cout << "Distance = \t" << realDistance << std::endl;
            //std::cout << "Repulsive force after border: " << i << "\t" << totalRepForceX << " \t" << totalRepForceY << "\n" <<  std::endl;

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

    std::tuple<double, double> totalRepForce = std::make_tuple(totalRepForceX, totalRepForceY);

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
    sampleList.erase(sampleList.begin());
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
        omega = PI / dt;
    } else {
        omega = S * acos(cosTheta) / dt;
    }

    // Limiteur de vitesse : on ne peut pas aller à max vitesse tout droit et angulaire.
    if (omega >= omegaMax) {
        double dTheta = (omegaMax - omega) * dt;
        omega = omegaMax;
        nextSpeedVector = std::make_tuple(
                std::get<0>(nextSpeedVector) * cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),
                std::get<0>(nextSpeedVector) * sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta));
    } else if (omega <= -omegaMax) {
        double dTheta = (omegaMax - omega) * dt;
        omega = -omegaMax;
        nextSpeedVector = std::make_tuple(
                std::get<0>(nextSpeedVector) * cos(dTheta) - std::get<1>(nextSpeedVector) * sin(dTheta),
                std::get<0>(nextSpeedVector) * sin(dTheta) + std::get<1>(nextSpeedVector) * cos(dTheta));
    }

    double vMaxReal = vMax - 0.18 * std::fabs(omega);
    if (vRefNext > vMaxReal) {
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector) * vMaxReal / vRefNext,
                                          std::get<1>(nextSpeedVector) * vMaxReal / vRefNext);
        vRefNext = vMaxReal;
    }
    //cout << "cosTheta" << cosTheta << "\n";
    setSpeedVector(nextSpeedVector); // update de speedVector
    return std::make_tuple(vRefNext, omega);
}

// Filtre sur les vitesses pour être physiquement cohérent.
std::tuple<double, double> Potential_Field::speedFilter(std::tuple<double, double> speedVector) {
    double V = std::get<0>(speedVector) / 10.0;
    double W = std::get<1>(speedVector) / 10.0;
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
void Potential_Field::addGoal(const std::tuple<double, double>& newGoalPosition, double goalWeight,
                              const std::vector<double>& weightSimpleBorder, const std::vector<double>& weightOblicBorder,
                              const std::vector<double>& weightSample, const std::vector<double>& weightRectangles) {
    if (numberOfGoals == 0) {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight,
                                     weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles));
        currentGoal = listOfGoal.at(0);
    } else {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight,
                                     weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles));
    }

}

// Remove the first goal added.
void Potential_Field::removeGoal() {
    numberOfGoals = numberOfGoals - 1;
    listOfGoal.erase(listOfGoal.begin());
}

// If the goal has been reached, we delete it from the list and set the new goal to the next one.
void Potential_Field::nextGoal(double newWeight, double precision) {
    // Next goal : appelée que s'il est stuck ou s'il est arrivé au goal. Mais s'il est stuck, faudrait un truc genre swap pour ne pas supprimer le goal.
    removeGoal(); // Faire nextGoalSwap pour les stucks.
    currentGoal = listOfGoal.at(0);
    currentGoal.setWeight(newWeight);
    int i = 0;
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
}

void Potential_Field::nextGoalBase(const std::tuple<double, double>& newPosition, double goalWeight,
                                   const std::vector<double>& weightSimpleBorder, const std::vector<double>& weightOblicBorder,
        // Default constructor.
                                   Potential_Field::Potential_Field() {
    std::cout << "The default constructor of 'Potential_Field' has been called."
              << std::endl; // endl = end_line => '\n' in Python.
    // cout means "character output" and wcout means "wide character output". std::cout format. Quick way of saying "print".
}

// Parameterized constructor.

Potential_Field::Potential_Field(const std::tuple<double, double> &position) {
    current_position = position;

    numberOfGoals = 0;

    filter_output_Vref = 0.0;
    filter_output_Wref = 0.0;
    // 10 push pour faire une moyenne sur 5 éléments pour V et W
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
    list_for_speed_filtering.push(0.0);
}

// Update function to get a new position.
void Potential_Field::setPosition(const std::tuple<double, double> &position) {
    if ( fabs(std::get<0>(current_position) - std::get<0>(position)) <0.00001 && fabs(std::get<1>(current_position) - std::get<1>(position)) <0.00001 ){
        didntMove += 1;
    }else{
        didntMove = 0;
    }
    std::cout << std::get<0>(current_position) << "\t" << std::get<0>(position) << "\n";
    std::cout << std::get<1>(current_position) << "\t" << std::get<1>(position) << "\n";
    current_position = position;
}


bool Potential_Field::GoalTest(double precision) {
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
    std::cout << "curent speed vector = \t" << tupleToString(currentSpeedVector)<< "\n";
    if ( fabs(std::get<0>( currentSpeedVector) - cos(theta))  < 0.1 &&  fabs(std::get<1>( currentSpeedVector) - sin(theta) ) < 0.1 ) {
        didntRotate += 1;
    }else{
        didntRotate = 0;
    }
    currentSpeedVector = std::make_tuple(cos(theta), sin(theta));
    std::cout << "new speed vector = \t" << tupleToString(currentSpeedVector)<< "\n";
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

            //std::cout << "Distance = \t" << realDistance << std::endl;
            //std::cout << "Repulsive force after border: " << i << "\t" << totalRepForceX << " \t" << totalRepForceY << "\n" <<  std::endl;

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

    std::tuple<double, double> totalRepForce = std::make_tuple(totalRepForceX, totalRepForceY);

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
    sampleList.erase(sampleList.begin());
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
        omega = PI / dt;
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

    double vMaxReal = vMax - 0.18 * std::fabs(omega);
    if (vRefNext > vMaxReal) {
        nextSpeedVector = std::make_tuple(std::get<0>(nextSpeedVector) * vMaxReal / vRefNext,
                                          std::get<1>(nextSpeedVector) * vMaxReal / vRefNext);
        vRefNext = vMaxReal;
    }
    //cout << "cosTheta" << cosTheta << "\n";
    //setSpeedVector(nextSpeedVector); // update de speedVector
    return std::make_tuple(vRefNext, omega);
}

// Filtre sur les vitesses pour être physiquement cohérent.
std::tuple<double, double> Potential_Field::speedFilter(std::tuple<double, double> speedVector) {
    double V = std::get<0>(speedVector) / 10.0;
    double W = std::get<1>(speedVector) / 10.0;
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
void Potential_Field::addGoal(const std::tuple<double, double>& newGoalPosition, double goalWeight,
                              const std::vector<double>& weightSimpleBorder, const std::vector<double>& weightOblicBorder,
                              const std::vector<double>& weightSample, const std::vector<double>& weightRectangles) {
    if (numberOfGoals == 0) {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight,
                                     weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles));
        currentGoal = listOfGoal.at(0);
    } else {
        numberOfGoals += 1;
        listOfGoal.emplace_back(Goal(newGoalPosition, goalWeight,
                                     weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles));
    }

}

// Remove the first goal added.
void Potential_Field::removeGoal() {
    numberOfGoals = numberOfGoals - 1;
    listOfGoal.erase(listOfGoal.begin());
}

// If the goal has been reached, we delete it from the list and set the new goal to the next one.
void Potential_Field::nextGoal(double newWeight) {
    // Next goal : appelée que s'il est stuck ou s'il est arrivé au goal. Mais s'il est stuck, faudrait un truc genre swap pour ne pas supprimer le goal.
    removeGoal(); // Faire nextGoalSwap pour les stucks.
    currentGoal = listOfGoal.at(0);
    currentGoal.setWeight(newWeight);
    int i = 0;
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
}

void Potential_Field::nextGoalBase(const std::tuple<double, double>& newPosition, double goalWeight,
                                   const std::vector<double>& weightSimpleBorder, const std::vector<double>& weightOblicBorder,
                                   const std::vector<double>& weightSample, const std::vector<double>& weightRectangles)
{
    currentGoal.setWeight(0.0);
    listOfGoal.insert(listOfGoal.begin(), Goal(newPosition, goalWeight,
                                               weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles));
    currentGoal = listOfGoal.at(0);

    int i = 0;
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
    }
}

void Potential_Field::nextGoalStuck(double newWeight) {
    /*
     * do do swap of goals
     */
}


// Calculation of the attractive force at the present point towards the goal. This gives back a vector.
std::tuple<double, double> Potential_Field::attractiveForce(std::tuple<double, double> position) {
    return currentGoal.attForce(std::move(position));
}

std::tuple<double, double> Potential_Field::getPosition() const {
    return current_position;
}



// ====================================================================================================================================================================================================================
// Goal Class
// ====================================================================================================================================================================================================================

Goal::Goal() {
    std::cout << "Default constructor of class 'Goal'" << std::endl;
}


Goal::Goal(const std::tuple<double, double> &goal_position, double goalWeight,
           std::vector<double> weightSimpleBorderA, std::vector<double>  weightOblicBorderA,
           std::vector<double> weightSampleA, std::vector<double> weightRectanglesA) {

    position = goal_position;   // Position du goal.
    weight = goalWeight;        // Poids du goal. Default weight : 0.0.
    weightSimpleBorder = std::move(weightSimpleBorderA);
    weightOblicBorder = std::move(weightOblicBorderA);
    weightSample = std::move(weightSampleA);
    weightRectangles = std::move(weightRectanglesA);
}

std::tuple<double, double> Goal::attForce(std::tuple<double, double> position_robot) {
    double co_X = -weight * (std::get<0>(position_robot) - std::get<0>(position));
    double co_Y = -weight * (std::get<1>(position_robot) - std::get<1>(position));
    std::tuple<double, double> forceAtt = std::make_tuple(co_X, co_Y);
    return forceAtt;
}


double Goal::computeDistance(std::tuple<double, double> robotPosition) {
    return sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) +
                pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
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

Obstacle::Obstacle() {
    std::cout << "Default constructor of class 'Obstacle'" << std::endl;
}

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
    std::cout << "Default constructor of class 'Rectangle'" << std::endl;
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
    double distanceToReturn = 50000.0; // Nombre inutilement grand.
    std::tuple<double,double> currentCoord = std::make_tuple(0.0,0.0);

    for(auto &coord : coordonnees)
    {   // /!\ Risque d'oscillations vu que le potential field est en mode "bulles" ?
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
    std::cout << "Default constructor of class 'Border'" << std::endl;
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
    std::cout << "Default constructor of class 'Border'" << std::endl;
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

// Give an approximation of the euclidean distance (squared) between the line and the center of the robot.
// The idea is that the euclidean distance will always be longer than the smallest distance to the x or the y.
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
            sqrt(pow(std::get<0>(robotPosition) - x2, 2) + pow(std::get<1>(robotPosition) - y2, 2)) - hitBox -
            radius_robot;

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

// gérer si le résultat est négatif -> shouldn't be, though.
double Opponent::computeDistance(std::tuple<double, double> robotPosition) {
    double distanceToCenter = sqrt(pow(std::get<0>(robotPosition) - std::get<0>(position), 2) +
                                   pow(std::get<1>(robotPosition) - std::get<1>(position), 2));
    double distanceObstacle = distanceToCenter - hitBox - radius_robot;
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

Potential_Field initPotentialField()
{

// Initialisation : on commence par déclarer les obstacles connus de la carte :
// D'abord les bords dans l'ordre défini (simple puis oblic) puis les rectangles.
// On définit ensuite la liste de goals (la liste donne l'ordre des samples qu'on veut aller trouver).

// Ensuite, on chope la position de notre robot et son orientation dans l'espace.

    Potential_Field myPotentialField = Potential_Field();


    /*
    // Simple borders.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 1, 0.0, 0.05)); // Bord horizontal bas.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 0, 0.0, 0.05)); // Bord vertical gauche.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 1, 3.0, 0.05)); // Bord horizontal haut.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 0, 2.0, 0.05)); // Bord vertical droit.

    // Oblic borders.
    myPotentialField.addOblicBorder(OblicBorder(2.0, 0.15, 1.0, -1.49, 0.05)); // Bord oblique en bas à droite.
    myPotentialField.addOblicBorder(OblicBorder(2.0, 0.15, -1.0, 4.49, 0.05)); // Bord oblique en haut à droite.

    // Rectangles.
    std::vector< std::tuple<double, double> > rectangleBasDroite { std::make_tuple(1.175, 0.0),
        std::make_tuple(1.175, 0.102), std::make_tuple(1.325, 0.102), std::make_tuple(1.325, 0.0) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, rectangleBasDroite));

    std::vector< std::tuple<double, double> > galerieExpoBas { std::make_tuple(0.0, 0.45),
        std::make_tuple(0.1, 0.45), std::make_tuple(0.1, 0.6675), std::make_tuple(0.1, 0.8525),
        std::make_tuple(0.1, 1.0375), std::make_tuple(0.1, 1.17), std::make_tuple(0.0, 1.17) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, galerieExpoBas));

    std::vector< std::tuple<double, double> > galerieExpoHaut { std::make_tuple(0.0, 1.83),
        std::make_tuple(0.1, 1.83), std::make_tuple(0.1, 2.055), std::make_tuple(0.1, 2.24),
        std::make_tuple(0.1, 2.245), std::make_tuple(0.1, 2.55), std::make_tuple(0.0, 2.55) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, galerieExpoHaut));

    std::vector< std::tuple<double, double> > sproutchAuDessusGalerieExpoBas { std::make_tuple(0.0, 1.275),
        std::make_tuple(0.102, 1.275), std::make_tuple(0.102, 1.425), std::make_tuple(0.0, 1.425) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, sproutchAuDessusGalerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchEnDessousGalerieExpoHaut { std::make_tuple(0.0, 1.575),
        std::make_tuple(0.102, 1.575), std::make_tuple(0.102, 1.725), std::make_tuple(0.0, 1.725) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, sproutchEnDessousGalerieExpoHaut));

    std::vector< std::tuple<double, double> > laTigeAGauche { std::make_tuple(0.0, 1.4975),
        std::make_tuple(0.102, 1.4975), std::make_tuple(0.3, 1.4975), std::make_tuple(0.3, 1.525),
        std::make_tuple(0.102, 1.525), std::make_tuple(0.0, 1.525) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, laTigeAGauche));

    std::vector< std::tuple<double, double> > rectangleHautDroite { std::make_tuple(1.175, 3.0),
        std::make_tuple(1.175, 2.898), std::make_tuple(1.325, 2.898), std::make_tuple(1.325, 3.0) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, rectangleHautDroite));

    std::vector< std::tuple<double, double> > carreDeFouille { std::make_tuple(2.0, 0.575),
        std::make_tuple(1.98, 0.575), std::make_tuple(1.98, 0.6675), std::make_tuple(1.98, 0.8525),
        std::make_tuple(1.98, 1.0375), std::make_tuple(1.98, 1.2225), std::make_tuple(1.98, 1.4075),
        std::make_tuple(1.98, 1.5925), std::make_tuple(1.98, 1.775), std::make_tuple(1.98, 1.9625),
        std::make_tuple(1.98, 2.1475), std::make_tuple(1.98, 2.3325), std::make_tuple(1.98, 2.425),
        std::make_tuple(2.0, 2.425) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, carreDeFouille));*/


    // Simple borders. Use of push_back : first in = the last one of the list.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 1, -1.5, 0.05)); // Bord horizontal bas.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 0, -1.0, 0.05)); // Bord vertical gauche.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 1, 1.5, 0.05));  // Bord horizontal haut.
    myPotentialField.addSimpleBorder(SimpleBorder(2.0, 0.15, 0, 1.0, 0.05));  // Bord vertical droit.

    std::vector<double> weightSimpleBorder;
    int i = 0;
    for(i; i < 4; i++)
    {
        weightSimpleBorder.push_back(2.0);
    }

    std::vector<double> weightOblicBorder;
    i = 0;
    for(i; i < 2; i++)
    {
        weightOblicBorder.push_back(2.0);
    }

    std::vector<double> weightRectangles;
    i = 0;
    for(i; i < 7; i++)
    {
        weightRectangles.push_back(2.0);
    }

    std::vector<double> weightSample;
    i = 0;
    for(i; i < 1; i++)
    {
        weightSample.push_back(0.0);
    }


    // Oblic borders.
    myPotentialField.addOblicBorder(OblicBorder(2.0, 0.15, 1.0, -2.07484, 0.06)); // Bord oblique en bas à droite.
    myPotentialField.addOblicBorder(OblicBorder(2.0, 0.15, -1.0, 2.07484, 0.06)); // Bord oblique en haut à droite.

    // Rectangles.
    std::vector< std::tuple<double, double> > rectangleBasDroite { std::make_tuple(0.175,-1.5),
                                                                   std::make_tuple(0.325,-1.5), std::make_tuple(0.325,-1.398), std::make_tuple(0.175, -1.398) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, rectangleBasDroite));

    std::vector< std::tuple<double, double> > rectangleHautDroite { std::make_tuple(0.175, 1.398),
                                                                    std::make_tuple(0.325, 1.398), std::make_tuple(0.325, 1.5), std::make_tuple(0.175, 1.5) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, rectangleHautDroite));

    std::vector< std::tuple<double, double> > galerieExpoHaut { std::make_tuple(-1.0, 0.33),
                                                                std::make_tuple(-0.915, 0.33), std::make_tuple(-0.915, 0.57), std::make_tuple(-0.915, 0.81),
                                                                std::make_tuple(-0.915, 1.05), std::make_tuple(-1.0, 1.05) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, galerieExpoHaut));

    std::vector< std::tuple<double, double> > galerieExpoBas { std::make_tuple(-1.0, -0.33),
                                                               std::make_tuple(-0.915, -0.33), std::make_tuple(-0.915, -0.57), std::make_tuple(-0.915, -0.81),
                                                               std::make_tuple(-0.915, -1.05), std::make_tuple(-1.0, -1.05) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, galerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchAuDessusGalerieExpoBas { std::make_tuple(-1.0, -0.225),
                                                                               std::make_tuple(-0.898, -0.225), std::make_tuple(-0.898, -0.075), std::make_tuple(-1.0, -0.075) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, sproutchAuDessusGalerieExpoBas));

    std::vector< std::tuple<double, double> > sproutchEnDessousGalerieExpoHaut { std::make_tuple(-1.0, 0.075),
                                                                                 std::make_tuple(-0.898, 0.075), std::make_tuple(-0.898, 0.225), std::make_tuple(-1.0, 0.225) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.05, sproutchEnDessousGalerieExpoHaut));

    std::vector< std::tuple<double, double> > laTigeAGauche { std::make_tuple(-0.8, 0),
                                                              std::make_tuple(-0.6, 0.0), std::make_tuple(-0.5, 0.0), std::make_tuple(-0.4, 0.0) };
    myPotentialField.addRectangle(Rectangle(2.0, 0.15, 0.022, laTigeAGauche));





    // Les goals, définis pour l'équipe bleue.
    myPotentialField.addGoal(std::make_tuple(0.85, 0.0), 2.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 3 points. First one so we give it a weight.
    myPotentialField.addGoal(std::make_tuple(-0.75, 0.2), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 2 points.
    myPotentialField.addGoal(std::make_tuple(-0.85, 1.35), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 1 point.
    myPotentialField.addGoal(std::make_tuple(0.45, 1.3), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 1 point.
    // More accessory goals on the other side of the map.
    myPotentialField.addGoal(std::make_tuple(-0.75, -0.2 ), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 2 points.
    myPotentialField.addGoal(std::make_tuple(-0.85, -1.35), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 1 point.
    myPotentialField.addGoal(std::make_tuple(0.45, -1.3), 0.0,
                             weightSimpleBorder, weightOblicBorder, weightSample, weightRectangles);     // 1 point.



    return myPotentialField;
}





std::tuple<double, double> iterPotentialFieldWithLogFile(Potential_Field * myPotential_Field, double dt, std::ofstream &myFile) {
    if (myPotential_Field->GoalTest(0.08)) {
        return std::make_tuple(0.0, 0.0);
    } else {
        myPotential_Field->current_position = myPotential_Field->getPosition();
        std::tuple<double, double> myRepulsiveForce = myPotential_Field->totalRepulsiveForce();
        std::tuple<double, double> attractionForce = myPotential_Field->attractiveForce(
                myPotential_Field->current_position);
        std::tuple<double, double> mySpeed = myPotential_Field->getSpeedVector(dt, 0.84, 4.6666, myPotential_Field->current_position);
        myFile << tupleToString(myPotential_Field->current_position) << " ";
        myFile << tupleToString(myRepulsiveForce) << " ";
        myFile << tupleToString(attractionForce) << " ";
        myFile << tupleToString(mySpeed) << " ";
        myFile << tupleToString(myPotential_Field->currentSpeedVector) << "\n";
        if (std::get<0>(mySpeed) == 0 && std::get<1>(mySpeed) == 0){
            myPotential_Field->willNotMove = true;
        }else{
            myPotential_Field->willNotMove = false;
        }
        std::cout << "didnt move = \t"<< myPotential_Field->didntMove << "\n";
        std::cout << "didnt Rotate = \t"<< myPotential_Field->didntRotate << "\n";
        std::cout << "Goal test = \t" << myPotential_Field->GoalTest(0.08)<< "\n";

        myPotential_Field->isStuck= (myPotential_Field->didntMove > 20) && !(myPotential_Field->GoalTest(0.08)) && (myPotential_Field->didntRotate > 20);
        std::cout << "help step bro i m stuck = " << myPotential_Field->isStuck << "\n";
        return mySpeed;
    }
}

std::tuple<double, double> iterPotentialField(Potential_Field myPotential_Field, double dt) {
    if (myPotential_Field.GoalTest(0.08)) {
        return std::make_tuple(0.0, 0.0);
    } else {
        myPotential_Field.current_position = myPotential_Field.getPosition();
        std::tuple<double, double> myRepulsiveForce = myPotential_Field.totalRepulsiveForce();
        std::tuple<double, double> attractionForce = myPotential_Field.attractiveForce(
                myPotential_Field.current_position);
        std::tuple<double, double> mySpeed = myPotential_Field.getSpeedVector(dt, 0.84, 4.6666,
                                                                              myPotential_Field.current_position);
        return mySpeed;
    }
}



















// À ne pas copier en 2002
// À faire passer dans un autre fichier -> main_ctrl_gr4

// ===============
// == TEST PART ==
// ===============



std::tuple<std::tuple<double, double>, std::tuple<double, double>>
next_position(const Potential_Field &myPotential_Field, std::tuple<double, double> orientation,
              std::tuple<double, double> speed, double dt) {
    std::tuple<double, double> position = myPotential_Field.current_position;
    double Wref = std::get<1>(speed);
    double Vref = std::get<0>(speed);
    double dTheta = Wref * dt;
    double norm = sqrt(pow(std::get<0>(orientation), 2) + pow(std::get<1>(orientation), 2));

    //std::cout << "norm = " << norm << " ";

    std::tuple<double, double> nextSpeedVector = std::make_tuple(
            std::get<0>(orientation) * cos(dTheta) - std::get<1>(orientation) * sin(dTheta),
            std::get<0>(orientation) * sin(dTheta) + std::get<1>(orientation) * cos(dTheta));
    /*
    std::cout << "speed vector" << tupleToString(nextSpeedVector) << '\n';
    std::cout << "Wref = " << Wref << " Vref = " << Vref << '\n';
    */

    double next_x =  Vref / norm * std::get<0>(nextSpeedVector) * dt + std::get<0>(position);
    double next_y =  Vref / norm * std::get<1>(nextSpeedVector) * dt + std::get<1>(position);

    /*
    std::cout <<std::get<0>(position) << " " << std::get<1>(position) << "\n";
    std::cout << next_x << " " << next_y << "\n";
     */
    return std::make_tuple(std::make_tuple(next_x, next_y), nextSpeedVector);

}





// Test function : instantiate here to see if it works.
int main(int arg, char *argv[]) {

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                      TEST OF PATH                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /*

    // création des obstacles
    double hitBox = 0.05;
    double distanceOfInfluence = 0.25;
    std::tuple<double, double> center = std::make_tuple<double, double>(-0.25, 0.75);
    double k_rep = 0.05;

    Sample Sample1 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 1.0);
    Sample Sample2 = Sample(center, k_rep, distanceOfInfluence, hitBox);



    center = std::make_tuple<double, double>(0.0, 120.0);
    Sample Sample3 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 80.0);
    Sample Sample4 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 125.0);
    Sample Sample5 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 130.0);
    Sample Sample6 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 135.0);
    Sample Sample7 = Sample(center, k_rep, distanceOfInfluence, hitBox);

    center = std::make_tuple<double, double>(0.0, 140.0);
    Sample Sample8 = Sample(center, k_rep, distanceOfInfluence, hitBox);


    // creation potential field :
    std::tuple<double, double> goal_position = std::make_tuple<double, double>(0.5, 0.75);
    std::tuple<double, double> position = std::make_tuple<double, double>(-0.75, 0.6);
    Potential_Field myPotential_Field = Potential_Field(position);

    myPotential_Field.addGoal(goal_position, 2.0);

    std::tuple<double, double> initialSpeedVector = std::make_tuple<double, double>(1.0, 0.0);
    myPotential_Field.setSpeedVector(initialSpeedVector);


    // créatrion des listes :


    std::vector<SimpleBorder> simpleBorderList;   // List of simple border obstacle type.
    std::vector<OblicBorder> oblicBorderList;    // List of oblic border obstacle type.
    std::vector<Opponent> opponentList;       // List of opponent obstacle type.
    std::vector<Sample> sampleList;         // List of sample obstacle type.

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
     
    double dt = 0.1;

    std::ofstream myFile;
    myFile.open("data.txt", std::ios::out);
    if (!myFile) {
        std::cout << "not open";
    }

    int number_of_sub_steps = 1;
    int i = 0;

    while (i < 10000 &&
           (pow(std::get<0>(myPotential_Field.currentGoal.position) - std::get<0>(myPotential_Field.current_position),
                2) > 0.008 ||
            pow(std::get<1>(myPotential_Field.currentGoal.position) - std::get<1>(myPotential_Field.current_position),
                2) > 0.008)) {


        std::tuple<double, double> mySpeed = iterPotentialFieldWithLogFile(myPotential_Field, dt, myFile);
        std::tuple<std::tuple<double, double>, std::tuple<double, double>> result = next_position(myPotential_Field,
                                                                                                  myPotential_Field.currentSpeedVector,
                                                                                                  mySpeed, dt /
                                                                                                           number_of_sub_steps);
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
            myFile << tupleToString(myPotential_Field.current_position) << " ";
            myFile << tupleToString(myRepulsiveForce) << " ";
            myFile << tupleToString(attractionForce) << " ";
            myFile << tupleToString(mySpeed) << " ";
            myFile << tupleToString(myPotential_Field.currentSpeedVector) << "\n";

        }

        i++;

    }

    myFile.close();

    std::cout << "Finish";

    */




    // To be modified if we want to test.
    /*
    double result_forceApp = testObject.forceApplied(valeurqcq);

    cout << "Test if everything works fine" << endl;
    cout << "Position: " << std::tupleToString(testObject.position_vector) << endl; // << here only works with strings !
    cout << "Goal: " << std::tupleToString(testObject.goal_position) << endl;
    cout << "Force: " << to_string(result_forceApp) << endl;
*/

}

