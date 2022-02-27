#include <cstdio>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>
#include <list>
#include <string>

using namespace std; // Might pose a problem later in the work on tuples.

typedef vector< tuple<double,double> > tuple_coord_list; // List of coordinates as tuples.
// Vector : basically works as a list. vector.push_back : add element at the end.

// The obstacles in the map will be represented as circles, just like the robot. 
// Borders will be represented as being rectangles.
#define rho_0_border 0.1; // Distance of influence of border set to 10 [cm]. Every border has the same distance of influence.
#define rho_0_circle 0.15; // Distance of influence of obstacle in map : the samples on the floor have a diameter of 15 [cm], so we say r_influence = 15 [cm].
// Every obstacle in the map has the same distance of influence.

// -> Potential problem with opponent robot, as it is bigger than the samples on the floor.



// For the reference in coordinate system : see the 'RepresentationCarte.png' file.


// Instantiation of class Potential Field : mother class of Attractive and Repulsive forces.
class Potential_Field
{
    // Access specifier: Public class accessible everywhere.
    public:

    // Data Members : the attribute of the class.
    tuple_coord_list obstacle_list;         // Position of obstacles on the board (not counting in borders).
    tuple <double, double> position_vector; // Current position of the robot.
    tuple <double, double> goal_position;   // Current goal position.
    double coefficient;

    // Adding new obstacles to the list of obstacles. This will allow us to put the number of known obstacles on the map at first and then update it with, typically, 
    // the position of the opponent !
    // Assumption : new obstacles will only be a moving robot or a fallen sample that the other robot may have mooved. 
    tuple_coord_list addObstacleCoord(tuple <double, double> newObstacleCoord){
        obstacle_list.push_back(newObstacleCoord);
        return obstacle_list;
    }

    void setObstacleInfluenceZone(){

    }

    void setBorderInfluenceZone(){

    }

    // Question tuteur : comment faire ? En gros, je visualise ceci : on a un cercle et dès qu'on détecte qu'on est dedans, on sait qu'on doit subir le potential field.
    // Problème : comment faire comprendre qu'on est dedans ? Retourner une liste de points qui approximent ce cercle ? Y'a un truc tout fait pour ça ? J'ai du mal à
    // imaginer comment je pourrais me dire "ok, là j'suis dans le cercle oula aled".
    // Et par rapport aux bords : par bloc : ok mais comment ? Et pour ça, ça semble plus simple de retourner le fait qu'on est dans une "zone interdite", et encore (bords penchés).


    // AJOUTER LA FONCTION OBSTACLE et cercle et rectangulaire
    // Implémentation primitive : on définit chaque obstacle comme un point : sur map : un seul point et on fait un cercle centré en ce point
    // Pour le bord : traitement spécial car on connait le bord à l'avance : faire par "blocs" comme ça on pourra enlever des portions pour les définir comme goal et tout.
    // SET RECTANGLES ET CERCLES (obstacles bords et internes)

};

class Attractive_Force : public Potential_Field
{
    public:

    // Constructor of class.
    Attractive_Force() // Test : soit faire la fonction directement ici avec l'argument à passer à l'instantiation ou alors on utilise dans la fonction main la fonction set.
    // Probablement plus propre de faire un set de l'objet de base avec des data et puis on utilise les fonctions de set pour update si nécessaire.
    {
        cout << "Default Constructor of class Attractive_Force" << endl; // endl = end_line => '\n' in Python.
        // cout means "character output" and wcout means "wide character output". std::cout format. Quick way of saying "print".
    }

    void setCoefficient(double value) {
        coefficient = value;
    }
    void setPosition(tuple <double, double> position){
        position_vector = position;
    }
    void setGoalPosition(tuple <double, double> current_goal){
        goal_position = current_goal; 
    }

    // Calculation of the total attractive force at the present point. We can specify a coefficient, but the positions must have been set prior to the call.
    double forceApplied(double value){
        setCoefficient(value);
        double k_att = coefficient;
        return -k_att * sqrt(   pow(get<0>(position_vector) - get<0>(goal_position), 2) + 
                                pow(get<1>(position_vector) - get<1>(goal_position), 2) ); 
    }

};

// ADD REPULSIVE_FORCE BUT FIRST : SEE NEXT COMMENT

// DEMANDER SI CA NE SERAIT PAS MIEUX DE TOUT FAIRE DANS UNE SEULE CLASSE, PARCE QU'EN VRAI J'AI L'IMPRESSION QUE JE ME CASSE LES COUILLES POUR RIEN. 


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

    Attractive_Force testObject;
    testObject.setPosition(position_1);
    testObject.setGoalPosition(goal_1);

    double result_forceApp = testObject.forceApplied(valeurqcq);

    cout << "Test if everything works fine" << endl;
    cout << "Position: " << tupleToString(testObject.position_vector) << endl; // << here only works with strings !
    cout << "Goal: " << tupleToString(testObject.goal_position) << endl;
    cout << "Force: " << to_string(result_forceApp) << endl;
}