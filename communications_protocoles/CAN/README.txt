1) Registre des documents dans le dossier :
=======CAN=======
can.cpp          : mise en oeuvre de communication can via un code en C
hexa.cpp         : conversions de double en caractères hexadécimaux
can_functions    : combinaisons des deux fichiers ci-dessus pour un controle des moteurs via can

2) explication des messages CAN : 
Ici nous avons pris le parti d'envoyer les commandes CAN à partir de la commande cansend de la librairie can-utils. 
Les messages se sont composé de la sorte :
"cansend <device> <ID du destinataire>#<message>"

Dans le cadre de ce projet, nous communiquons uniquement avec un driver moteur possédant la puce mcp2515.
L'adresse utilisé est 708, et le device est can0.
Les messages (uniquement des modifications de valeurs de registres) serons composés de la sorte :
<addr><mask><value>
addr : adresse du registre
mask : bits modifiers dans le registre
value: valeur dont les bits correspndant aux bits modifiable du registre (indiqués par le mask) remplaceront les anciennes valeurs de celui-ci

Pour allumer la led :
cansend can0 708#1EFF40
Pour éteindre la led :
cansend can0 708#1EFF00

Pour initialiser un moteur :
cansend can0 708#1CFF80
Et modifier sa vitesse (Duty cycle des hasheurs):
cansend can0 708#25FFXX <- XX duty cycle du hasheur (22-> vitesse nulle, 00 -> vitesse maximale négative, 44 (et plus) -> vitesse maximale positive)

Pour le deuxieme moteur : 
cansend can0 708#1DFF80
cansend can0 708#26FFXX



