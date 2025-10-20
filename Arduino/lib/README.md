# Motor
Ensemble de fonctions permettant le contrôle des deux moteurs du robot.

# Serial
Ensemble de fonctions basiques pour une communication série par bus CAN.

# Protocole
Ensemble de fonctions permettant d'établir et respecter le protocole de communication entre le robot et le Raspberry PI. Une trame commence toujours par un octet '0x02' et se finit par un octet '0x03'. Il y a 5 types de message :
- 'F' : Forward
- 'B' : Backward
- 'L' : Left
- 'R' : Right
- 'S' : Stop
Donc si on veut par exemple faire tourner le robot à gauche, il faut envoyer la trame 0x02 | 0x4C (L) | 0x03
C'est un protocole relativement simpliste mais assez efficace, libre à vous de l'étoffer si besoin.