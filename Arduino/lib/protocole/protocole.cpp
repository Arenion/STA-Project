#include "protocole.h"

void parse_serial_input()
{
    // Fonctionnement :
    // On lit les données depuis le port série. Une trame de données
    // commence par un octet START et se finit par un octet STOP.
    // Entre ses deux octets, on a un message d'une taille à définir.
    // C'est ce message qu'il faut traiter et adapter le comportement en fonction.
    // Si on n'a pas l'octet START ou STOP, on ignore les données.

    // Enumération pour gérer l'état de la machine à états
    static enum {
        WAIT_START,
        READ_CMD_G,
        READ_CMD_D,
        WAIT_STOP
    } state = WAIT_START;
    
    static int rmd_g = 0; // Variable pour stocker la commande sur le moteur gauche
    static int rmd_d = 0; // Variable pour stocker la commande sur le moteur droit

    // Serial.print('.');
    while(Serial2.available() > 0) {
        int c = Serial2.read();
        // Serial.print("Caractère lu :");
        // Serial.println(c);

        switch(state) {
            case WAIT_START :
                if (c == START_BYTE)
                    state = READ_CMD_G; 
            break;

            case READ_CMD_G :
                rmd_g = c;
                state = READ_CMD_D;
            break;

            case READ_CMD_D :
                rmd_d = c;
                state = WAIT_STOP;
            break;

            case WAIT_STOP :
                if (c == STOP_BYTE)consigneMoteur(rmd_g,rmd_d);
                // Que stop ait été reçu ou non, on repart au début
                state = WAIT_START;
            break;
        }
    }
}

// void process_command(char cmd)
// {
//     switch(cmd) {
//         case FORWARD:
//             Serial.println("[process_command] FORWARD");
//             motors_forward(RPM);
//         break;

//         case BACKWARD:
//             Serial.println("[process_command] BACKWARD");
//             motors_backward(RPM);
//         break;

//         case LEFT:
//             Serial.println("[process_command] LEFT");
//             motors_left(RPM);
//         break;

//         case RIGHT:
//             Serial.println("[process_command] RIGHT");
//             motors_right(RPM);
//         break;

//         case STOP:
//             Serial.println("[process_command] STOP");
//             motors_stop();
//         break;

//         default:
//             Serial.print("[process_command] Commande inconnue : ");
//             Serial.print(cmd);
//             motors_stop(); // Par sécurité, on arrête les moteurs
//         break;    
//     }
// }