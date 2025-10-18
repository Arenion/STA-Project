/**
 * @file protocole.h
 * @author Téo Trény
 * @brief Ensemble de fonctions et structures pour la gestion du protocole de communication
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PROTOCOLE_H
#define PROTOCOLE_H

#include <Arduino.h>
#include "serial.h"
#include "motor.h"

// Octets de contrôle du protocole
#define START_BYTE 0x02  // Octet de début de trame
#define STOP_BYTE 0x03   // Octet de fin de trame
#define FORWARD 'F' // Commande pour avancer
#define BACKWARD 'B' // Commande pour reculer
#define LEFT 'L' // Commande pour tourner à gauche
#define RIGHT 'R' // Commande pour tourner à droite
#define STOP 'S' // Commande pour s'arrêter

// Un message de protocole mesure 3 octets : START, CMD, STOP
// CMD est un caractère qui représente la commande à exécuter

void parse_serial_input();

void process_command(char cmd);
#endif