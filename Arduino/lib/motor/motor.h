/**
 * @file motor.h
 * @author Téo Trény
 * @brief Ensemble de fonctions pour piloter les moteurs
 * @version 0.1
 * @date 2025-06-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
// #include <MeMegaPi.h>

// Déclaration anticipée car il y a des soucis avec MeMegaPi.h !!
class MeEncoderOnBoard;

#define MAX_RPM 150
#define RPM 80

/**
 * @brief Initialise les moteurs avec les paramètres.
 * 
 * @param m1 Moteur 1 (Droite ou gauche ?)
 * @param m2 Moteur 2 (Droite ou gauche ?)
 */
void motors_init(MeEncoderOnBoard * m1, MeEncoderOnBoard * m2);

/**
 * @brief Fait tourner les moteurs en sens avant.
 * 
 * @param rpm Vitesse de rotation des moteurs [tr/min]
 */
void consigneMoteur(int rpmg, int rpmd);
// /**
//  * @brief Fait tourner les moteurs en sens arrière.
//  * 
//  * @param rpm Vitesse de rotation des moteurs [tr/min]
//  */
// void motors_backward(int rpm);

// /**
//  * @brief Fait tourner les moteurs de manière à pivoter à gauche.
//  * 
//  * @param rpm Vitesse de rotation des moteurs [tr/min]
//  */
// void motors_left(int rpm);

// /**
//  * @brief Fait tourner les moteurs de manière à pivoter à droite.
//  * 
//  * @param rpm Vitesse de rotation des moteurs [tr/min]
//  */
// void motors_right(int rpm);

/**
 * @brief Arrête les moteurs
 * 
 */
void motors_stop();

#endif