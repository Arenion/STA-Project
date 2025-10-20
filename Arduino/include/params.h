/**
 * @file params.h
 * @author Téo Trény
 * @brief Fichier contenant les paramètres du système
 * @version 0.1
 * @date 2025-06-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PARAMS_H
#define PARAMS_H

#define PWM_MAX 255.0                   // Valeur MAX PWM
#define V_MAX 12.0                      // Tension MAX
#define V2PWM ((PWM_MAX) / (V_MAX))     // Ratio PWM Tension
#define MOTOR_V_MAX 9.0                 // Tension max pour les moteurs
#define DT 5.0                          // Période d'échantillonnage en ms
#define BELT_PITCH 2.032                // pas de la courroie en cm
#define NTEETH 90.0                     // nombre de dents de la courroie
#define RADS2MMS BELT_PITCH*NTEETH / 6.28 // Ratio pour passer de radian à mm
#define PULSE_PER_ROUND 9               // Impulsion par tour
#define MM_PER_PULSE 0.49               // distance en mm par impulsion
#define DIAMETRE_ROUE 55.0              // Diamètre de la roue en mm
#define PERIMETRE_ROUE (DIAMETRE_ROUE * 3.14) // Périmètre de la roue en mm

#define ERREUR_MAX 0.5                  // Valeur max de l'erreur
#define DISTANCE_ARRET 50               // Distance d'arret en mm
#define Ka 0.5                          // Paramètre Ka (???)
#define KX 30                           // Paramètre KX (???)
#define KY 15                           // Paramètre KY (???)
#define KMOTOR 51.0                     // Paramètre KMOTOR (???)
#define LENGTH 230.0                    // Longueur (de quoi en quoi ?)
#define WIDTH 180.0                     // Largeur (de quoi en quoi ?)
#define RAYON 30                        // Rayon en mm (de quoi ?)

#endif