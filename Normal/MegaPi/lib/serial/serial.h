/**
 * @file serial.h
 * @author Téo Trény
 * @brief Ensemble de fonctions pour la communication série depuis un Arduino
 * @version 0.1
 * @date 2025-06-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Initialise la communication série avec le baud rate spécifié.
 * 
 * @param baud_rate Baud rate pour la communication série
 * @return int 0 si l'initialisation a réussi, -1 en cas d'erreur
 */
int init_serial(unsigned long baud_rate);

/**
 * @brief Lit des données depuis le port série dans un buffer.
 * 
 * @param buffer Buffer où les données lues seront stockées
 * @param max_buffer Taille maximale du buffer
 * @return size_t Nombre de caractères lus, ou -1 en cas d'erreur
 */
size_t read_serial(char * buffer, size_t max_buffer);

/**
 * @brief Ecrit des données dans le port série.
 * 
 * @param data Buffer contenant les données à écrire
 * @return int Nombre de caractères écrits, ou -1 en cas d'erreur
 */
int write_serial(const char * data);

/**
 * @brief Vide le buffer de sortie du port série, en s'assurant que toutes les données sont envoyées.
 * 
 * @return int 0 si le flush a réussi, -1 en cas d'erreur
 */
int flush_serial();

/**
 * @brief Ferme la communication série.
 * 
 * @return int 0 si la fermeture a réussi, -1 en cas d'erreur
 */
int close_serial();

#endif