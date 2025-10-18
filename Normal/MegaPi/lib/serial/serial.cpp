/**
 * @file serial.cpp
 * @author Téo Trény
 * @version 0.1
 * @date 2025-06-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "serial.h"

int init_serial(unsigned long baud_rate)
{
    Serial2.begin(baud_rate);    // Initialiser la communication série avec le baud rate spécifié
    return 0; // Retourner 0 pour indiquer que l'initialisation a réussi
}

size_t read_serial(char * buffer, size_t max_buffer)
{
    Serial.println("Lecture série..."); // Afficher un message de débogage
    if (buffer == NULL || max_buffer == 0) {
        return -1; // Retourner -1 si le buffer est NULL ou de taille 0
    }
    
    size_t bytes_read = 0;
    while (Serial2.available() > 0) {
        Serial.println("Lecture en cours..."); // Afficher un message de débogage
        if (bytes_read >= max_buffer - 1) {
            break; // Arrêter la lecture si le buffer est plein
        }
        buffer[bytes_read] = Serial2.read(); // Lire les données du port série
        Serial.print("Caractère lu : ");
        Serial.println(buffer[bytes_read]);
        bytes_read++;
    }
    
    buffer[bytes_read] = '\0'; // Terminer la chaîne de caractères
    return bytes_read; // Retourner le nombre de caractères lus
}

int write_serial(const char * data)
{
    if (data == NULL) {
        return -1; // Retourner -1 si les données sont NULL
    }
    
    size_t length = strlen(data);
    for (size_t i = 0; i < length; i++) {
        Serial2.write(data[i]); // Écrire chaque caractère dans le port série
    }
    return length; // Retourner le nombre de caractères écrits
}

int flush_serial()
{
    Serial2.flush(); // Nettoie le buffer de sortie
    return 0; // Retourner 0 pour indiquer que le flush a réussi
}

int close_serial()
{
    Serial2.end(); // Terminer la communication série
    return 0; // Retourner 0 pour indiquer que la fermeture a réussi
}