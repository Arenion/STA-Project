#include <Arduino.h>
#include <MeMegaPi.h>
#include "params.h"
#include "pins.h"
#include "serial.h"
#include "protocole.h"
#include "motor.h"

// Fonctions
void encoder1_pulse_handler();
void encoder2_pulse_handler();
void config_clock();

// Variables globales
char buffer[256];
MeEncoderOnBoard encoder1(SLOT1);
MeEncoderOnBoard encoder2(SLOT2);
int rpm = 80; // Vitesse de rotation en tours par minute

void setup()
{
    // Configuration des communications série
    Serial.begin(9600);
    init_serial(9600);
    Serial.println("Communication série initialisée");

    pinMode(LED_BUILTIN, OUTPUT);

    // Initialisation des interruptions des encodeurs
    attachInterrupt(encoder1.getIntNum(), encoder1_pulse_handler, RISING);
    attachInterrupt(encoder2.getIntNum(), encoder2_pulse_handler, RISING);

    // Configuration de l'horloge interne PWM
    config_clock();
    
    // Configuration des encodeurs
    motors_init(&encoder1, &encoder2);

    // Attendre 1sec
    delay(1000);
}

void loop()
{
    // Loop des encodeurs
    encoder1.loop();
    encoder2.loop();

    // Lit les entrées et adapte son comportement à la trame reçue
    parse_serial_input();

    delay(10);
}

void encoder1_pulse_handler()
{
    if (digitalRead(encoder1.getPortB()) == 0)
        encoder1.pulsePosMinus();
    else
        encoder1.pulsePosPlus();
}

void encoder2_pulse_handler()
{
    if (digitalRead(encoder2.getPortB()) == 0)
        encoder2.pulsePosMinus();
    else
        encoder2.pulsePosPlus();
}

void config_clock()
{
    // Configuration de l'horloge interne PWM à 8 MHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}