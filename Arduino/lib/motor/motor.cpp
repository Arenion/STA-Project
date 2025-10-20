#include "motor.h"
#include <MeMegaPi.h>

static MeEncoderOnBoard* motor1 = nullptr;
static MeEncoderOnBoard* motor2 = nullptr;

int check_rpm(int rpm)
{
    if (rpm > MAX_RPM)
        return MAX_RPM;
    return rpm;
}

void motors_init(MeEncoderOnBoard* m1, MeEncoderOnBoard* m2)
{
    // On récupère les moteurs passés par argument pour pouvoir
    // les utiliser à travers toute cette bibliothèque
    motor1 = m1;
    motor2 = m2;

    // Configuration de l'encodeur
    // Nombre d'impulsions par tour
    motor1->setPulse(7);
    // Ratio de réduciton du moteur : il faut tourner 26.9 fois pour faire un tour
    motor1->setRatio(26.9);
    // Configuration des PID pour la position et la vitesse
    motor1->setPosPid(1.8, 0, 0.5); // Kp=1.8 et Kd=0.5
    motor1->setSpeedPid(0.18, 0, 0); // Kp = 0.18

    motor2->setPulse(7);
    motor2->setRatio(26.9);
    motor2->setPosPid(1.8, 0, 0.5);
    motor2->setSpeedPid(0.18, 0, 0);
}

void consigneMoteur(int rpmg, int rpmd)//rpmg est le rpm du moteur gauche et rpmd celui du moteur droit
{
    rpmg = check_rpm(rpmg);
    rpmd = check_rpm(rpmd);
    motor1->runSpeed(rpmg);
    motor2->runSpeed(rpmd);
}

// void motors_backward(int rpm)
// {
//     int r = check_rpm(rpm);
//     motor1->runSpeed(-r);
//     motor2->runSpeed(r);
// }

// void motors_left(int rpm)
// {
//     int r = check_rpm(rpm);
//     motor1->runSpeed(r);
//     motor2->runSpeed(r);
// }

// void motors_right(int rpm)
// {
//     int r = check_rpm(rpm);
//     motor1->runSpeed(-r);
//     motor2->runSpeed(-r);
// }

// void motors_stop()
// {
//     motor1->runSpeed(0);
//     motor2->runSpeed(0);
// }
