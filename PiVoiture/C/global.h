#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/un.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "marvelmind.c/src/marvelmind.h"
#include <sys/wait.h>
#include <time.h>
#include <signal.h>
#include <stdatomic.h>
#include <math.h>
#include <time.h>

#define START_BYTE -128 

#define CHECK_ERROR(val1, val2, msg) \
    if (val1 == val2)                \
    {                                \
        perror(msg);                 \
        exit(EXIT_FAILURE);          \
    }
#define MAX_CARS    80
#define PSEUDOFICHIER "/tmp/cam.sock" //emplacement du pseudo fichier
#define LENT 80 //RPM pour instruction de vitesse lente
#define RAPIDE 127 //RPM pour instruction de vitesse rapide
#define REMOTE_PORT 6000
#define REMOTE_IP   "192.168.8.241"
#define LOCAL_PORT 6000
#define rayon_roue 27 //rayon de la roue
#define largeur_voiture 113 //Largeur de la voiture entre deux roues
#define MY_HEDGE    18


enum demandeetat{
    DEMANDEORMAL,
    DEMANDEVIRAGE,
    DEMANDEINIT,
    DEMANDEGARER,
    DEMANDEOBSTACLE,
    DEMANDESTOP,
    DEMANDEDEPPASSEMENT,
};

enum etat{
    NORMAL,
    VIRAGE,
    INIT,
    GARER,
    OBSTACLE,
    STOP,
    DEPPASSEMENT,
};

static enum {
    GOTOPOINT_INIT,
    GOTOPOINT_DURING,
} ETATGOTOPOINT=GOTOPOINT_INIT;

enum reservation{
    NONITINIALISE = 0,
    RESERVATIONRONDPOINT,
    RESERVATPONT,
    PASDERESERVATION,
};

struct arg_socket{
    int Port;
    char message[MAX_CARS];
};
struct infoarduino{
    float ratio;
    int RPM;
};
struct position{
    int32_t x; // mm
    int32_t y; // mm
};

#define POINT_STOP ((struct position){-1, -1})
#define OBJECTIF_STOP POINT_STOP

extern struct infoarduino INFORMATIONARDUINO;                  
extern pthread_mutex_t MUTEX_INFORMATIONARDUINO;

extern struct position NEXTOBJECTIF;
extern pthread_mutex_t MUTEX_NEXTOBJECTIF;

extern struct position POSITION;//position actuelle de la voiture
extern pthread_mutex_t MUTEX_POSITION;

extern enum reservation RESERVATION;
extern pthread_mutex_t MUTEX_RESERVATION;

extern char INFOCAMERA[MAX_CARS-1];//données de la caméra, via une af_unix
extern int FD;
extern char donneecontroleur[MAX_CARS-1];//donnée du controleur, via une socket af_inet
extern enum demandeetat DEMANDEETAT;//état de la voiture
extern enum etat ETAT;

extern bool terminateProgram;
extern char vitesse[MAX_CARS-1];
extern char obstacle[MAX_CARS-1];
extern char panneau[MAX_CARS-1];
extern int8_t vitessemot[2];
extern int RPM;

extern int l1;//distance l1 en mm, à changer pour changer le calcul d'erreur
extern int k0;//coefficient k0 à changer pour changer le calcul d'erreur


extern int ANGLE;//Angle de la voiture, issu de l'arduino

void demandereservation(enum reservation areserver);
void annoncereussiteobjectif();

extern float erreur_angulaire;
extern pthread_mutex_t MUTEX_ERREUR_ANGULAIRE;

extern float erreur_distance;
extern pthread_mutex_t MUTEX_ERREUR_DISTANCE;

extern int sdobjectifsuivant; //socket de demande de l'objectif suivant
extern int sddemandedereservation;//socket de la demande de la demande de reservation
extern int sdenvoiposition;//socket pour l'envoi de la position