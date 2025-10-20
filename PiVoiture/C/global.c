// Octets du protocole
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>



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
#define REMOTE_IP   "127.0.0.1"
#define LOCAL_PORT 6000
#define rayon_roue 27 //rayon de la roue
#define largeur_voiture 113 //Largeur de la voiture entre deux roues


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

struct arg_socket{
    int Port;
    char message[MAX_CARS];
};
struct infoarduino{
    float ratio;
    int RPM;
};
struct position{
    int32_t x;
    int32_t y;
};

struct infoarduino INFORMATIONARDUINO;
pthread_mutex_t MUTEX_INFORMATIONARDUINO;

struct position NEXTOBJECTIF;
pthread_mutex_t MUTEX_NEXTOBJECTIF;

struct position POSITION;//position actuelle de la voiture
pthread_mutex_t MUTEX_POSITION;

char INFOCAMERA[MAX_CARS-1];//données de la caméra, via une af_unix
int FD;
char donneecontroleur[MAX_CARS-1];//donnée du controleur, via une socket af_inet
enum demandeetat DEMANDEETAT=DEMANDEINIT;//état de la voiture
enum etat ETAT=NORMAL;

bool terminateProgram=false;
char vitesse[MAX_CARS-1];
char obstacle[MAX_CARS-1];
char panneau[MAX_CARS-1];
int8_t vitessemot[2];
int RPM;
int ANGLE;//Angle de la voiture, issu de l'arduino