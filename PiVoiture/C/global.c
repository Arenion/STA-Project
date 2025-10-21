#include "global.h" 

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

long tempsobstacle; //temps passé par la voiture à esquiver une obstacle
long tempsavancer;//temps pour avancer d'une certaine longueur
long tempsdebutavancer;//temps de début d'avancé en ligne droite
int RPM;
int ANGLE;//Angle de la voiture, issu de l'arduino
