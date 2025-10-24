#include "global.h" 

struct infoarduino INFORMATIONARDUINO;
pthread_mutex_t MUTEX_INFORMATIONARDUINO;

struct position NEXTOBJECTIF = OBJECTIF_STOP;
pthread_mutex_t MUTEX_NEXTOBJECTIF;

struct position POSITION;//position actuelle de la voiture
pthread_mutex_t MUTEX_POSITION;

enum reservation RESERVATION = NONITINIALISE;
pthread_mutex_t MUTEX_RESERVATION;

pthread_mutex_t MUTEX_REUSSITEOBJECTIF;

float erreur_angulaire;
pthread_mutex_t MUTEX_ERREUR_ANGULAIRE;

float erreur_distance;
pthread_mutex_t MUTEX_ERREUR_DISTANCE;


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
int sdobjectifsuivant; //socket de demande de l'objectif suivant
int sddemandedereservation;//socket de la demande de la demande de reservation
int sdenvoiposition;//socket pour l'envoi de la position

int l1=10;//distance l1 en mm, à changer pour changer le calcul d'erreur
int k0=5;//coefficient k0 à changer pour changer le calcul d'erreur

void demandereservation(enum reservation areserver){
    static enum reservation derniere_demande = NONITINIALISE;

    // Pour éviter de spam la demande, si elle a déjà été faite, ne rien faire.
    if (derniere_demande && derniere_demande == areserver)
        return;

    printf("Envoie de la demande de réservation.\n");
    // Sinon faire la demande.
    switch (areserver)
    {
    case RESERVATIONRONDPOINT:
        send(sddemandedereservation,"1",1,0);
        break;
    case RESERVATPONT:
        send(sddemandedereservation,"2",1,0);
        break;
    case PASDERESERVATION:
        send(sddemandedereservation,"0",1,0);    
        break;
    }
    derniere_demande = areserver;
}

void annoncereussiteobjectif(){
    send(sdobjectifsuivant,"1",0,0);
}
