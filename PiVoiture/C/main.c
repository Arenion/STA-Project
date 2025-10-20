
/*Application : gère la communication entre la raspberry de la voiture et le reste de l'environement, et controle la raspberry dans les différents modes de fonctionnement de la voiture*/ 


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


// Octets du protocole
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
struct nextobjectif{
    int x;
    int y;
};

void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]);//focntion transformant le ratio et le rpm en vitesse mot gauche et droite
void lecturedonneescamera(char pseudofich[MAX_CARS]);//connexion de socket de la caméra pour lire données: caméra -> voiture
void envoidedonne(char doneecam[MAX_CARS-1]);//fonction intérprétant les données et les envoie à l'arduino, c'est ici qu'on interprete dans quel mode on se trouve: voiture -> raspberry 
void receptioncontrolleur(struct arg_socket * arg);// fonction permettant de recevoir des données du controlleur: controlleur -> voiture
void envoicontrolleur(struct arg_socket * arg);//fonction envoyant des données 
void receptionposition();//fonction permettant de recevoir la postion via le marvelmind

void advance(int dist,int8_t RPM);//fonction avancer la voiture d'une distance dist en mm pour une rotation des moteurs de RPM
void turn(int theta, int8_t RPM);//fonction pour tourner d'un angle theta dans le sens des aiguilles d'une montre pour une rotation des moteurs de RPMs
// les deux codes précédent sont à utiliser dans deux situations: trajectoire suivant des points, et peut-être le dépassement 
void CtrlHandler(int signum);
void gestionfinprogramme();

struct infoarduino INFORMATIONARDUINO;
struct nextobjectif NEXTOBJECTIF;
char INFOCAMERA[MAX_CARS-1];//données de la caméra, via une af_unix
int FD;
char donneecontroleur[MAX_CARS-1];//donnée du controleur, via une socket af_inet
enum demandeetat DEMANDEETAT=DEMANDEINIT;//état de la voiture
enum etat ETAT=NORMAL;
int32_t position[2];//position actuelle de la voiture
bool terminateProgram=false;
char vitesse[MAX_CARS-1];
char obstacle[MAX_CARS-1];
char panneau[MAX_CARS-1];
int8_t vitessemot[2];
int RPM;

// Fonction pour convertir un entier baudrate → valeur termios
int get_baudrate(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default:
            fprintf(stderr, "Baudrate non supporté: %d\n", baud);
            return -1;
    }
}

// Fonction pour configurer le port série
int serial_open(const char *portname, int baudrate) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erreur ouverture port série");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Erreur tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0; // mode non canonique

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Erreur tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

// Envoi d’une commande au MegaPi
void send_command(int fd, int8_t rpmg, int8_t rpmd) {
    int8_t frame[3] = {-128, rpmg,rpmd };
    write(fd, frame, 3);
    tcdrain(fd);
    printf("Commande envoyée : %d %d\n", rpmg,rpmd);
}


int main(int argc, char *argv[]) {
    int vitessemot[2]={80,80};

    //initialisation des threads
    pthread_t thread_autorisationdepassement;// thread pour l'autorisation de depassement controlleur -> voiture
    pthread_t thread_objectifsuivant;// thread pour l'envoi d'un nouvel objectif controlleur -> voiture
    pthread_t thread_demandereservation;// thread pour la demande de reservation d'une section voiture -> controlleur
    pthread_t thrad_reussiteobjectif; // thread annonçant la réussite du dernier objectif voiture -> controlleur
    pthread_t thread_actuatlisationtronçon;// thread actualisant le tronçon de la voiture à l'aide d'informations de la caméra voiture -> contnrolleur
    pthread_t thread_socketcamera;
    pthread_t thread_sendcommandarduino;
    pthread_t thread_getposition;//thread permettant d'obtenir la position via le marvelmind
    pthread_t thread_stop; //thread gérant une commande stop du controlleur controlleur -> voiture 


    //initialisation des arguments des threads
    struct arg_socket autorisationdepassement={6000, 0};// pour le depassement on note 0 pour un refus et 1 pour une autorisation

    struct arg_socket objectifsuivant;//pour les objectifs, on utilise un arrray de 2 entiers de 32 bits
    objectifsuivant.Port=6001;
    char objdefault[MAX_CARS]="0 0";
    strcpy(objectifsuivant.message,objdefault);

    struct arg_socket demandestop={6002,"0"};// envoi de 1 pour une demande d'arrét

    struct arg_socket demandereservation={6000, "0"};// Pour la demande de reservation, on note juste un entier représentant le tronçon demandé

    struct arg_socket reussiteobjectif={6001, "0"};// 0 pour non réussite, 1 pour réussite
    
    struct arg_socket actualisationtronçon={6002,"0"};//même syntaxe que demandereservation
    

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <port> <baudrate>\n", argv[0]);
        fprintf(stderr, "Exemple: %s /dev/ttyAMA0 9600\n", argv[0]);
        return 1;
    }

    const char *port = argv[1];
    int baud_raw = atoi(argv[2]);
    int baud = get_baudrate(baud_raw);

    if (baud < 0) {
        return 1; // baudrate invalide
    }

    FD = serial_open(port, baud);
    if (FD < 0) {
        return 1;
    }
    printf("Ouverture de %s à %d bauds réussie.\n", port, baud_raw);
    
    //signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);

    //pthread_create(&thread_stop,NULL,receptioncontrolleur,(void*)&demandestop);
    pthread_create(&thread_socketcamera,NULL,lecturedonneescamera,PSEUDOFICHIER);
    pthread_create(&thread_sendcommandarduino,NULL,envoidedonne,NULL);
    //pthread_create(&thread_autorisationdepassement,NULL,receptioncontrolleur,(void*)&autorisationdepassement);
    //pthread_create(&thread_objectifsuivant,NULL,receptioncontrolleur,(void*)&objectifsuivant);
    //pthread_create(&thread_demandereservation,NULL,envoicontrolleur,(void*)&demandereservation);
    //pthread_create(&thrad_reussiteobjectif,NULL,envoicontrolleur,(void*)&reussiteobjectif);
    //pthread_create(&thread_actuatlisationtronçon,NULL,envoicontrolleur,(void*)&actualisationtronçon);
    //pthread_create(&thread_getposition,NULL,receptionposition,NULL);
    
   //pthread_join(thread_stop,NULL);
    pthread_join(thread_sendcommandarduino,NULL);
    pthread_join(thread_socketcamera,NULL);
    close(FD);
    return 0;
}

void lecturedonneescamera(char *pseudofich){
    int sd_lect;// socked de dialogue du lecteur
    struct sockaddr_un addr_lect,addr_ecriv;//Pour definir l'adresse du lecteur et recupérer l'adresse de l'émetteur
    int lg_addr; //pour spécifier la longueur en octet d'une adresse
    int nbcar;//déifni le nmobre d'octets lus
    char buff_lect[MAX_CARS-1]="debut";//buffer de récéption de data 
    int check_exit=1;
    char infocamera[MAX_CARS-1]="@";
    float ratio =0.5;
    //Etape 1 : Création du socket de lécture
    sd_lect=socket(AF_UNIX, SOCK_DGRAM,0);
    CHECK_ERROR(sd_lect, -1, "erreur de création de la socket \n");

    //Etape 2 : définition de l'adresse de la socket
    addr_lect.sun_family =AF_UNIX;
    strcpy(addr_lect.sun_path, pseudofich);
    //Etape 3 : affectation de l'adresse à la socket    
    int erreur =bind(sd_lect, (const struct sockaddr *) &addr_lect, sizeof(addr_lect.sun_family)+strlen(addr_lect.sun_path));//pour gérer certaines erreures;
    CHECK_ERROR( erreur, -1, "ERREUR DE BIND : vérifier si le pseudo_fichier n'existe pas");

    //Etape 4 : Mise en reception
    printf("RECEPTION LECTEUR CAMERA\n");
    // printf("%d\n",(strcmp(buff_lect,"exit\n")));
    while(!(check_exit==0)||(terminateProgram)){
        nbcar = recv(sd_lect,buff_lect, MAX_CARS,0);//On ne récupére pas l'adresse de l'écrivain
        if (nbcar) {
            strcpy(INFOCAMERA,buff_lect);
            printf("Infos caméra reçues: ");
            printf(INFOCAMERA);
            sscanf(INFOCAMERA,"%f %s %s %s",&ratio,vitesse,obstacle,panneau);
	        printf(" ratio %f\n",ratio);
	        printf("vitesse %s\n",vitesse);
            INFORMATIONARDUINO.ratio=ratio;
	        INFORMATIONARDUINO.RPM=LENT;
            //if (strcmp(vitesse,"LENT")) INFORMATIONARDUINO.RPM=LENT;
            //if (strcmp(vitesse,"RAPIDE"))INFORMATIONARDUINO.RPM=RAPIDE;
            //if (strcmp(vitesse,"STOP"))INFORMATIONARDUINO.RPM=0;       
            printf("\n");
        }
        if (strcmp(buff_lect,"exit")==0 ){
            check_exit=0;
        } 
    }
    printf("fin de LECTEUR CAMERA");

    //Etape 5: Fermeture du programme
    close (sd_lect);//fermeture de la socket
    unlink(addr_lect.sun_path); //supression du pseudo fichier créé sur le disque dur
    return;
}

void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]){
    float erreur=ratio-0.5;
    if (erreur>=0){
        vitessemot[0]=(int8_t)RPM;
        vitessemot[1]=(int8_t)(RPM*(1-3*erreur));
    };
    if(erreur<0){
        vitessemot[0]=(int8_t)(RPM*(1+3*erreur));
        vitessemot[1]=(int8_t)(RPM);
    };

}

void envoidedonne(char doneecam[MAX_CARS-1]){
    DEMANDEETAT=DEMANDEORMAL;

    while (1){
	usleep(100000);
    //if(ETAT=NORMAL){
	printf("etat normal");
	printf("%f, %d\n",INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM);
        ratRPMtovitmot(INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM,vitessemot);
        send_command(FD,vitessemot[0],vitessemot[1]);
    //   }
    if(ETAT=DEPPASSEMENT){
        turn(-45,INFORMATIONARDUINO.RPM);
        advance(1.4*200,INFORMATIONARDUINO.RPM);
        turn(45,INFORMATIONARDUINO.RPM);
        advance(100,INFORMATIONARDUINO.RPM);
        turn(45,INFORMATIONARDUINO.RPM);
        advance(1.4*200,INFORMATIONARDUINO.RPM);
        turn(-45,INFORMATIONARDUINO.RPM);


    }
        //if(ETAT=STOP){
        //    send_command(FD,0,0);
       // }
     }
}

void envoicontrolleur(struct arg_socket * arg)
{
    int remote_port =arg->Port;
    int sd1 ; //socket de dialogue
    
    struct sockaddr_in adrlect; //adresse du lecteur
    
    int erreur; 
    int nbcars; 
    
    char buff[MAX_CARS];
    
    //Etape 1 : creation de la socket
    
    sd1=socket(AF_INET, SOCK_STREAM,0); // La socket va utilise TCP
    
    printf("Descripteur de socket = %d \n", sd1); // Inutile
    
    CHECK_ERROR(sd1,-1, "erreur de creation de la socket !!! \n");
    
    // Etape 2  : On complete l'adresse de la socket
    
    adrlect.sin_family=AF_INET;
    adrlect.sin_port=htons(remote_port);
    adrlect.sin_addr.s_addr=inet_addr(REMOTE_IP);
    
    // Etape 3 : connexion vers le serveur
    
    erreur=connect(sd1, (const struct sockaddr *) &adrlect, sizeof(adrlect));
    
    CHECK_ERROR(erreur,-1, "La connexion n'a pas ete ouverte !!! \n");

    // Etape 4 : dialogue unidirectionnel vers le serveur
    do
    {
    
    
    //Etape 4 : Envoi de data
        
    strcmp(buff,arg->message);
    
    nbcars=send(sd1, buff, strlen(buff)+1, 0); 
    }
    while (strcmp(buff, "fin")!=0 ||(terminateProgram));
    
    
    getchar();
   
    close(sd1);
    
      
    return;
    
}



void receptioncontrolleur(struct arg_socket * arg)
{
    int local_port =arg->Port;
    int se ; //socket d'ecoute
    int sd1 ; //socket de dialogue
    
    int pid;
    
    struct sockaddr_in adrlect , adrecriv ; //adresses dans le domaine AF_INET
    
    int erreur; 
    int nbcars; 
    
    char buff[MAX_CARS];
    
    int nbfils=1;
    
    //Etape 1 : creation de la socket
    
    se=socket(AF_INET, SOCK_STREAM,0); // La socket va utilise TCP
    
    printf("Descripteur de socket = %d \n", se); // Inutile
    
    CHECK_ERROR(sd1,-1, "erreur de creation de la socket !!! \n");
    
    // Etape 2  : On complete l'adresse de la socket
    
    adrlect.sin_family=AF_INET;
    adrlect.sin_port=htons(local_port);
    adrlect.sin_addr.s_addr=INADDR_ANY; // Toutes adresse de la machine est valable
    
    
    // Etape 3 : On affecte une adresse a la socket d'ecoute
    
    erreur=bind(se, (const struct sockaddr *) &adrlect, sizeof(struct sockaddr_in));
    
    CHECK_ERROR(erreur,-1, "Echec de bind !!! Verifie que le pseudo fichier n'existe pas deja !!! \n");
    
    
    
    
    // Etape 4 : on se met l'ecoute des demandes des connexions
    
    listen(se, 8);
    
    // printf("Lecteur en attente de connexion !!! \n");
    
    // En attende de demande de connexions
    while (1)
    {
        
    sd1=accept(se, NULL, NULL); //Je ne recupere pas l'adresse de celui qui ouvre la connexion
    // printf("Nouvelle connexion accepte par le serveur !!!\n");
    
    pid=fork(); //Creation du processus fils
    
    //Etape 4
    
    if (pid) {
        //Je suis dans le pere
        close(sd1); // Je ferme la socket de dialogue utilisee par le fils
        nbfils++;
    }
    
    else
    {
        //Je suis dans le fils
    close(se);
    do
    {
    //(recvfrom(int sockfd, void *buf, size_t len, int flags, struct sockaddr *src_addr, socklen_t *addrlen);

    nbcars=recv(sd1, buff,MAX_CARS, 0) ; // Les deux NULL a la fin indique que je ne veux pas recuperer l'adresse de l'ecrivain

    if (nbcars){
        
        strcpy(arg->message,buff);
        if  (local_port=6000){//gestion des depassements
            printf("DEPASSEMENT AUTORISE\n");
            ETAT=DEPPASSEMENT;
            }
        if (local_port=6001){
            printf("OBJECTIF SUIVANT RECU\n");
            sscanf("%d %d",&(NEXTOBJECTIF.x),&(NEXTOBJECTIF.y));
            }
        if (local_port==6002){//gestion de la commande stop
            printf("ARRET DU SYSTEME\n");
            send_command(FD,0,0);//si on reçoit un message du thread stop on arréte les moteurs
            terminateProgram=true;//et on arréte les programmes
            }

    
    
        }   
    
    }
    while (strcmp(buff, "fin")!=0 ||(terminateProgram));
    
    // printf("Appuyez sur une touche pour quitter !!! \n");
    getchar();
    close(sd1);
    return;
    }
} // fin de la boucle accept du pere
    
 close(se); // Ne sert a rien car je n'arrive jamais ici !!!
    
    
    
}


void receptionposition()
{
    int se ; //socket d'ecoute
    int sd1 ; //socket de dialogue
    
    int pid;
    
    struct sockaddr_in adrlect , adrecriv ; //adresses dans le domaine AF_INET
    
    int erreur; 
    int size_rcvd; 
    
    struct PositionValue pos;
    
    int nbfils=1;
    
    //Etape 1 : creation de la socket
    
    se=socket(AF_INET, SOCK_STREAM,0); // La socket va utilise TCP
    
    printf("Descripteur de socket = %d \n", se); // Inutile
    
    CHECK_ERROR(sd1,-1, "erreur de creation de la socket !\n");
    
    // Etape 2  : On complete l'adresse de la socket
    
    adrlect.sin_family=AF_INET;
    adrlect.sin_port=htons(6003);
    adrlect.sin_addr.s_addr=INADDR_ANY; // Toutes adresse de la machine est valable
    
    
    // Etape 3 : On affecte une adresse a la socket d'ecoute
    
    erreur=bind(se, (const struct sockaddr *) &adrlect, sizeof(struct sockaddr_in));
    
    CHECK_ERROR(erreur,-1, "Echec de bind ! Verifie que le pseudo fichier n'existe pas deja.\n");
    
    
    
    
    // Etape 4 : on se met l'ecoute des demandes des connexions
    
    listen(se, 8);
    
    printf("Lecteur en attente de connexion\n");
    
    // En attende de demande de connexions
    while (1)
    {
        
    sd1=accept(se, NULL, NULL); //Je ne recupere pas l'adresse de celui qui ouvre la connexion
    printf("Nouvelle connexion accepte par le serveur\n");
    
    pid=fork(); //Creation du processus fils
    
    //Etape 4
    
    if (pid) {
        //Je suis dans le pere
        close(sd1); // Je ferme la socket de dialogue utilisee par le fils
        nbfils++;
    }
    
    else
    {
        //Je suis dans le fils
    close(se);
    do
    {
    if (size_rcvd) {
        position[0]=pos.x;
        position[1]=pos.y;
    }
    
    size_rcvd=recv(sd1, &pos, sizeof(pos), 0) ; // Les deux NULL a la fin indique que je ne veux pas recuperer l'adresse de l'ecrivain
    
    }
    while (strcmp((char *)&pos, "fin"));

    printf("Fermeture de LECTEUR %d\n", nbfils);
    close(sd1);
    return;
    }
} // fin de la boucle accept du pere
    
 close(se); // Ne sert a rien car je n'arrive jamais ici !!!
    
        
}
void advance(int distance, int8_t RPM){
    float vitesse= RPM*rayon_roue/60;//conversion de RPM en vitesse en mm/s
    int time= (int)(1000000*distance/vitesse);
    send_command(FD,RPM,RPM);
    usleep(time);
    send_command(FD,0,0);//on arréte les moteurs aprés avoir travérsé la distance
}

void turn(int theta, int8_t RPM){
    float vitesse= RPM*rayon_roue/60;//conversion de RPM en vitesse en mm/s
    float omega= 2*vitesse/largeur_voiture;//vitesse angulaire de la voiture en rad/s
    float thetarad =theta*3.14/180;//conversion de l'angle en degrés en radians
    int time =(int)(100000*theta/omega);
    send_command(FD,RPM,-RPM);
    usleep(time);
    send_command(FD,0,0);
}

void CtrlHandler(int signum)
{
    terminateProgram=true;
}
