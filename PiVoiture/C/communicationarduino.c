#include "global.c"

void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]);//focntion transformant le ratio et le rpm en vitesse mot gauche et droite
void advance(int dist,int8_t RPM);//fonction avancer la voiture d'une distance dist en mm pour une rotation des moteurs de RPM
void turn(int theta, int8_t RPM);//fonction pour tourner d'un angle theta dans le sens des aiguilles d'une montre pour une rotation des moteurs de RPMs
void stopcommand();
void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]);
void *lignedroite(void *arg);
void send_command(int fd, int8_t rpmg, int8_t rpmd);



void send_command(int fd, int8_t rpmg, int8_t rpmd) {
    int8_t frame[3] = {-128, rpmg,rpmd };
    write(fd, frame, 3);
    tcdrain(fd);
    printf("Commande envoyée : %d %d\n", rpmg,rpmd);
}
void stopcommand(){
    send_command(FD,0,0);
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
void *lignedroite(void *arg){
    char * doneecam = (char *)arg;
    DEMANDEETAT=DEMANDEORMAL;

    while (1){
	usleep(100000);
    //if(ETAT=NORMAL){
	printf("etat normal");
    pthread_mutex_lock(&MUTEX_INFORMATIONARDUINO);
	printf("%f, %d\n",INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM);
    ratRPMtovitmot(INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM,vitessemot);
    pthread_mutex_unlock(&MUTEX_INFORMATIONARDUINO);
    send_command(FD,vitessemot[0],vitessemot[1]);

    //   }
    // if(ETAT=DEPPASSEMENT){
    //     pthread_mutex_lock(&MUTEX_INFORMATIONARDUINO);
    //     turn(-45,INFORMATIONARDUINO.RPM);
    //     advance(1.4*200,INFORMATIONARDUINO.RPM);
    //     turn(45,INFORMATIONARDUINO.RPM);
    //     advance(100,INFORMATIONARDUINO.RPM);
    //     turn(45,INFORMATIONARDUINO.RPM);
    //     advance(1.4*200,INFORMATIONARDUINO.RPM);
    //     turn(-45,INFORMATIONARDUINO.RPM);
    //     pthread_mutex_unlock(&MUTEX_INFORMATIONARDUINO);



    // }
        //if(ETAT=STOP){
        //    send_command(FD,0,0);
       // }
     }
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
