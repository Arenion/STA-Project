#include "global.h"
#include "communicationarduino.h"

void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]);//focntion transformant le ratio et le rpm en vitesse mot gauche et droite
void advance(int dist,int8_t RPM);//fonction avancer la voiture d'une distance dist en mm pour une rotation des moteurs de RPM
void initturn(int theta, int8_t RPM, int * timetoturn);//fonction pour tourner d'un angle theta dans le sens des aiguilles d'une montre pour une rotation des moteurs de RPMs
void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]);
long initgothrougharc(double longueur_arc, int diffang, int RPM);
float vitesseangulairepoint(int RPM,float erreurangulaire,float erreurdistance);//fonnction calculant la vitesse angulaire en fonction de l'erreur angulaire et l'erreur de distance


void send_command(int fd, int8_t rpmg, int8_t rpmd) {
    int8_t frame[3] = {-128, rpmg,-rpmd };
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

void initturn(int theta, int8_t RPM, int * timetoturn){
    float vitesse= RPM*rayon_roue/60;//conversion de RPM en vitesse en mm/s
    float omega= 2*vitesse/largeur_voiture;//vitesse angulaire de la voiture en rad/s
    float thetarad =theta*3.14/180;//conversion de l'angle en degrés en radians
    *timetoturn =(int)(100000*thetarad/omega);
    send_command(FD,RPM,-RPM);
//     usleep(time);
//     send_command(FD,0,0);
}

void lignedroite(struct map_node *node){
	printf("ligne droite\n");
    pthread_mutex_lock(&MUTEX_INFORMATIONARDUINO);
	printf("%f, %d\n",INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM);
    ratRPMtovitmot(INFORMATIONARDUINO.ratio,INFORMATIONARDUINO.RPM,vitessemot);
    pthread_mutex_unlock(&MUTEX_INFORMATIONARDUINO);
    send_command(FD,vitessemot[0],vitessemot[1]);
}

void ratRPMtovitmot(float ratio, int RPM, int8_t vitessemot[2]){

    if (ratio==0.5){

        vitessemot[0]=(int8_t)RPM;

        vitessemot[1]=(int8_t)RPM;//on prend l'opposé pour la vitesse du moteur droit car celui ci est monté à l'envers

    };

    if (ratio>0.5){

        vitessemot[0]=(int8_t)RPM;

        vitessemot[1]=(int8_t)(RPM-(ratio-0.5)*4*RPM);

    }

    if(ratio<0.5){

        vitessemot[0]=(int8_t)-RPM+4*RPM*ratio;

        vitessemot[1]=(int8_t)RPM;

    }

}

float calculratiosuivitraj(struct position debutsegment,struct position finsegment,struct position voiture){ // fonction calculant l'erreur(le ratio) de la trajectoire de la position de la  voiture par rapport àu segment souhaité
    int xv = voiture.x;
    int yv = voiture.y;
    int xd= debutsegment.x;
    int yd= debutsegment.y;
    int xf=finsegment.x;
    int yf=finsegment.y;
    int vecx= yf-yd;
    int vecy= xd-xf;
    int scal = (vecx*(xv-xd)+vecy*(yv-yd));
    float ratio = 0.5+(scal/abs(scal))*point_segment_distance2(debutsegment,finsegment,voiture)/205;
    return ratio;
}

void followtraj(struct map_node * NodeP){
	printf("suivi de points\n");
    struct map_node Node =*NodeP;
    struct position segment[2];
    int8_t vitessemot[2];
    pthread_mutex_lock(&MUTEX_POSITION);
    int xv = POSITION.x;
    int yv = POSITION.y;
    pthread_mutex_unlock(&MUTEX_POSITION);
    struct position voiture={xv,yv};
    struct line LINE =Node.line;
    // point_line_distance2(LINE, voiture, segment);
    // float ratio=calculratiosuivitraj(segment[0],segment[1],voiture);
    // pthread_mutex_lock(&MUTEX_INFORMATIONARDUINO);
    // int RPM= INFORMATIONARDUINO.RPM;
    // pthread_mutex_unlock(&MUTEX_INFORMATIONARDUINO);
    pthread_mutex_lock(&MUTEX_ERREUR_ANGULAIRE);
    float erang= erreur_angulaire;
    pthread_mutex_unlock(&MUTEX_ERREUR_ANGULAIRE);
    pthread_mutex_lock(&MUTEX_ERREUR_DISTANCE);
    float erdis =erreur_distance;
    pthread_mutex_unlock(&MUTEX_ERREUR_DISTANCE);
    float vitesseangulaire=vitesseangulairepoint(RPM,erdis,erang);
    float vitessemoy= RPM*rayon_roue/60;
    float vg= vitessemoy+largeur_voiture*vitesseangulaire;
    float vd= vitessemoy-largeur_voiture*vitesseangulaire;
    send_command(FD,(int8_t) 60*vg/(2*3.14*rayon_roue),(int8_t) 60*vd/(2*3.14*rayon_roue));

    // switch (ETATGOTOPOINT)
    // {
    // case  GOTOPOINT_INIT:
        
    //     int phi=(int)atan2(abs(xd-xv),abs(yd-yv))+180-atomic_load(&ANGLE);
    //     pthread_mutex_lock(&MUTEX_INFORMATIONARDUINO);
    //     int RPM=INFORMATIONARDUINO.RPM;
    //     pthread_mutex_unlock(&MUTEX_INFORMATIONARDUINO);
    //     tempsavancer =initgothrougharc(sqrt((xv-xd)*(xv-xd)+(yv-yd)*(yv-yd)), phi,RPM);//il faut changer la fonction calculant la distance 
    //     tempsdebutavancer = (long)time(NULL);
    //     ETATGOTOPOINT=GOTOPOINT_DURING;
    //     break;
    
    // case GOTOPOINT_DURING:
    //     if ((long)time(NULL)>tempsavancer+tempsdebutavancer+tempsobstacle){
    //         i++;
    //         tempsavancer=0;
    //         tempsobstacle=0;
    //     }
    //     break;
    // }
    //turn(phi,RPM);
    //advance(sqrt((xd-xv)*(xd-xv)+(yd-yv)*(yd-yv)),RPM);
}

long initgothrougharc(double longueur_arc, int diffang, int RPM){
    float vitesse= RPM*rayon_roue/60;//conversion de RPM en vitesse en mm/s
    long time=(long) longueur_arc/vitesse;
    float omega=diffang/time;
    send_command(FD,(int8_t)(60/rayon_roue)*(vitesse+largeur_voiture*omega),(int8_t)(60/rayon_roue)*(vitesse-largeur_voiture*omega));
    return time;
}

float vitesseangulairepoint(int RPM,float erreurangulaire,float erreurdistance){//fonnction calculant la vitesse angulaire en fonction de l'erreur angulaire et l'erreur de distance
    float vitesse= (rayon_roue*RPM*2*3.14/60);//on passe de rotation des vitesse des roues en tr/min à une vitesse en m/s
    float k =k0*cos(erreur_angulaire);//fonction chois arbitrairement
    float vitesseangulaire= ((-vitesse*tan(erreurangulaire)/l1)-(vitesse*k*erreurdistance/cos(erreur_angulaire)));
    return vitesseangulaire;
}